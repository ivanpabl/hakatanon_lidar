"""ObstacleDetector -- связывает шаги обработки кадра: путь -> полотно -> зона ->
объекты -> подтверждение -> решение о тревоге -- решение -- detection/decision.py
(status: stop | unknown | caution | clear). Ничего не знает про ROS 2: на вход
массивы x, y, z одного кадра (см. tunnel_od.pointcloud.parse_pointcloud2).

    det = ObstacleDetector()
    for i, msg in enumerate(stream):
        x, y, z = parse_pointcloud2(msg.data, msg.point_step, msg.fields)
        result = det.detect(x, y, z, refit_path=(i % 10 == 0))

Пересчёт пути (update_path) -- десятки мс, проверка кадра (check_frame) -- единицы мс,
поэтому путь можно пересчитывать не на каждом кадре.
"""
from collections import deque

import numpy as np

from ..geometry.ego_motion import S_RANGE as EGO_S_RANGE, EgoMotion
from ..geometry.bed import bed_at, estimate_bed_profile, estimate_floor_z, rail_top_at
from ..geometry.path import TrackPath, extend_path_by_walls, splice_far_axis
from ..geometry.rails import fit_path
from . import background
from .clustering import MIN_POINTS_FLOOR, MIN_POINTS_K, cluster_points, mark_edge_lines, min_points_at
from .decision import axis_curvature, decide
from .plausibility import CTX_HALF_WIDTH, Context, is_implausible, object_features
from .tracking import EvidenceTracker, Tracker
from .zone import RECT_DEFAULT, rect_zone, zone_mask

MIN_FRAME_POINTS = 50  # меньше -- кадр пустой/обрезанный: полотно и зону не оценить, статус unknown


class ObstacleDetector:
    """Состояние между кадрами: путь (рельсы), смещение головки рельса над
    полотном, треки объектов для подтверждения.

    method='zone' -- объект = точки внутри зоны над рельсами (zone);
    method='background' -- точки внутри зоны (по умолчанию background.ZONE), которые
    ближе к оси пути, чем фон тоннеля на этом угле, больше чем на bg_residual; где фон
    не оценить -- как method='zone' с прямоугольником RECT_DEFAULT."""

    def __init__(self, near_cutoff=2.0, max_range=250.0, half_width=1.0,
                 clearance=0.15, height=3.0, confirm_hits=3, confirm_window=5,
                 max_path_age=30, default_rail_offset=0.5, path_margin=0.0, zone=None,
                 min_points_k=MIN_POINTS_K, min_points_floor=MIN_POINTS_FLOOR,
                 method='zone', bg_residual=background.RESIDUAL_M, ego_motion=False,
                 tracker='evidence', evidence_threshold=2.2, evidence_decay=0.8, far_axis=True,
                 edge_lines=True, far_half_width=0.7, far_top=1.5, far_from=20.0, alarm_hold=3,
                 alarm_hold_min=5, path_hold=0, ego_check=True, ego_min_travel=4.0, ego_max_slope=-0.35,
                 features=True, plausibility=True, plausible_min_dist=60.0, plausible_behind_n=2,
                 persist_hits=6, persist_slope=(-1.25, -0.75), persist_edge_margin=0.25, min_sight_m=30.0,
                 sensor_axis_union_m=40.0, sensor_axis_max_dev=0.25,
                 far_acc_from=None, far_acc_floor=1.5, far_acc_min_hits=4, range_hold=0,
                 persist_min_top_m=0.0, front_lift_caution_m=None, front_lift_max_low_m=2.0,
                 stop_confirm_far_m=None, persist_min_top_lat_m=None):
        """Правила против ложных СТОП (fp_autopsy R1-R3), по умолчанию ВЫКЛЮЧЕНЫ:
        persist_min_top_m    R1: подтверждение по устойчивому треку (persist) только при верхе объекта
                             (height_m) >= этого; 0 -- выкл. Рекомендуется 0.5.
        persist_min_top_lat_m R1: правило по верху действует только при |lateral_m| >= этого (точки у головки
                             рельса); None -- для всех объектов. Рекомендуется 0.55 (куб 0,2 м на оси не теряется).
        front_lift_caution_m R2: front_maxh >= low_m + это и low_m < front_lift_max_low_m -> caution
                             (reason front_lift) вместо stop; None -- выкл. Рекомендуется 0.1.
        stop_confirm_far_m   R3: СТОП на d >= этого только если трек был СТОП и в прошлом кадре,
                             иначе caution (reason far_unconfirmed); None -- выкл. Рекомендуется 30."""
        self.min_sight_m = min_sight_m
        self.front_lift_caution_m = front_lift_caution_m
        self.front_lift_max_low_m = front_lift_max_low_m
        self.stop_confirm_far_m = stop_confirm_far_m
        self._prev_stop_ids = set()
        if method not in ('zone', 'background'):
            raise ValueError(method)
        if tracker not in ('hits', 'evidence'):
            raise ValueError(tracker)
        self.method = method
        self.sensor_axis_union_m = sensor_axis_union_m
        self.sensor_axis_max_dev = sensor_axis_max_dev
        self.features = features
        self.plausibility = plausibility
        self.plausible_min_dist = plausible_min_dist
        self.plausible_behind_n = plausible_behind_n
        self.far_axis = far_axis
        self.evidence_threshold = evidence_threshold
        self.edge_lines = edge_lines
        self.far_half_width = far_half_width
        self.far_top = far_top
        self.far_from = far_from
        self.bg_residual = bg_residual
        self.near_cutoff = near_cutoff
        self.max_range = max_range
        if zone is None:
            zone = background.ZONE if method == 'background' else rect_zone(clearance, height, half_width)
        self.zone = tuple(zone)
        self.half_width = max(hw for _, _, hw in self.zone)
        self.confirm_hits = confirm_hits
        self.confirm_window = confirm_window
        self.max_path_age = max_path_age
        self.path_margin = path_margin
        self.min_points_k = min_points_k
        self.min_points_floor = min_points_floor
        # Накопление свидетельства вдали (exp4), по умолчанию ВЫКЛЮЧЕНО:
        # far_acc_from (м, None -- выкл.): дальше этой дальности ожидаемый минимум точек НА КАДР --
        #   far_acc_floor вместо min_points_at (там 3): объект из 1-2 точек копит evidence по нескольким
        #   кадрам (EvidenceTracker, прогноз на путь поезда); чтобы одиночные повторы шума не подтверждались,
        #   там же нужно far_acc_min_hits кадров с объектом.
        # range_hold (кадров, 0 -- выкл.): граница «за концом оси» (beyond_path) -- наибольшая из дальностей
        #   оси за последние range_hold кадров, сдвинутых на путь поезда с тех пор: дальность оси вдали скачет
        #   100 <-> 200 м от кадра к кадру, и подтверждённый дальний объект получает ВНИМАНИЕ вместо СТОП
        #   в кадрах с короткой осью. Геометрия оси (коридор) остаётся текущей.
        self.far_acc_from = far_acc_from
        self.far_acc_floor = far_acc_floor
        self.range_hold = int(range_hold or 0)
        self._range_hist = deque(maxlen=max(self.range_hold, 1))
        self.rail_offset = default_rail_offset
        self._fit_fwd = None
        self._fitted_cl = None
        self._track_gauge = None
        self._path_frame = None
        self._bed = None
        self._rail_prof = None
        self._path_range = None
        self._offset_measured = False
        self._frame = 0
        self.path_hold = path_hold
        self._travel = 0.0
        self._seen_path_frame = None
        self._held_axis = None
        if tracker == 'evidence':
            self._tracker = EvidenceTracker(evidence_threshold, evidence_decay, min_hits=confirm_hits,
                                            hold=alarm_hold, hold_min=alarm_hold_min, ego_check=ego_check,
                                            ego_min_travel=ego_min_travel, ego_max_slope=ego_max_slope,
                                            far_min_hits=None if far_acc_from is None
                                            else (float(far_acc_from), int(far_acc_min_hits)),
                                            persist_hits=persist_hits, persist_slope=tuple(persist_slope),
                                            persist_max_lat=(None if persist_edge_margin is None
                                                             else max(hw for _, _, hw in self.zone) - persist_edge_margin),
                                            persist_min_top=persist_min_top_m,
                                            persist_min_top_lat=persist_min_top_lat_m)
            ego_motion = True
        else:
            self._tracker = Tracker(confirm_hits, confirm_window)
        self._ego = EgoMotion() if ego_motion else None
        self.speed = None
        self._displacement = None

    def update_path(self, x, y, z) -> bool:
        """Пересчёт пути по рельсам. True -- путь найден и обновлён;
        False -- держим предыдущий фит, пока он не старше max_path_age кадров."""
        pair, fit_fwd, cl = fit_path(x, y, z)
        if pair is None:
            return False
        self._fit_fwd, self._fitted_cl, self._track_gauge = fit_fwd, cl, pair[3]
        self._path_frame = self._frame

        _, left, right, _ = pair
        rf = np.concatenate([left['fwd'], right['fwd']])
        rz = np.concatenate([left['z'], right['z']])
        bins, inv = np.unique(rf, return_inverse=True)
        tor = np.array([np.median(rz[inv == k]) for k in range(len(bins))])
        if len(tor) >= 3:
            tor = np.median(np.stack([np.r_[tor[:1], tor[:-1]], tor, np.r_[tor[1:], tor[-1:]]]), axis=0)
        self._rail_prof = (bins, tor)

        bed = estimate_bed_profile(x, y, z, self._center_of_fwd)
        if bed is not None:
            mid = (bins > 5) & (bins < 25) if ((bins > 5) & (bins < 25)).sum() >= 5 else np.ones_like(bins, bool)
            offset = float(np.median(tor[mid] - bed_at(bins[mid], bed)))
            if 0.1 < offset < 0.8:
                if self._offset_measured:
                    self.rail_offset += 0.3 * (offset - self.rail_offset)
                else:
                    self.rail_offset, self._offset_measured = offset, True
            self._fit_fwd, self._fitted_cl = extend_path_by_walls(
                x, y, z, fit_fwd, cl, lambda f: self._tor_at(f, bed), far_extend=self.far_axis)
        self._path_range = float(self._fit_fwd[-1])
        return True

    def _hold_far_axis(self):
        """Новый путь пришёл: если он короче предыдущего, хвост предыдущего (сдвинутый на путь
        поезда) держится до path_hold кадров с момента, когда этот хвост был измерен."""
        held, far_frame = self._held_axis, self._frame
        if held is not None and self.path_hold and self._frame - held[2] <= self.path_hold:
            fwd, cl, spliced = splice_far_axis(self._fit_fwd, self._fitted_cl, held[0], held[1],
                                               self._travel - held[3])
            if spliced:
                self._fit_fwd, self._fitted_cl, self._path_range = fwd, cl, float(fwd[-1])
                far_frame = held[2]
        self._held_axis = (self._fit_fwd, self._fitted_cl, far_frame, self._travel)

    def _tor_at(self, fwd, bed):
        return rail_top_at(fwd, bed, self.rail_offset, self._rail_prof)

    def _path_valid(self):
        if self._fitted_cl is not None and self._frame - self._path_frame > self.max_path_age:
            self._fit_fwd = self._fitted_cl = self._track_gauge = self._rail_prof = None
            self._path_range = None
        return self._fitted_cl is not None

    def _center_of_fwd(self, fwd):
        if self._fitted_cl is None:
            return np.zeros_like(fwd)
        return np.interp(fwd, self._fit_fwd, self._fitted_cl)

    @property
    def path_range(self):
        """До какой дальности ось пути известна (None -- путь не найден)."""
        return self._path_range

    def track_path(self) -> TrackPath:
        """Снимок текущей геометрии пути (копии массивов). Для размещения синтетических
        объектов и отрисовки; имеет смысл после хотя бы одного check_frame/detect."""
        cp = lambda a: None if a is None else a.copy()
        pair = lambda p: None if p is None else (p[0].copy(), p[1].copy())
        return TrackPath(cp(self._fit_fwd), cp(self._fitted_cl), pair(self._bed), pair(self._rail_prof),
                         self.rail_offset, self._path_range)

    def check_frame(self, x, y, z, stamp=None) -> dict:
        """Быстрый путь, каждый кадр: использует последний известный путь."""
        self._frame += 1
        path_ok = self._path_valid()
        if len(x) < MIN_FRAME_POINTS:
            return self._unknown_frame(path_ok, stamp)
        if path_ok and self._path_frame != self._seen_path_frame:
            self._seen_path_frame = self._path_frame
            self._hold_far_axis()

        bed = estimate_bed_profile(x, y, z, self._center_of_fwd)
        if bed is not None:
            self._bed = bed
        elif self._bed is None:
            self._bed = (np.array([0.0]), np.array([estimate_floor_z(x, y, z) - self.rail_offset]))

        fwd = -y
        m = (fwd > self.near_cutoff) & (fwd < self.max_range)
        fwd, x, z = fwd[m], x[m], z[m]
        lat = x - self._center_of_fwd(fwd)
        all_pts = (fwd, lat, z)
        if self._ego is not None:
            e = fwd < EGO_S_RANGE[1] + self._ego._max_shift
            self.speed, self._displacement = self._ego.update(
                fwd[e], lat[e], z[e] - self._tor_at(fwd[e], self._bed), stamp)
            self._travel += self._displacement or 0.0
        lat = self._axis_union(fwd, x, lat)
        if self.method == 'zone':
            m = np.abs(lat) < self.half_width
            fwd, lat, z, x = fwd[m], lat[m], z[m], x[m]
            z_rel = z - self._tor_at(fwd, self._bed)
            m = zone_mask(lat, z_rel, self.zone)
        else:
            z_rel = z - self._tor_at(fwd, self._bed)
            rails_end = self._rail_prof[0][-1] if self._rail_prof is not None else 0.0
            hw = np.where(fwd <= rails_end, self.half_width, min(self.half_width, background.FAR_HALF_WIDTH))
            in_zone = (np.abs(lat) < hw) & zone_mask(lat, z_rel, self.zone)
            res = background.background_residual(fwd, lat, z_rel, in_zone, self.max_range)
            unknown = np.isnan(res)
            thr = background.residual_threshold(fwd, self.bg_residual)
            m = in_zone & ((~unknown & (res > thr)) | (unknown & zone_mask(lat, z_rel, RECT_DEFAULT)))

        objects = cluster_points(fwd[m], lat[m], z_rel[m], keep_idx=self.features)
        for o in objects:
            o['too_small'] = o['n_points'] < self._expected_points(o['distance_m'])
        rails_end = self._rail_prof[0][-1] if self._rail_prof is not None else 0.0
        far_start = max(rails_end, 20.0) + self.far_from
        if self.edge_lines:
            mark_edge_lines(objects, far_start=far_start)
        for o in objects:
            o['edge_line'] = o.get('edge_line', False) or bool(o['distance_m'] > far_start and (
                (self.far_half_width is not None and abs(o['lateral_m']) > self.far_half_width)
                or (self.far_top is not None and o['low_m'] > self.far_top)))
        if hasattr(self._tracker, 'persist_max_d'):
            self._tracker.persist_max_d = far_start
        self._tracker.update([o for o in objects if not o.get('edge_line')], self._displacement,
                             self._expected_points, travel=self._travel if self._displacement is not None else None)
        for o in objects:
            o.setdefault('confirmed', False)
            o['held'] = False
        if self.features:
            self._add_features(objects, all_pts, np.arctan2(z[m], np.hypot(x[m], fwd[m])))
            if self.plausibility:
                for o in objects:
                    if 'behind_n' in o:
                        o['implausible'] = is_implausible(o, self.plausible_min_dist, self.plausible_behind_n)
        if objects:
            rz = self._tor_at(np.array([o['distance_m'] for o in objects]), self._bed)
            for o, r in zip(objects, rz):
                o['rail_z_m'] = float(r)
        for tr in self._tracker.held(min_distance=self.near_cutoff):
            d = float(tr['distance_m'])
            objects.append({'distance_m': d, 'far_m': d, 'lateral_m': float(tr['lateral_m']),
                            'height_m': tr.get('height_m', 0.0), 'low_m': tr.get('low_m', 0.0), 'n_points': 0,
                            'too_small': False, 'edge_line': False, 'confirmed': True, 'held': True,
                            'evidence': round(tr['score'], 2), 'track_id': tr['id'],
                            'ego_slope': None, 'ego_carried': False,
                            'level': tr.get('level', 'stop'), 'reason': tr.get('reason', 'in_gauge'),
                            'rail_z_m': float(self._tor_at(np.array([d]), self._bed)[0])})
        self._add_output_fields(objects)
        path_range = self._path_range if path_ok else None
        limit = (self._held_range(path_range) or 0.0) + self.path_margin
        for o in objects:
            o['beyond_path'] = o['distance_m'] > limit and not o['held']
        curvature = axis_curvature(self._fit_fwd, self._fitted_cl) if path_ok else 0.0
        dec = decide(objects, path_available=path_ok, path_range=path_range, curvature=curvature,
                     pending_score=0.5 * self.evidence_threshold, min_sight_m=self.min_sight_m,
                     front_lift_m=self.front_lift_caution_m, front_lift_max_low_m=self.front_lift_max_low_m,
                     stop_confirm_far_m=self.stop_confirm_far_m, prev_stop_ids=self._prev_stop_ids)
        self._prev_stop_ids = dec['stop_ids']
        for o in objects:
            o.pop('_idx', None)
        self._tracker.set_alarm({o['track_id'] for o in objects if o['level'] == 'stop' and 'track_id' in o})
        self._tracker.set_levels({o['track_id']: (o['level'], o['reason']) for o in objects
                                  if 'track_id' in o and not o['held']})
        return {
            'status': dec['status'],
            'obstacle': dec['status'] == 'stop',
            'distance_m': dec['distance_m'],
            'caution_distance_m': dec['caution_distance_m'],
            'sight_m': dec['sight_m'],
            'clear_to_m': dec['clear_to_m'],
            'n_points': int(m.sum()),
            'path_available': path_ok,
            'gauge_m': self._track_gauge,
            'path_age_frames': None if not path_ok else self._frame - self._path_frame,
            'path_range_m': path_range,
            'objects': objects,
            'speed_mps': self.speed,
            'displacement_m': self._displacement,
            'travel_m': self._travel,
            'stamp': stamp,
        }

    def _expected_points(self, d):
        """Ожидаемый минимум точек объекта на кадр для накопления свидетельства (EvidenceTracker):
        дальше far_acc_from -- far_acc_floor, иначе min_points_at."""
        if self.far_acc_from is not None and d > self.far_acc_from:
            return self.far_acc_floor
        return min_points_at(d, self.min_points_k, self.min_points_floor)

    def _held_range(self, path_range):
        """Граница beyond_path с учётом range_hold: max(текущая дальность оси, дальности оси прошлых
        кадров минус путь поезда с тех пор). range_hold=0 -- как есть; без оценки пути поезда
        (displacement None) история сбрасывается."""
        if not self.range_hold:
            return path_range
        if path_range is None or self._displacement is None:
            self._range_hist.clear()
            return path_range
        held = max((r - (self._travel - t) for t, r in self._range_hist), default=path_range)
        self._range_hist.append((self._travel, path_range))
        return max(path_range, held)

    def _axis_union(self, fwd, x, lat):
        """sensor_axis_union_m: ближе этой дальности точка в зоне, если она в зоне от оси пути ИЛИ
        от оси лидара (x=0) -- пока ось пути на участке от лидара не отходит от оси лидара дальше
        sensor_axis_max_dev (на прямой оси расходятся из-за разворота лидара; в кривой -- только ось
        пути). Объединение полос [c-hw, c+hw] и [-hw, hw] -- полоса вокруг отрезка [min(c,0), max(c,0)]:
        lat = x - clip(x, min(c,0), max(c,0)) -- непрерывно; зона, кластеры, края и persist -- от неё."""
        if self.sensor_axis_union_m is None or self._fitted_cl is None:
            return lat
        f, cl = self._fit_fwd, self._fitted_cl
        bad = (f <= self.sensor_axis_union_m) & (np.abs(cl) > self.sensor_axis_max_dev)
        to = float(min(self.sensor_axis_union_m, f[-1], f[bad][0] if bad.any() else np.inf))
        c = x - lat
        u = fwd < to
        return np.where(u, x - np.clip(x, np.minimum(c, 0.0), np.maximum(c, 0.0)), lat)

    def _unknown_frame(self, path_ok, stamp):
        """Результат той же структуры для пустого/крошечного кадра: габарит не проверить.
        Память R3 (_prev_stop_ids) не сбрасывается: треки в трекере тоже живут через такой кадр."""
        return {
            'status': 'unknown',
            'obstacle': False,
            'distance_m': None,
            'caution_distance_m': None,
            'sight_m': 0.0,
            'clear_to_m': 0.0,
            'n_points': 0,
            'path_available': path_ok,
            'gauge_m': self._track_gauge,
            'path_age_frames': None if not path_ok else self._frame - self._path_frame,
            'path_range_m': self._path_range if path_ok else None,
            'objects': [],
            'speed_mps': self.speed,
            'displacement_m': None,
            'travel_m': self._travel,
            'stamp': stamp,
        }

    def _add_output_fields(self, objects):
        """Размер, уверенность и центр объекта в системе облака (обратное к fwd=-y, lat=x-ось(fwd))."""
        for o in objects:
            has_edges = 'lat_min_m' in o and not o.get('held')
            o['size_m'] = None if not has_edges else {
                'length': round(o['far_m'] - o['distance_m'], 2),
                'width': round(o['lat_max_m'] - o['lat_min_m'], 2),
                'height': round(o['height_m'] - o['low_m'], 2)}
            ev = o.get('evidence')
            o['confidence'] = 1.0 if o.get('confirmed') else (
                0.0 if ev is None else round(min(1.0, ev / self.evidence_threshold), 2))
            f = 0.5 * (o['distance_m'] + o.get('far_m', o['distance_m']))
            lat = 0.5 * (o['lat_min_m'] + o['lat_max_m']) if has_edges else o['lateral_m']
            z = o.get('rail_z_m', 0.0) + 0.5 * (o.get('low_m', 0.0) + o.get('height_m', 0.0))
            o['position_m'] = {'x': round(float(self._center_of_fwd(np.array([f]))[0]) + lat, 2),
                               'y': round(-f, 2), 'z': round(float(z), 2)}

    def _add_features(self, objects, all_pts, elev):
        """Признаки правдоподобности (plausibility.object_features) объектам с треком."""
        todo = [o for o in objects if 'track_id' in o and not o.get('edge_line')]
        if not todo:
            return
        f, l, z = all_pts
        c = np.abs(l) < CTX_HALF_WIDTH
        f, l, z = f[c], l[c], z[c]
        ctx = Context(f, l, z - self._tor_at(f, self._bed))
        for o in todo:
            o.update(object_features(o, elev[o['_idx']], ctx))

    def detect(self, x, y, z, refit_path=False, stamp=None) -> dict:
        """Единая точка входа. refit_path=True -- пересчитать путь в этом же вызове."""
        if refit_path:
            self.update_path(x, y, z)
        return self.check_frame(x, y, z, stamp=stamp)
