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
import numpy as np

from ..geometry.ego_motion import S_RANGE as EGO_S_RANGE, EgoMotion
from ..geometry.bed import bed_at, estimate_bed_profile, estimate_floor_z, rail_top_at
from ..geometry.path import TrackPath, extend_path_by_walls, splice_far_axis
from ..geometry.rails import fit_path
from . import background
from .clustering import MIN_POINTS_FLOOR, MIN_POINTS_K, cluster_points, mark_edge_lines, min_points_at
from .decision import axis_curvature, decide
from .tracking import EvidenceTracker, Tracker
from .zone import RECT_DEFAULT, rect_zone, zone_mask


class ObstacleDetector:
    """Состояние между кадрами: путь (рельсы), смещение головки рельса над
    полотном, треки объектов для подтверждения.

    method='zone' -- объект = точки внутри зоны над рельсами (zone);
    method='background' -- точки внутри зоны (по умолчанию background.ZONE), которые
    ближе к оси пути, чем фон тоннеля на этом угле, больше чем на bg_residual; где фон
    не оценить -- как method='zone' с прямоугольником RECT_DEFAULT."""

    def __init__(self, near_cutoff=2.0, max_range=250.0, half_width=1.0,
                 clearance=0.15, height=2.0, confirm_hits=3, confirm_window=5,
                 max_path_age=30, default_rail_offset=0.5, path_margin=0.0, zone=None,
                 min_points_k=MIN_POINTS_K, min_points_floor=MIN_POINTS_FLOOR,
                 method='zone', bg_residual=background.RESIDUAL_M, ego_motion=False,
                 tracker='evidence', evidence_threshold=2.2, evidence_decay=0.8, far_axis=True,
                 edge_lines=True, far_half_width=0.7, far_top=1.5, far_from=20.0, alarm_hold=3,
                 alarm_hold_min=5, path_hold=0):
        if method not in ('zone', 'background'):
            raise ValueError(method)
        if tracker not in ('hits', 'evidence'):
            raise ValueError(tracker)
        self.method = method
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
        # удержание дальней части оси: хвост предыдущей оси, если новая короче (splice_far_axis).
        # Делается в check_frame, а не в update_path: путь поезда известен только здесь, а
        # update_path в узле ROS считается в отдельном процессе (path_worker)
        self.path_hold = path_hold
        self._travel = 0.0
        self._seen_path_frame = None
        self._held_axis = None           # (fwd, cl, кадр, когда хвост измерен, путь поезда тогда)
        if tracker == 'evidence':
            self._tracker = EvidenceTracker(evidence_threshold, evidence_decay, min_hits=confirm_hits,
                                            hold=alarm_hold, hold_min=alarm_hold_min)
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
        if self._ego is not None:
            e = fwd < EGO_S_RANGE[1] + self._ego._max_shift
            self.speed, self._displacement = self._ego.update(
                fwd[e], lat[e], z[e] - self._tor_at(fwd[e], self._bed), stamp)
            self._travel += self._displacement or 0.0
        if self.method == 'zone':
            m = np.abs(lat) < self.half_width
            fwd, lat, z = fwd[m], lat[m], z[m]
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

        objects = cluster_points(fwd[m], lat[m], z_rel[m])
        for o in objects:
            o['too_small'] = o['n_points'] < min_points_at(o['distance_m'], self.min_points_k, self.min_points_floor)
        rails_end = self._rail_prof[0][-1] if self._rail_prof is not None else 0.0
        far_start = max(rails_end, 20.0) + self.far_from
        if self.edge_lines:
            mark_edge_lines(objects, far_start=far_start)
        # за концом найденных рельсов ось и высота полотна -- оценка: объект у края
        # коридора или целиком под сводом там скорее стена/свод при ошибке в десятки см.
        # Фильтр по объектам, а не по точкам: иначе рвутся цепочки edge_line
        for o in objects:
            o['edge_line'] = o.get('edge_line', False) or bool(o['distance_m'] > far_start and (
                (self.far_half_width is not None and abs(o['lateral_m']) > self.far_half_width)
                or (self.far_top is not None and o['low_m'] > self.far_top)))
        self._tracker.update([o for o in objects if not o.get('edge_line')], self._displacement,
                             lambda d: min_points_at(d, self.min_points_k, self.min_points_floor))
        for o in objects:
            o.setdefault('confirmed', False)
            o['held'] = False
        if objects:
            rz = self._tor_at(np.array([o['distance_m'] for o in objects]), self._bed)
            for o, r in zip(objects, rz):
                o['rail_z_m'] = float(r)
        # объект с тревогой пропал на кадр-два (вдали на нём 2-3 точки): тревога держится
        # alarm_hold кадров по прогнозу трека, объект помечается held
        for tr in self._tracker.held():
            d = float(tr['distance_m'])
            objects.append({'distance_m': d, 'far_m': d, 'lateral_m': float(tr['lateral_m']),
                            'height_m': tr.get('height_m', 0.0), 'low_m': tr.get('low_m', 0.0), 'n_points': 0,
                            'too_small': False, 'edge_line': False, 'confirmed': True, 'held': True,
                            'evidence': round(tr['score'], 2), 'track_id': tr['id'],
                            'level': tr.get('level', 'stop'), 'reason': tr.get('reason', 'in_gauge'),
                            'rail_z_m': float(self._tor_at(np.array([d]), self._bed)[0])})
        path_range = self._path_range if path_ok else None
        limit = (path_range if path_range is not None else 0.0) + self.path_margin
        for o in objects:
            # удержанный объект поднимал тревогу в пределах оси и с тех пор только приблизился;
            # короткая ось в этом кадре (её дальность скачет на 50-200 м) тревогу не снимает
            o['beyond_path'] = o['distance_m'] > limit and not o['held']
        curvature = axis_curvature(self._fit_fwd, self._fitted_cl) if path_ok else 0.0
        dec = decide(objects, path_available=path_ok, path_range=path_range, curvature=curvature,
                     pending_score=0.5 * self.evidence_threshold)
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

    def detect(self, x, y, z, refit_path=False, stamp=None) -> dict:
        """Единая точка входа. refit_path=True -- пересчитать путь в этом же вызове."""
        if refit_path:
            self.update_path(x, y, z)
        return self.check_frame(x, y, z, stamp=stamp)
