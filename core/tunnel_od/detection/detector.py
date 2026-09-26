"""ObstacleDetector -- связывает шаги обработки кадра: путь -> полотно -> зона ->
объекты -> подтверждение -> решение о тревоге. Ничего не знает про ROS 2: на вход
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
from ..geometry.path import TrackPath, extend_path_by_walls
from ..geometry.rails import fit_path
from . import background
from .clustering import MIN_POINTS_FLOOR, MIN_POINTS_K, cluster_points, min_points_at
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
                 max_path_age=30, default_rail_offset=0.5, path_margin=10.0, zone=None,
                 min_points_k=MIN_POINTS_K, min_points_floor=MIN_POINTS_FLOOR,
                 method='zone', bg_residual=background.RESIDUAL_M, ego_motion=False,
                 tracker='hits', evidence_threshold=2.2, evidence_decay=0.8, far_axis=True):
        if method not in ('zone', 'background'):
            raise ValueError(method)
        if tracker not in ('hits', 'evidence'):
            raise ValueError(tracker)
        self.method = method
        self.far_axis = far_axis
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
        if tracker == 'evidence':
            self._tracker = EvidenceTracker(evidence_threshold, evidence_decay, min_hits=confirm_hits)
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
        self._tracker.update(objects, self._displacement,
                             lambda d: min_points_at(d, self.min_points_k, self.min_points_floor))
        for o in objects:
            o.setdefault('confirmed', False)
        path_range = self._path_range if path_ok else None
        limit = (path_range if path_range is not None else 0.0) + self.path_margin
        for o in objects:
            o['beyond_path'] = o['distance_m'] > limit
        confirmed = [o for o in objects if o['confirmed'] and not o['beyond_path']]
        return {
            'obstacle': bool(confirmed),
            'distance_m': min(o['distance_m'] for o in confirmed) if confirmed else None,
            'n_points': int(m.sum()),
            'path_available': path_ok,
            'gauge_m': self._track_gauge,
            'path_age_frames': None if not path_ok else self._frame - self._path_frame,
            'path_range_m': path_range,
            'objects': objects,
            'speed_mps': self.speed,
            'displacement_m': self._displacement,
            'stamp': stamp,
        }

    def detect(self, x, y, z, refit_path=False, stamp=None) -> dict:
        """Единая точка входа. refit_path=True -- пересчитать путь в этом же вызове."""
        if refit_path:
            self.update_path(x, y, z)
        return self.check_frame(x, y, z, stamp=stamp)
