"""Ось пути дальше видимых рельсов и снимок геометрии пути (TrackPath).

Рельсы видны до 25-40м, а в кривых именно дальше этого коридор "по прямой"
заходит на стену -- главный источник ложных тревог. Стены (или край
платформы) видны до 115-160м и идут параллельно пути: где рельсы известны,
меряем расстояние от оси до ближайшей поверхности слева/справа, а дальше
ведём ось шагами по WALL_BIN_M так, чтобы это расстояние сохранялось.
Точки ближе WALL_MIN_LAT к оси (сам коридор) в поиск стены не входят --
препятствие на пути не может "сдвинуть стену".
"""
from dataclasses import dataclass
from typing import Optional, Tuple

import numpy as np

from .bed import rail_top_at

WALL_BIN_M = 4.0
WALL_BAND = (0.5, 2.5)
WALL_MIN_LAT = 1.1
WALL_MAX_LAT = 4.5
WALL_TOL = 0.3
WALL_MAX_MISSES = 3


def _wall_dist(lat, side):
    """Расстояние до ближайшей поверхности с одной стороны: 10-й перцентиль |lat|."""
    d = lat * side
    d = d[(d > WALL_MIN_LAT) & (d < WALL_MAX_LAT)]
    return np.percentile(d, 10) if len(d) >= 3 else None


def extend_path_by_walls(x, y, z, fit_fwd, cl, tor_fn, far=250.0):
    """Продлевает ось (fit_fwd, cl) дальше рельсов по стенам. tor_fn(fwd) --
    высота головки рельса. Возвращает (fwd, center) -- исходный участок + продление."""
    fwd = -y
    f_end = fit_fwd[-1]
    sel = (fwd > fit_fwd[0]) & (fwd < far)
    fwd, x, z = fwd[sel], x[sel], z[sel]
    h = z - tor_fn(fwd)
    sel = (h > WALL_BAND[0]) & (h < WALL_BAND[1])
    fwd, x = fwd[sel], x[sel]

    near = fwd <= f_end
    lat_near = x[near] - np.interp(fwd[near], fit_fwd, cl)
    offsets = {}
    for side in (1, -1):
        ds = []
        for f0 in np.arange(fit_fwd[0], f_end, WALL_BIN_M):
            b = (fwd[near] >= f0) & (fwd[near] < f0 + WALL_BIN_M)
            d = _wall_dist(lat_near[b], side)
            if d is not None:
                ds.append(d)
        if len(ds) >= 3 and np.std(ds) < 0.3:
            offsets[side] = float(np.median(ds))
    if not offsets:
        return fit_fwd, cl

    af = list(fit_fwd[::10]) + [fit_fwd[-1]]
    ac = list(cl[::10]) + [cl[-1]]
    far_sel = fwd > f_end
    ff, xf = fwd[far_sel], x[far_sel]
    order = np.argsort(ff)
    ff, xf = ff[order], xf[order]
    misses = 0
    for f0 in np.arange(f_end, far, WALL_BIN_M):
        fc = f0 + WALL_BIN_M / 2
        k = min(len(af), 6)
        slope = np.polyfit(af[-k:], ac[-k:], 1)[0]
        pred = ac[-1] + slope * (fc - af[-1])
        lo, hi = np.searchsorted(ff, [f0, f0 + WALL_BIN_M])
        lat = xf[lo:hi] - pred
        cands = []
        for side, off in offsets.items():
            d = _wall_dist(lat, side)
            if d is not None and abs(d - off) < WALL_TOL:
                cands.append(pred + side * (d - off))
        if cands:
            af.append(fc); ac.append(float(np.mean(cands)))
            misses = 0
        else:
            misses += 1
            if misses >= WALL_MAX_MISSES:
                break
    if af[-1] <= f_end:
        return fit_fwd, cl
    ext_f = np.array(af); ext_c = np.array(ac)
    tail = ext_f > f_end
    c_t = ext_c[tail]
    if len(c_t) >= 3:
        c_t = np.convolve(np.r_[c_t[0], c_t, c_t[-1]], np.ones(3) / 3, mode='valid')
    return np.concatenate([fit_fwd, ext_f[tail]]), np.concatenate([cl, c_t])


@dataclass
class TrackPath:
    """Снимок геометрии пути на момент кадра (копия, не меняется вместе с детектором).

    fit_fwd/center -- ось пути (None: путь не найден, ось -- прямая x=0);
    bed -- профиль дна лотка (fwd, z); rail_prof -- высота головки по найденным
    рельсам (fwd, z); path_range -- до какой дальности ось известна."""
    fit_fwd: Optional[np.ndarray]
    center: Optional[np.ndarray]
    bed: Optional[Tuple[np.ndarray, np.ndarray]]
    rail_prof: Optional[Tuple[np.ndarray, np.ndarray]]
    rail_offset: float
    path_range: Optional[float]

    def center_at(self, fwd):
        """Боковое положение оси пути на дальностях fwd. За концом оси -- последнее значение."""
        fwd = np.asarray(fwd, float)
        if self.center is None:
            return np.zeros_like(fwd)
        return np.interp(fwd, self.fit_fwd, self.center)

    def rail_top_at(self, fwd):
        """Высота головки рельса на дальностях fwd."""
        return rail_top_at(np.atleast_1d(np.asarray(fwd, float)), self.bed, self.rail_offset, self.rail_prof)
