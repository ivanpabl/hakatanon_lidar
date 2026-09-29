# © 2026 команда «Голуби», github.com/ivanpabl/hakatanon_lidar. Все права защищены, условия — в файле LICENSE. GLB-K5-4660c47a8c6c
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
FAR_TAIL_M = 30.0
FAR_MIN_RADIUS = 250.0
FAR_MIN_PTS = 15
FAR_MAX_GAP = 30.0
FAR_STEP_M = 20.0
TREND_M = 30.0


def _wall_dist(lat, side):
    """Расстояние до ближайшей поверхности с одной стороны: 10-й перцентиль |lat|."""
    d = lat * side
    d = d[(d > WALL_MIN_LAT) & (d < WALL_MAX_LAT)]
    return np.percentile(d, 10) if len(d) >= 3 else None


def _local_trend(af, ac):
    """Прогноз оси вперёд по опорным точкам за последние TREND_M метров: парабола
    (кривизна не круче FAR_MIN_RADIUS), при коротком хвосте -- прямая. Прямая в кривой
    отстаёт: на R=400м за 40м продления ошибка доходит до 0.3-0.5м -- коридор в стене."""
    f, c = np.asarray(af), np.asarray(ac)
    t = f >= f[-1] - TREND_M
    if t.sum() < 2:
        return lambda s: np.full_like(np.asarray(s, float), c[-1])
    fe = f[-1]
    if t.sum() >= 5 and f[t][0] <= fe - 15.0:
        c2, c1, _ = np.polyfit(f[t] - fe, c[t], 2)
        c2max = 1.0 / (2 * FAR_MIN_RADIUS)
        if abs(c2) > c2max:
            c2 = np.clip(c2, -c2max, c2max)
            c1 = np.polyfit(f[t] - fe, c[t] - c2 * (f[t] - fe) ** 2, 1)[0]
    else:
        c2, c1 = 0.0, np.polyfit(f[t] - fe, c[t], 1)[0]
    return lambda s: c[-1] + c1 * (np.asarray(s) - fe) + c2 * (np.asarray(s) - fe) ** 2


def _extend_far(ff, xf, af, ac, offsets, far):
    """Дальше пошагового продления стены видны редкими столбцами залпа (на 150м один
    столбец 0.1° покрывает ~20м стены), и бин по WALL_BIN_M часто пуст. Здесь ось
    продолжается одной параболой: значение и наклон -- с хвоста уже известной оси,
    кривизна (ограничена радиусом FAR_MIN_RADIUS) -- по всем точкам стен дальше
    сразу. Конец оси -- последняя точка стены без разрыва больше FAR_MAX_GAP."""
    af, ac = list(af), list(ac)
    s0 = af[-1]
    tf, tc = np.array(af), np.array(ac)
    t = tf >= s0 - FAR_TAIL_M
    if t.sum() < 3 or tf[t][0] > s0 - 10.0:
        return af, ac
    c2_max = 1.0 / (2 * FAR_MIN_RADIUS)
    c1 = np.polyfit(tf[t] - s0, tc[t], 1)[0]
    c0 = ac[-1]
    q = tf >= s0 - 2 * FAR_TAIL_M
    c2 = float(np.clip(np.polyfit(tf[q] - s0, tc[q], 2)[0], -c2_max, c2_max)) if q.sum() >= 5 else 0.0
    sel = ff > s0
    s, x = ff[sel] - s0, xf[sel]
    if len(s) < FAR_MIN_PTS:
        return af, ac

    def pick(limit, tol):
        pred = c0 + c1 * s + c2 * s ** 2
        best_d, best_c = np.full(len(s), np.inf), np.zeros(len(s))
        for side, off in offsets.items():
            c = x - side * off
            d = np.abs(c - pred)
            b = d < best_d
            best_d[b], best_c[b] = d[b], c[b]
        return (best_d < tol + 0.002 * s) & (s < limit), best_c

    def fit_c2(used, cen):
        su, cu = s[used], cen[used]
        return float(np.clip(np.sum((cu - c0 - c1 * su) * su ** 2) / np.sum(su ** 4), -c2_max, c2_max))

    used = np.zeros(len(s), bool)
    for limit in np.arange(FAR_STEP_M, far - s0 + FAR_STEP_M, FAR_STEP_M):
        u, cen = pick(limit, 0.5)
        if u.sum() >= FAR_MIN_PTS:
            c2, used = fit_c2(u, cen), u
    for tol in (0.35, 0.25):
        u, cen = pick(np.inf, tol)
        if u.sum() < FAR_MIN_PTS:
            break
        c2, used = fit_c2(u, cen), u
    if used.sum() < FAR_MIN_PTS:
        return af, ac
    su = np.sort(s[used])
    gaps = np.where(np.diff(np.r_[0.0, su]) > FAR_MAX_GAP)[0]
    s_end = su[gaps[0] - 1] if len(gaps) and gaps[0] > 0 else (0.0 if len(gaps) else su[-1])
    s_end = min(s_end, far - s0)
    for sk in np.arange(WALL_BIN_M, s_end + 1e-6, WALL_BIN_M):
        af.append(s0 + sk); ac.append(c0 + c1 * sk + c2 * sk ** 2)
    return af, ac


def extend_path_by_walls(x, y, z, fit_fwd, cl, tor_fn, far=250.0, far_extend=True):
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
        predict = _local_trend(af, ac)
        pred = predict(fc)
        lo, hi = np.searchsorted(ff, [f0, f0 + WALL_BIN_M])
        lat = xf[lo:hi] - predict(ff[lo:hi])
        cands, at = [], []
        for side, off in offsets.items():
            d = _wall_dist(lat, side)
            if d is not None and abs(d - off) < WALL_TOL:
                band = np.abs(lat * side - d) < 0.3
                sf = float(np.median(ff[lo:hi][band]))
                cands.append(float(predict(sf)) + side * (d - off)); at.append(sf)
        if cands and np.mean(at) > af[-1] + 0.5:
            af.append(float(np.mean(at))); ac.append(float(np.mean(cands)))
            misses = 0
        else:
            misses += 1
            if misses >= WALL_MAX_MISSES:
                break
    if far_extend:
        af, ac = _extend_far(ff, xf, af, ac, offsets, far)
    if af[-1] <= f_end:
        return fit_fwd, cl
    ext_f = np.array(af); ext_c = np.array(ac)
    tail = ext_f > f_end
    c_t = ext_c[tail]
    if len(c_t) >= 3:
        c_t = np.convolve(np.r_[2 * c_t[0] - c_t[1], c_t, 2 * c_t[-1] - c_t[-2]], np.ones(3) / 3, mode='valid')
    return np.concatenate([fit_fwd, ext_f[tail]]), np.concatenate([cl, c_t])


def splice_far_axis(new_fwd, new_cl, old_fwd, old_cl, shift, min_gain=5.0, overlap=20.0, max_dlat=0.3):
    """Продление короткой новой оси хвостом предыдущей: (fwd, cl, склеено ли).

    Дальность оси по стенам от кадра к кадру скачет (на стоящем поезде -- от 50 до 200 м), и
    объект за концом короткой оси тревогу не поднимает. Предыдущая ось сдвигается на путь
    поезда shift; хвост берётся, если она длиннее новой на min_gain и на последних overlap
    метрах новой оси расходится с ней по медиане не больше max_dlat (иначе это другая ось,
    например за стрелкой). Хвост сдвигается вбок так, чтобы на стыке не было ступеньки."""
    of = np.asarray(old_fwd) - shift
    end = new_fwd[-1]
    if of[-1] < end + min_gain:
        return new_fwd, new_cl, False
    ov = (new_fwd >= end - overlap) & (new_fwd >= of[0])
    if ov.sum() < 3 or np.median(np.abs(new_cl[ov] - np.interp(new_fwd[ov], of, old_cl))) > max_dlat:
        return new_fwd, new_cl, False
    tail = of > end
    off = new_cl[-1] - np.interp(end, of, old_cl)
    return np.concatenate([new_fwd, of[tail]]), np.concatenate([new_cl, np.asarray(old_cl)[tail] + off]), True


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
