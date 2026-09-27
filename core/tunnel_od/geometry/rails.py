"""Поиск пары рельсов своего пути и центральной линии пути по одному кадру лидара.

Рельсы -- локальные максимумы поперечного профиля высоты в 1-метровых срезах по
дальности (find_floor_bumps). Пара ищется там, где она однозначна: вблизи и по
бокам от лидара (поезд стоит на своём пути), а дальше каждый рельс ведётся по
одному срезу, принимая только пик рядом с предсказанием (trace_rail_pair).
Прежний подход -- склеивать пики по боковой координате на всём участке 2-40м --
в кривых и у мусора сцеплял рельс с соседними линиями, и в записях со
станциями/стрелками выбиралась чужая пара.
"""
import warnings

import numpy as np

NEAR, FAR = 2.0, 40.0
HALF_WIDTH = 3.5
LAT_BIN = 0.05
FWD_BIN = 1.0
PROMINENCE = 0.05

_LAT_EDGES = np.arange(-HALF_WIDTH, HALF_WIDTH + LAT_BIN, LAT_BIN)
_N_LAT = len(_LAT_EDGES) + 1
_LAT_CENTERS = np.concatenate([[_LAT_EDGES[0] - LAT_BIN], _LAT_EDGES])
_BASELINE_W = 10
_PEAK_W = 3


def _sliding(a, w, fill):
    """Окна длины 2w+1 вдоль последней оси, края дополнены fill (nan = "нет данных")."""
    pad = np.full(a.shape[:-1] + (w,), fill)
    return np.lib.stride_tricks.sliding_window_view(np.concatenate([pad, a, pad], axis=-1), 2 * w + 1, axis=-1)


def find_floor_bumps(x, y, z, return_z=False):
    """Кандидаты в головки рельсов: локальные максимумы поперечного профиля высоты
    над медианной подложкой, в каждом 1-метровом срезе по дальности.
    Возвращает (fwd, lat, prominence[, z_top]) найденных пиков."""
    fwd = -y
    m = (fwd > NEAR) & (fwd < FAR) & (np.abs(x) < HALF_WIDTH)
    xf, zf, fwdf = x[m], z[m], fwd[m]
    empty = (np.array([]),) * (4 if return_z else 3)
    if len(xf) < 100:
        return empty

    f_starts = np.arange(NEAR, FAR, FWD_BIN)
    fbin = np.floor((fwdf - NEAR) / FWD_BIN).astype(np.int64)
    n_f = len(f_starts)
    ok = (fbin >= 0) & (fbin < n_f)
    xf, zf, fbin = xf[ok], zf[ok], fbin[ok]

    # одна сортировка по (срез, высота): из неё и перцентиль среза, и максимум ячейки
    zmin = zf.min()
    order = np.argsort(fbin * 1000.0 + (zf - zmin), kind='stable')
    fb_s, z_s, x_s = fbin[order], zf[order], xf[order]
    counts = np.bincount(fb_s, minlength=n_f)
    starts = np.concatenate([[0], np.cumsum(counts)[:-1]])
    zlo = np.full(n_f, np.nan)
    have = counts >= 50
    zlo[have] = z_s[starts[have] + (counts[have] - 1) * 30 // 100]
    keep = np.isfinite(zlo[fb_s]) & (z_s < zlo[fb_s] + 0.9)
    xf, zf, fbin = x_s[keep], z_s[keep], fb_s[keep]
    row_ok = np.bincount(fbin, minlength=n_f) >= 30

    lbin = np.digitize(xf, _LAT_EDGES)
    profile = np.full(n_f * _N_LAT, -np.inf)
    # точки идут по возрастанию высоты внутри среза: при повторе индекса остаётся
    # последняя запись, то есть максимум ячейки (быстрее np.maximum.at в разы)
    profile[fbin * _N_LAT + lbin] = zf
    profile = profile.reshape(n_f, _N_LAT)
    profile[np.isneginf(profile)] = np.nan
    valid = ~np.isnan(profile)
    row_ok &= valid.sum(axis=1) >= 15
    profile, valid = profile[row_ok], valid[row_ok]
    f_rows = f_starts[row_ok]
    if len(f_rows) == 0:
        return empty

    with warnings.catch_warnings():
        warnings.simplefilter('ignore', RuntimeWarning)
        baseline = np.nanmedian(_sliding(profile, _BASELINE_W, np.nan), axis=-1)
        local_max = np.nanmax(_sliding(profile, _PEAK_W, np.nan), axis=-1)
    prominence = profile - baseline
    with np.errstate(invalid='ignore'):
        is_peak = valid & (prominence > PROMINENCE) & (profile >= local_max)

    ri, li = np.nonzero(is_peak)
    out = (f_rows[ri] + FWD_BIN / 2, _LAT_CENTERS[li], prominence[ri, li])
    return out + (profile[ri, li],) if return_z else out


SEED_FWD = 10.0
SEED_GAUGE = (1.45, 1.75)
SEED_MAX_MID = 0.4
SEED_WIN = 0.1
SEED_MIN_SUPPORT = 3
TRACE_TOL = 0.12
TRACE_MAX_GAP = 8.0


def _trace_rail(pf, pl, pz, seed_idx):
    """Ведёт один рельс от затравочных пиков вперёд. Возвращает индексы пиков."""
    acc = list(seed_idx)
    for f in np.unique(pf[pf > pf[acc].max()]):
        af, al = pf[acc], pl[acc]
        if f - af.max() > TRACE_MAX_GAP:
            break
        deg = 2 if af.max() - af.min() > 15 else 1
        pred = np.polyval(np.polyfit(af, al, deg), f) if len(set(af)) > deg else np.median(al)
        cand = np.where(pf == f)[0]
        err = np.abs(pl[cand] - pred)
        k = np.argmin(err)
        if err[k] < TRACE_TOL + 0.005 * (f - af.max()):
            acc.append(cand[k])
    acc = np.array(acc)
    return {'fwd': pf[acc], 'lat': pl[acc], 'z': pz[acc], 'x_center': float(np.median(pl[acc]))}


def trace_rail_pair(pf, pl, pz):
    """Пара рельсов своего пути: (score, left, right, gauge) или None."""
    near = np.where(pf < SEED_FWD)[0]
    if len(near) < 2 * SEED_MIN_SUPPORT:
        return None
    ln = pl[near]
    support = (np.abs(ln[:, None] - ln[None, :]) < SEED_WIN).sum(axis=1)
    i, j = np.triu_indices(len(near), 1)
    gap = np.abs(ln[j] - ln[i])
    mid = (ln[i] + ln[j]) / 2
    ok = (gap > SEED_GAUGE[0]) & (gap < SEED_GAUGE[1]) & (np.abs(mid) < SEED_MAX_MID) \
        & (support[i] >= SEED_MIN_SUPPORT) & (support[j] >= SEED_MIN_SUPPORT)
    if not ok.any():
        return None
    score = np.where(ok, np.minimum(support[i], support[j]) - 10 * np.abs(mid), -np.inf)
    b = np.argmax(score)
    a_lat, b_lat = sorted((ln[i[b]], ln[j[b]]))
    rails = []
    for lat0 in (a_lat, b_lat):
        seed = near[np.abs(ln - lat0) < SEED_WIN]
        rails.append(_trace_rail(pf, pl, pz, seed))
    left, right = rails
    gauge = float(np.median(right['lat'][right['fwd'] < SEED_FWD]) - np.median(left['lat'][left['fwd'] < SEED_FWD]))
    return (float(score[b]), left, right, gauge)


def fit_path(x, y, z):
    """Пара рельсов своего пути + квадратичная центральная линия по одному кадру.
    Returns (pair_or_None, fit_fwd, centerline) -- centerline is None if no pair found."""
    pf, pl, ph, pz = find_floor_bumps(x, y, z, return_z=True)
    if len(pf) == 0:
        return None, None, None
    pair = trace_rail_pair(pf, pl, pz)
    if pair is None:
        return None, None, None
    _, left, right, gauge = pair
    fit_fwd = np.linspace(max(left['fwd'].min(), right['fwd'].min()),
                          min(left['fwd'].max(), right['fwd'].max()), 100)
    cl = (_rail_poly(left, fit_fwd) + _rail_poly(right, fit_fwd)) / 2
    return pair, fit_fwd, cl


def _rail_poly(rail, f):
    deg = 2 if rail['fwd'].max() - rail['fwd'].min() > 15 else 1
    deg = min(deg, len(set(rail['fwd'])) - 1)
    return np.polyval(np.polyfit(rail['fwd'], rail['lat'], deg), f) if deg > 0 else np.full_like(f, np.median(rail['lat']))
