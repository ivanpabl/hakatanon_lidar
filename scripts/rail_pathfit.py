"""Поиск пары рельсов и центральной линии пути по одному кадру лидара.

Библиотечная часть (find_floor_bumps, cluster_lines, pick_rail_pair, fit_path)
зависит только от numpy -- её импортирует detector_interface и ROS2-нода.
rosbags/matplotlib нужны только для отладочного запуска этого файла как скрипта.
"""
import warnings
from pathlib import Path

import numpy as np

NEAR, FAR = 2.0, 40.0
HALF_WIDTH = 3.5
LAT_BIN = 0.05
FWD_BIN = 1.0
PROMINENCE = 0.05
GAUGE_TARGET = 1.52
GAUGE_TOL = 0.40  # accept 1.12 - 1.92 m
CLUSTER_GAP = 0.25  # meters, gap to split clusters when sorting lateral positions

_LAT_EDGES = np.arange(-HALF_WIDTH, HALF_WIDTH + LAT_BIN, LAT_BIN)
_N_LAT = len(_LAT_EDGES) + 1  # np.digitize даёт индексы 0..len(edges)
_LAT_CENTERS = np.concatenate([[_LAT_EDGES[0] - LAT_BIN], _LAT_EDGES])
_BASELINE_W = 10  # полуширина окна медианы-подложки, в лат. бинах
_PEAK_W = 3       # полуширина окна проверки локального максимума


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

    # точки ниже "пол + 0.9м" своего среза (30-й перцентиль по срезу), срезы с >=50 точками
    order = np.lexsort((zf, fbin))
    fb_s, z_s = fbin[order], zf[order]
    counts = np.bincount(fb_s, minlength=n_f)
    starts = np.concatenate([[0], np.cumsum(counts)[:-1]])
    zlo = np.full(n_f, np.nan)
    for b in np.where(counts >= 50)[0]:
        zlo[b] = np.percentile(z_s[starts[b]:starts[b] + counts[b]], 30)
    keep = np.isfinite(zlo[fbin]) & (zf < zlo[fbin] + 0.9)
    xf, zf, fbin = xf[keep], zf[keep], fbin[keep]
    row_ok = np.bincount(fbin, minlength=n_f) >= 30

    # поперечный профиль: максимум z в каждом (срез, лат. бин)
    lbin = np.digitize(xf, _LAT_EDGES)
    profile = np.full(n_f * _N_LAT, -np.inf)
    np.maximum.at(profile, fbin * _N_LAT + lbin, zf)
    profile = profile.reshape(n_f, _N_LAT)
    profile[np.isneginf(profile)] = np.nan
    valid = ~np.isnan(profile)
    row_ok &= valid.sum(axis=1) >= 15
    profile, valid = profile[row_ok], valid[row_ok]
    f_rows = f_starts[row_ok]
    if len(f_rows) == 0:
        return empty

    with warnings.catch_warnings():  # окна целиком из nan -- ожидаемо, дают nan
        warnings.simplefilter('ignore', RuntimeWarning)
        baseline = np.nanmedian(_sliding(profile, _BASELINE_W, np.nan), axis=-1)
        local_max = np.nanmax(_sliding(profile, _PEAK_W, np.nan), axis=-1)
    prominence = profile - baseline
    with np.errstate(invalid='ignore'):
        is_peak = valid & (prominence > PROMINENCE) & (profile >= local_max)

    ri, li = np.nonzero(is_peak)
    out = (f_rows[ri] + FWD_BIN / 2, _LAT_CENTERS[li], prominence[ri, li])
    return out + (profile[ri, li],) if return_z else out

def cluster_lines(pf, pl, pz=None):
    order = np.argsort(pl)
    pl_s, pf_s = pl[order], pf[order]
    pz_s = None if pz is None else pz[order]
    clusters = []
    cur_idx = [0]
    for i in range(1, len(pl_s)):
        if pl_s[i] - pl_s[i - 1] > CLUSTER_GAP:
            clusters.append(cur_idx)
            cur_idx = []
        cur_idx.append(i)
    clusters.append(cur_idx)
    result = []
    for idxs in clusters:
        if len(idxs) < 5:
            continue
        c = {'x_center': np.median(pl_s[idxs]), 'fwd': pf_s[idxs], 'lat': pl_s[idxs]}
        if pz_s is not None:
            c['z'] = pz_s[idxs]  # высота головки рельса в каждом пике
        result.append(c)
    result.sort(key=lambda c: c['x_center'])
    return result

def pick_rail_pair(clusters):
    # Проверяем ВСЕ пары кластеров, а не только соседние по сортировке --
    # соседняя пара может быть разделена слабым ложным кластером-шумом,
    # из-за чего настоящая пара рельсов никогда не сравнивается напрямую.
    best = None
    for i in range(len(clusters)):
        for j in range(i + 1, len(clusters)):
            d = clusters[j]['x_center'] - clusters[i]['x_center']
            if abs(d - GAUGE_TARGET) < GAUGE_TOL:
                mid = (clusters[j]['x_center'] + clusters[i]['x_center']) / 2
                score = abs(mid)  # prefer pair straddling sensor x=0 (own track)
                if best is None or score < best[0]:
                    best = (score, clusters[i], clusters[j], d)
    return best

# --- прослеживание пары рельсов своего пути ------------------------------
# cluster_lines склеивает пики только по боковой координате на всём участке
# 2-40м: рельс на кривой или рядом с мусором сцепляется с соседними линиями,
# центр кластера уезжает, и в записях со станциями/стрелками выбиралась чужая
# пара (ось пути на 1.5-2.4м в стороне -> коридор на стене/платформе).
# Здесь пара ищется там, где она однозначна: вблизи и по бокам от лидара
# (поезд стоит на своём пути), а дальше каждый рельс ведётся по одному
# срезу, принимая только пик рядом с предсказанием.

SEED_FWD = 10.0              # затравка пары -- только по пикам ближе этого
SEED_GAUGE = (1.45, 1.75)    # колея по пикам профиля (1.52 номинал + ширина головки/бины)
SEED_MAX_MID = 0.4           # середина пары не дальше этого от лидара
SEED_WIN = 0.1               # поддержка рельса: пики в ±SEED_WIN от его положения
SEED_MIN_SUPPORT = 3
TRACE_TOL = 0.12             # допуск пика от предсказания при прослеживании
TRACE_MAX_GAP = 8.0          # столько метров без пика -- рельс потерян


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
    """Пара рельсов своего пути: (score, left, right, gauge) как у pick_rail_pair, или None."""
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
    # лучшая пара -- с наибольшей поддержкой; каждые 10см смещения от лидара -- минус 1 пик
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


# --- продление оси пути дальше рельсов по стенам тоннеля -----------------
# Рельсы видны до 25-40м, а в кривых именно дальше этого коридор "по прямой"
# заходит на стену -- главный источник ложных тревог. Стены (или край
# платформы) видны до 115-160м и идут параллельно пути: где рельсы известны,
# меряем расстояние от оси до ближайшей поверхности слева/справа, а дальше
# ведём ось шагами по WALL_BIN_M так, чтобы это расстояние сохранялось.
# Точки ближе WALL_MIN_LAT к оси (сам коридор) в поиск стены не входят --
# препятствие на пути не может "сдвинуть стену".

WALL_BIN_M = 4.0
WALL_BAND = (0.5, 2.5)     # высота над головкой рельса, в которой ищем стену
WALL_MIN_LAT = 1.1         # ближе к оси -- это уже коридор, не стена
WALL_MAX_LAT = 4.5         # дальше -- стены нет (двухпутный тоннель, пустота)
WALL_TOL = 0.3             # допуск стены от предсказания, м
WALL_MAX_MISSES = 3        # столько бинов подряд без стены -- ось дальше не известна


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

    # расстояние до стен там, где ось известна по рельсам
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
        if len(ds) >= 3 and np.std(ds) < 0.3:  # стена ровная на известном участке
            offsets[side] = float(np.median(ds))
    if not offsets:
        return fit_fwd, cl

    # ведём ось дальше
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
    # сгладить продление скользящим средним по 3 бинам
    c_t = ext_c[tail]
    if len(c_t) >= 3:
        c_t = np.convolve(np.r_[c_t[0], c_t, c_t[-1]], np.ones(3) / 3, mode='valid')
    return np.concatenate([fit_fwd, ext_f[tail]]), np.concatenate([cl, c_t])


def _load_xyz(msg):
    buf = np.frombuffer(msg.data, dtype=np.uint8).reshape(-1, msg.point_step)
    x = buf[:, 0:4].view(np.float32).ravel()
    y = buf[:, 4:8].view(np.float32).ravel()
    z = buf[:, 8:12].view(np.float32).ravel()
    valid = np.isfinite(x) & np.isfinite(y) & np.isfinite(z)
    return x[valid], y[valid], z[valid]


def get_frame(bag, idx):
    from rosbags.highlevel import AnyReader
    from rosbags.typesys import Stores, get_typestore
    ts = get_typestore(Stores.ROS2_HUMBLE)
    with AnyReader([bag], default_typestore=ts) as reader:
        conn = reader.connections[0]
        for i, (connection, timestamp, rawdata) in enumerate(reader.messages(connections=[conn])):
            if i == idx:
                msg = reader.deserialize(rawdata, connection.msgtype)
                return _load_xyz(msg)
    return None


if __name__ == '__main__':
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    ROOT = Path(__file__).resolve().parent.parent / 'Датасет' / 'archive' / 'for_hackathon'
    OUT = Path(__file__).parent / 'out'
    OUT.mkdir(exist_ok=True)
    BAGS = sorted([p for p in ROOT.iterdir() if p.is_dir()])
    FRAME_IDX = 10

    fig, axes = plt.subplots(2, 3, figsize=(18, 11))
    report = []
    for ax, bag in zip(axes.ravel(), BAGS):
        res = get_frame(bag, FRAME_IDX)
        x, y, z = res
        fwd_all = -y
        m = (fwd_all > 0) & (fwd_all < FAR) & (np.abs(x) < HALF_WIDTH + 0.5)
        ax.scatter(x[m], fwd_all[m], s=0.2, c='lightgray', alpha=0.5)

        pf, pl, ph = find_floor_bumps(x, y, z)
        clusters = cluster_lines(pf, pl)
        for c in clusters:
            ax.scatter(c['lat'], c['fwd'], s=8, c='silver')

        pair, fit_fwd, cl = fit_path(x, y, z)
        if pair is None:
            report.append((bag.name, None))
            ax.set_title(f'{bag.name}: пара рельсов НЕ найдена')
        else:
            _, left, right, gauge = pair
            ax.scatter(left['lat'], left['fwd'], s=20, c='red', label='rail A')
            ax.scatter(right['lat'], right['fwd'], s=20, c='blue', label='rail B')
            ax.plot(cl, fit_fwd, 'k--', lw=2, label='центральная линия пути')
            drift = cl[-1] - cl[0]
            report.append((bag.name, gauge, drift, fit_fwd[-1] - fit_fwd[0]))
            ax.set_title(f'{bag.name}: колея={gauge:.2f}м, снос={drift:+.2f}м/{fit_fwd[-1]-fit_fwd[0]:.0f}м')
            ax.legend(fontsize=7)
        ax.set_xlabel('x, м'); ax.set_ylabel('вперёд (-y), м')
        ax.set_xlim(-HALF_WIDTH - 0.5, HALF_WIDTH + 0.5)
        ax.invert_xaxis()

    fig.tight_layout()
    fig.savefig(OUT / 'rail_path_final.png', dpi=130)
    print('saved', OUT / 'rail_path_final.png')
    print()
    for r in report:
        print(r)
