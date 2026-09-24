"""Профиль полотна по дальности и высота головки рельса.

Полотно вдали НЕ на той же высоте, что вблизи: уклон пути + наклон лидара.
В doubleT_obstacle (pitch 0.8°) дно лотка на 80м на 1.3м ниже, чем на 5м --
при плоском "полу" объект на 56м оказывается под коридором, а подъём
полотна вдали даёт ложные тревоги. Поэтому высота дна лотка между рельсами
оценивается в каждом кадре как функция дальности, а головка рельса = дно +
смещение (0.27-0.56м в зависимости от типа пути, оценивается по найденным рельсам).
"""
import numpy as np

BED_BIN_M = 4.0
BED_CORE_HALF = 0.5
BED_QUANTILE = 10
BED_MIN_PTS = 5
MAX_GRADE = 0.06
BED_TOL = 0.15


def estimate_floor_z(x, y, z, near=2.0, far=15.0, half_width=3.0):
    """Грубая оценка высоты пола одной константой -- запасной вариант, когда
    профиль полотна по дальности (estimate_bed_profile) построить не удалось."""
    fwd = -y
    m = (fwd > near) & (fwd < far) & (np.abs(x) < half_width)
    if m.sum() < 50:
        return float(np.percentile(z, 10))
    return float(np.percentile(z[m], 10))


def _robust_line_pred(f, z, at):
    """Продолжение профиля в точку at: прямая Тейла-Сена (медиана попарных
    наклонов) -- один принятый бин-выброс (предмет на пути) её не сдвигает."""
    f, z = np.asarray(f), np.asarray(z)
    if len(f) < 2:
        return z[-1]
    i, j = np.triu_indices(len(f), 1)
    slope = np.clip(np.median((z[j] - z[i]) / (f[j] - f[i])), -MAX_GRADE, MAX_GRADE)
    return np.median(z - slope * f) + slope * at


def estimate_bed_profile(x, y, z, center_fn, near=2.0, far=250.0):
    """Высота дна лотка между рельсами по дальности: (fwd_bins, z_bed) или None.

    Бины принимаются последовательно от ближних к дальним, только если
    продолжают уже принятый профиль с правдоподобным уклоном. Это защищает от
    того, чтобы крупное препятствие (стоящий состав), закрывающее полотно,
    было принято за "поднявшийся пол" и тем самым само себя спрятало."""
    fwd = -y
    sel = (fwd > near) & (fwd < far)
    fwd, x, z = fwd[sel], x[sel], z[sel]
    sel = np.abs(x - center_fn(fwd)) < BED_CORE_HALF
    fwd, z = fwd[sel], z[sel]
    if len(fwd) < BED_MIN_PTS:
        return None
    n_bins = int(np.ceil((far - near) / BED_BIN_M))
    b = ((fwd - near) / BED_BIN_M).astype(np.int64)
    order = np.lexsort((z, b))
    b_s, z_s = b[order], z[order]
    cnt = np.bincount(b_s, minlength=n_bins)
    start = np.concatenate([[0], np.cumsum(cnt)[:-1]])
    have = np.where(cnt >= BED_MIN_PTS)[0]
    vals = z_s[start[have] + (cnt[have] - 1) * BED_QUANTILE // 100]
    centers = near + (have + 0.5) * BED_BIN_M

    acc_f, acc_z = [], []
    for f, v in zip(centers, vals):
        if not acc_f:
            if f < 15.0:
                acc_f.append(f); acc_z.append(v)
            continue
        pred = _robust_line_pred(acc_f[-6:], acc_z[-6:], f)
        if abs(v - pred) < BED_TOL + 0.01 * (f - acc_f[-1]):
            acc_f.append(f); acc_z.append(v)
    if not acc_f:
        return None
    return np.array(acc_f), np.array(acc_z)


def bed_at(fwd, profile, tail_m=20.0):
    """Высота дна лотка на дальностях fwd: интерполяция внутри профиля,
    за его концом -- линейное продолжение по последним tail_m метрам."""
    pf, pz = profile
    out = np.interp(fwd, pf, pz)
    beyond = fwd > pf[-1]
    if beyond.any():
        t = pf >= pf[-1] - tail_m
        slope = np.clip(np.polyfit(pf[t], pz[t], 1)[0], -MAX_GRADE, MAX_GRADE) if t.sum() >= 2 else 0.0
        out[beyond] = pz[-1] + slope * (fwd[beyond] - pf[-1])
    return out


def rail_top_at(fwd, bed, rail_offset, rail_prof=None):
    """Высота головки рельса по дальности: по самим рельсам (rail_prof), где они
    найдены, дальше -- дно лотка + rail_offset."""
    tor = bed_at(fwd, bed) + rail_offset
    if rail_prof is not None:
        rf, rz = rail_prof
        inside = fwd <= rf[-1]
        tor[inside] = np.interp(fwd[inside], rf, rz)
    return tor
