"""Физические признаки объекта в зоне по окружающему облаку: похож ли он на предмет на пути
или на часть тоннеля, которую коридор задел из-за ошибки оси или высоты полотна.

Каждый признак -- утверждение о мире, а не подобранный по записи порог:

    rings        сколько вертикальных лучей лидара попало на объект. Предмет высотой h на
                 дальности d занимает h / (d * 0.125 град) лучей; полоса пола или рельса,
                 поднявшаяся в зону из-за ошибки высоты полотна, -- один луч.
    behind_n     точек сразу ЗА объектом (по дальности), в его полосе вбок и на его высоте.
                 За настоящим предметом -- тень или пол ниже его основания; поверхность
                 (пол на подъёме, рельс) продолжается за "объектом" на той же высоте.
    shell_n      объект высокий, и прямо над ним без разрыва начинается свод или стена:
                 колонна, мачта, стена в кривой (ось ушла), а не предмет на пути.
    wall_left_n, wall_right_n
                 точек тоннеля по бокам объекта (lat больше / меньше его краёв). Посторонний предмет не может быть
                 самой внешней конструкцией на своей стороне: за ним всегда есть стена.

Признаки считаются в detector.check_frame для объектов с треком; решения по ним -- там же
(plausibility_check), здесь только измерение.
"""
import numpy as np

BEAM_RAD = np.radians(0.125)
RING_GAP_RAD = np.radians(0.06)
LIDAR_H = 1.5
CTX_HALF_WIDTH = 4.0


def count_rings(elev):
    """Число различных лучей по углам места точек (рад)."""
    if len(elev) == 0:
        return 0
    e = np.sort(elev)
    return int(1 + np.count_nonzero(np.diff(e) > RING_GAP_RAD))


def _stable_argsort(a):
    """np.argsort(a, kind='stable') для float32 через один np.sort int64-ключей:
    старшие 32 бита -- монотонный образ float32 (-0.0 приравнен к +0.0, как при сравнении),
    младшие -- исходный индекс, поэтому равные значения остаются в порядке входа.
    Иные dtype и NaN -- обычный стабильный argsort."""
    if a.dtype != np.float32 or len(a) >= 1 << 31 or np.isnan(a).any():
        return np.argsort(a, kind='stable')
    b = (a + np.float32(0.0)).view(np.int32).astype(np.int64)
    key = np.where(b < 0, ~b & 0xFFFFFFFF, b | 0x80000000).astype(np.uint64)
    key = (key << np.uint64(32)) | np.arange(len(a), dtype=np.uint64)
    key.sort()
    return (key & np.uint64(0xFFFFFFFF)).astype(np.intp)


class Context:
    """Точки кадра в координатах пути (fwd, lat, h над головкой рельса), отсортированные по fwd:
    выборка окна по дальности -- два searchsorted."""

    def __init__(self, fwd, lat, h):
        o = _stable_argsort(fwd)
        self.f, self.l, self.h = fwd[o], lat[o], h[o]

    def window(self, lo, hi):
        a, b = np.searchsorted(self.f, [lo, hi])
        return self.f[a:b], self.l[a:b], self.h[a:b]


def floor_step(d):
    """На сколько метров по дальности отстоят попадания соседних лучей в пол на дальности d."""
    return d * d * BEAM_RAD / LIDAR_H


def is_implausible(o, min_dist=60.0, behind_thr=2):
    """Подтверждённый объект в дальней зоне, за которым НА ЕГО ВЫСОТЕ идут точки, -- не предмет.
    Сплошное препятствие непрозрачно: сразу за ним по лучу либо тень (точки ниже основания),
    либо ничего. behind_n точек на высоте объекта позади него бывает только у поверхности,
    задевшей коридор из-за ошибки оси/высоты полотна вдали (пол на подъёме, рельс, стена в кривой)."""
    return o['distance_m'] > min_dist and o.get('behind_n', 0) >= behind_thr


def object_features(o, elev, ctx):
    """Признаки объекта o (distance_m, far_m, lat_min_m, lat_max_m, low_m, height_m); elev --
    углы места его точек. Возвращает dict, значения -- числа."""
    d, far = o['distance_m'], o['far_m']
    lo_l, hi_l = o['lat_min_m'], o['lat_max_m']
    low, top = o['low_m'], o['height_m']
    out = {'rings': count_rings(elev),
           'ext_beams': round((top - low) / (max(d, 1.0) * BEAM_RAD), 2)}

    L = float(np.clip(1.5 * floor_step(d), 2.0, 30.0))
    f, l, h = ctx.window(far + 0.3, far + L)
    band = (l > lo_l - 0.3) & (l < hi_l + 0.3)
    out['behind_n'] = int(np.count_nonzero(band & (h >= low - 0.05) & (h <= top + 0.3)))
    out['behind_low_n'] = int(np.count_nonzero(band & (h < low - 0.05) & (h > low - 0.6)))

    f, l, h = ctx.window(d - L, d - 0.3)
    band = (l > lo_l - 0.3) & (l < hi_l + 0.3) & (h < low + 0.5)
    out['front_maxh'] = round(float(h[band].max()), 3) if band.any() else None

    f, l, h = ctx.window(d - 0.3, far + 0.3)
    mid = 0.5 * (low + top)
    side = ((l < lo_l - 0.15) | (l > hi_l + 0.15)) & (np.abs(l) < 2.5) & (np.abs(h - mid) < 0.08 + 0.5 * (top - low))
    out['side_n'] = int(np.count_nonzero(side))

    ds = 1.0 + 0.01 * d
    f, l, h = ctx.window(d - ds, far + ds)
    m = (h > top) & (h < top + 1.2) & (l > lo_l - 0.4) & (l < hi_l + 0.4)
    touch = max(0.25, 3.0 * BEAM_RAD * d)
    out['shell_n'] = int(np.count_nonzero(m)) if m.any() and float(h[m].min()) - top <= touch else 0

    ds = 2.0 + 0.03 * d
    f, l, h = ctx.window(d - ds, far + ds)
    hm = (h > low - 0.6) & (h < top + 0.6)
    out['wall_left_n'] = int(np.count_nonzero(hm & (l > hi_l + 0.3)))
    out['wall_right_n'] = int(np.count_nonzero(hm & (l < lo_l - 0.3)))
    return out
