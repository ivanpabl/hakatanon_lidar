"""Точки в зоне -> объекты, и минимальный размер объекта по дальности."""
import numpy as np

# Настоящий объект даёт точек примерно ~1/d^2 (угловое разрешение лидара).
# По вставкам (tools/eval_injection.py): куб 0.4м на головке рельса даёт ~40000/d^2
# точек, из них выше clearance (0.15м) ~60% -> ~24000/d^2. Порог -- половина
# этого, но не меньше MIN_POINTS_FLOOR: 30 точек на 20м, 7.5 на 40м, 3 с 63м.
# Без порога 60% кадров с тревогой -- объекты из 1-2 точек (tools/alarms.py).
MIN_POINTS_K = 12000.0
MIN_POINTS_FLOOR = 3


def min_points_at(distance_m, k=MIN_POINTS_K, floor=MIN_POINTS_FLOOR):
    return max(floor, k / max(distance_m, 1.0) ** 2)


def cluster_points(fwd, lat, z_rel, fwd_gap=1.0, lat_gap=0.6):
    """Точки в зоне -> объекты: разрыв >fwd_gap по дальности или >lat_gap вбок
    разделяет объекты."""
    if len(fwd) == 0:
        return []
    o = np.argsort(fwd)
    fwd, lat, z_rel = fwd[o], lat[o], z_rel[o]
    objects = []
    for g in np.split(np.arange(len(fwd)), np.where(np.diff(fwd) > fwd_gap)[0] + 1):
        go = g[np.argsort(lat[g])]
        for h in np.split(go, np.where(np.diff(lat[go]) > lat_gap)[0] + 1):
            objects.append({
                'distance_m': float(fwd[h].min()),
                'lateral_m': float(np.median(lat[h])),
                'height_m': float(z_rel[h].max()),  # верх объекта над головкой рельса
                'n_points': int(len(h)),
            })
    return objects
