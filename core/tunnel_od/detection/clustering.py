"""Точки в зоне -> объекты, и минимальный размер объекта по дальности."""
import numpy as np

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
                'height_m': float(z_rel[h].max()),
                'n_points': int(len(h)),
            })
    return objects
