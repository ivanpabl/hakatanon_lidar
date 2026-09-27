"""Точки в зоне -> объекты, и минимальный размер объекта по дальности."""
import numpy as np

MIN_POINTS_K = 12000.0
MIN_POINTS_FLOOR = 3


def min_points_at(distance_m, k=MIN_POINTS_K, floor=MIN_POINTS_FLOOR):
    return max(floor, k / max(distance_m, 1.0) ** 2)


def cluster_points(fwd, lat, z_rel, fwd_gap=1.0, lat_gap=0.6, keep_idx=False):
    """Точки в зоне -> объекты: разрыв >fwd_gap по дальности или >lat_gap вбок
    разделяет объекты. keep_idx -- у объекта '_idx': номера его точек во входных массивах."""
    if len(fwd) == 0:
        return []
    o = np.argsort(fwd)
    fwd, lat, z_rel = fwd[o], lat[o], z_rel[o]
    objects = []
    for g in np.split(np.arange(len(fwd)), np.where(np.diff(fwd) > fwd_gap)[0] + 1):
        go = g[np.argsort(lat[g])]
        for h in np.split(go, np.where(np.diff(lat[go]) > lat_gap)[0] + 1):
            ob = {
                'distance_m': float(fwd[h].min()),
                'far_m': float(fwd[h].max()),
                'lateral_m': float(np.median(lat[h])),
                'lat_min_m': float(lat[h].min()),
                'lat_max_m': float(lat[h].max()),
                'height_m': float(z_rel[h].max()),
                'low_m': float(z_rel[h].min()),
                'n_points': int(len(h)),
            }
            if keep_idx:
                ob['_idx'] = o[h]
            objects.append(ob)
    return objects


EDGE_MIN_LAT = 0.45
EDGE_LAT_TOL = 0.2
EDGE_MIN_SPAN = 8.0
EDGE_FAR_MAX_LEN = 5.0


def mark_edge_lines(objects, min_abs_lat=EDGE_MIN_LAT, lat_tol=EDGE_LAT_TOL, min_span=EDGE_MIN_SPAN,
                    far_start=np.inf, far_max_len=EDGE_FAR_MAX_LEN):
    """Помечает o['edge_line'] у объектов, которые вместе с соседями образуют продольную
    линию у края коридора: контактный рельс, кабель, край платформы, стена в кривой.
    Вдали такая линия распадается на много мелких кластеров на одном смещении вбок,
    и каждый из них похож на отдельный предмет. Настоящее препятствие компактно вдоль
    пути; линия длиннее min_span на почти постоянном смещении -- конструкция.

    Дальше far_start (за концом найденных рельсов ось -- оценка) в цепочки идут объекты
    с любым смещением: при ошибке оси стена пересекает коридор наискосок. Там же
    одиночный объект длиннее far_max_len вдоль пути считается конструкцией."""
    cand = sorted((o for o in objects if abs(o['lateral_m']) >= min_abs_lat or o['distance_m'] > far_start),
                  key=lambda o: o['distance_m'])
    chains = []
    for o in cand:
        gap = max(3.0, 0.05 * o['distance_m'])
        best = None
        for ch in chains:
            last = ch[-1]
            same_side = np.sign(o['lateral_m']) == np.sign(last['lateral_m']) or o['distance_m'] > far_start
            if (o['distance_m'] - last['far_m'] <= gap and same_side
                    and abs(o['lateral_m'] - last['lateral_m']) < lat_tol):
                if best is None or last['far_m'] > best[-1]['far_m']:
                    best = ch
        if best is None:
            chains.append([o])
        else:
            best.append(o)
    for ch in chains:
        span = max(o['far_m'] for o in ch) - ch[0]['distance_m']
        for o in ch:
            o['edge_line'] = span >= min_span or (o['distance_m'] > far_start and o['far_m'] - o['distance_m'] > far_max_len)
    for o in objects:
        o.setdefault('edge_line', False)
    return objects
