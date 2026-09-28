"""Вставка синтетических препятствий в реальные кадры лидара.

Объект не "дорисовывается" точками, а снимается нашим же лидаром: для каждого
луча кадра считается пересечение с объектом, и если объект ближе того, что луч
увидел на самом деле, точка заменяется точкой на объекте. Отсюда автоматически:
- плотность точек как у настоящего датчика на этой дальности (на 150м объект
  задевают 1-3 канала);
- тень -- за объектом пропадает пол/стена;
- лучи, которые в исходном кадре ничего не вернули (38-62% облака, в т.ч.
  "в глубину" тоннеля), объект тоже перехватывает.

Направления лучей -- tunnel_od.pointcloud.beam_directions. Фигуры -- sim.shapes.
Пример запуска на записи -- python -m evaluation inject-demo.
"""
import numpy as np

from ..geometry.path import TrackPath
from ..pointcloud import COLUMN_HEIGHT, beam_directions, xyz_views


def inject(data, point_step, objects, fields=None, dirs=None, range_noise=0.02, dropout=0.1,
           max_range=250.0, intensity=30.0, rng=None):
    """Вставляет объекты в сырой буфер PointCloud2. Возвращает (новые байты, info).

    range_noise -- шум дальности, м (СКО); dropout -- доля лучей, попавших в объект,
    но не вернувших отражение (тёмная поверхность): такая точка становится нулевой,
    тень за объектом при этом остаётся. dirs -- направления лучей, если уже посчитаны
    (beam_directions); иначе считаются по этому кадру."""
    rng = np.random.default_rng() if rng is None else rng
    buf = np.frombuffer(data, dtype=np.uint8).reshape(-1, point_step).copy()
    v = xyz_views(buf, fields)
    x, y, z = v['x'], v['y'], v['z']
    if dirs is None:
        dirs = beam_directions(x, y, z)
    r = np.sqrt(x.astype(np.float64) ** 2 + y ** 2 + z ** 2)
    valid = r > 0

    t = np.full(len(x), np.inf)
    per_obj = []
    for ob in objects:
        c, rad = ob.bound()
        dist = np.linalg.norm(c)
        cos_lim = np.cos(np.arcsin(min(1.0, rad / dist)) + np.radians(0.2)) if dist > rad else -1.0
        cand = np.where(dirs @ (c / dist) >= cos_lim)[0]
        tk = ob.intersect(dirs[cand])
        t[cand] = np.minimum(t[cand], tk)
        per_obj.append(int(np.isfinite(tk).sum()))

    blocked = (t < max_range) & (~valid | (t < r))
    ret = blocked & (rng.random(len(x)) >= dropout)
    tn = t[ret] + rng.normal(0, range_noise, ret.sum())
    x[ret] = dirs[ret, 0] * tn
    y[ret] = -dirs[ret, 1] * tn
    z[ret] = dirs[ret, 2] * tn
    if 'intensity' in v:
        v['intensity'][ret] = intensity
    lost = blocked & ~ret
    x[lost] = y[lost] = z[lost] = 0.0

    first = lambda m: int(m.reshape(-1, 2, COLUMN_HEIGHT)[:, 0].sum())
    info = {'rays_on_object': per_obj, 'points_replaced': int(ret.sum()), 'rays_dropped': int(lost.sum()),
            'points_on_object': first(ret),
            'rays_geometric': first(t < max_range), 'rays_visible': first(blocked)}
    return buf.tobytes(), info


def on_track(path: TrackPath, fwd, lat_offset=0.0):
    """Положение на оси пути (ObstacleDetector.track_path()): (lat, z0 = высота головки рельса)."""
    f = np.array([float(fwd)])
    return float(path.center_at(f)[0] + lat_offset), float(path.rail_top_at(f)[0])
