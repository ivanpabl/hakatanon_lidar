"""Вставка синтетических препятствий в реальные кадры лидара.

Объект не "дорисовывается" точками, а снимается нашим же лидаром: для каждого
луча кадра считается пересечение с объектом, и если объект ближе того, что луч
увидел на самом деле, точка заменяется точкой на объекте. Отсюда автоматически:
- плотность точек как у настоящего датчика на этой дальности (на 150м объект
  задевают 1-3 канала);
- тень -- за объектом пропадает пол/стена;
- лучи, которые в исходном кадре ничего не вернули (38-62% облака, в т.ч.
  "в глубину" тоннеля), объект тоже перехватывает.

Геометрия лучей восстанавливается по самому кадру (проверено на всех 7 записях):
облако -- столбцы по 128 каналов, пара столбцов (2k, 2k+1) -- dual return одного
залпа, азимут пары = A0 - 0.1 град * k, у каждого канала постоянные угол места
и сдвиг азимута (до +-7.8 град). Ошибка восстановления направления < 0.001 град.

Система координат та же, что в detector_interface: x -- вбок, вперёд = -y, z -- вверх.
Объекты задаются в (lat, fwd, z): lat = x, fwd = -y.

Пример (8 кадров подряд, ящик 0.5x0.5x1.0м на оси пути в 60м):
    python scripts/inject.py --bag roundT_doubleT --start 100 --shape box --dims 0.5,0.5,1.0 --fwd 60
"""
from dataclasses import dataclass

import numpy as np

COLUMN_HEIGHT = 128
AZ_STEP_DEG = 0.1  # шаг азимута между парами столбцов


# --- фигуры -----------------------------------------------------------------
# У всех фигур z0 -- низ объекта (обычно головка рельса), lat/fwd -- центр основания.
# intersect(d) -> расстояние вдоль луча до первого попадания, inf -- промах.

@dataclass
class Box:
    fwd: float
    lat: float
    z0: float
    length: float   # вдоль пути
    width: float    # поперёк пути
    height: float
    yaw_deg: float = 0.0  # поворот вокруг вертикали (90 -- балка поперёк пути при length > width)

    def bound(self):
        c = np.array([self.lat, self.fwd, self.z0 + self.height / 2])
        return c, 0.5 * np.sqrt(self.length ** 2 + self.width ** 2 + self.height ** 2)

    def intersect(self, d):
        # переводим лучи в систему ящика и режем слэбами
        a = np.radians(self.yaw_deg)
        ca, sa = np.cos(a), np.sin(a)
        o = -np.array([self.lat, self.fwd, self.z0 + self.height / 2])
        o = np.array([ca * o[0] + sa * o[1], -sa * o[0] + ca * o[1], o[2]])
        dl = ca * d[:, 0] + sa * d[:, 1]
        df = -sa * d[:, 0] + ca * d[:, 1]
        half = np.array([self.width, self.length, self.height]) / 2
        tmin = np.full(len(d), -np.inf)
        tmax = np.full(len(d), np.inf)
        for k, dk in enumerate((dl, df, d[:, 2])):
            with np.errstate(divide='ignore', invalid='ignore'):
                t1 = (-half[k] - o[k]) / dk
                t2 = (half[k] - o[k]) / dk
            par = np.abs(dk) < 1e-12  # луч параллелен граням: попадает, только если внутри слэба
            inside = np.abs(o[k]) <= half[k]
            t1 = np.where(par, np.where(inside, -np.inf, np.inf), t1)
            t2 = np.where(par, np.where(inside, np.inf, -np.inf), t2)
            tmin = np.maximum(tmin, np.minimum(t1, t2))
            tmax = np.minimum(tmax, np.maximum(t1, t2))
        return np.where((tmax >= tmin) & (tmin > 1e-3), tmin, np.inf)


@dataclass
class Cylinder:
    """Вертикальный цилиндр (столб, бочка, человек -- r~0.25, h~1.7)."""
    fwd: float
    lat: float
    z0: float
    radius: float
    height: float

    def bound(self):
        c = np.array([self.lat, self.fwd, self.z0 + self.height / 2])
        return c, np.sqrt(self.radius ** 2 + (self.height / 2) ** 2)

    def intersect(self, d):
        ol, of = -self.lat, -self.fwd
        a = d[:, 0] ** 2 + d[:, 1] ** 2
        b = 2 * (ol * d[:, 0] + of * d[:, 1])
        c = ol ** 2 + of ** 2 - self.radius ** 2
        disc = b ** 2 - 4 * a * c
        with np.errstate(divide='ignore', invalid='ignore'):
            t_side = (-b - np.sqrt(np.maximum(disc, 0))) / (2 * a)
        zs = d[:, 2] * t_side
        side_ok = (disc >= 0) & (a > 1e-12) & (t_side > 1e-3) & (zs >= self.z0) & (zs <= self.z0 + self.height)
        t = np.where(side_ok, t_side, np.inf)
        # крышка (сверху лидар видит её у низких широких объектов)
        with np.errstate(divide='ignore', invalid='ignore'):
            t_cap = (self.z0 + self.height) / d[:, 2]
        pl, pf = d[:, 0] * t_cap, d[:, 1] * t_cap
        cap_ok = (d[:, 2] < 0) & (t_cap > 1e-3) & ((pl - self.lat) ** 2 + (pf - self.fwd) ** 2 <= self.radius ** 2)
        return np.minimum(t, np.where(cap_ok, t_cap, np.inf))


@dataclass
class Sphere:
    fwd: float
    lat: float
    z0: float
    radius: float

    def bound(self):
        return np.array([self.lat, self.fwd, self.z0 + self.radius]), self.radius

    def intersect(self, d):
        o = -self.bound()[0]
        b = 2 * (d @ o)
        c = o @ o - self.radius ** 2
        disc = b ** 2 - 4 * c
        t = (-b - np.sqrt(np.maximum(disc, 0))) / 2
        return np.where((disc >= 0) & (t > 1e-3), t, np.inf)


def make_shape(kind, dims, fwd, lat, z0, yaw_deg=0.0):
    """kind: box (length,width,height) | cylinder (radius,height) | sphere (radius)."""
    if kind == 'box':
        return Box(fwd, lat, z0, *dims, yaw_deg=yaw_deg)
    if kind == 'cylinder':
        return Cylinder(fwd, lat, z0, *dims)
    if kind == 'sphere':
        return Sphere(fwd, lat, z0, *dims)
    raise ValueError(kind)


# --- геометрия лучей ----------------------------------------------------------

def _xyz_views(buf, fields):
    offs = {'x': 0, 'y': 4, 'z': 8, 'intensity': 12}
    if fields is not None:
        offs.update({f.name: f.offset for f in fields if f.name in offs})
    return {k: buf[:, o:o + 4].view(np.float32)[:, 0] for k, o in offs.items()}


def beam_directions(x, y, z):
    """Единичные направления всех лучей упорядоченного облака, (N, 3) в (lat, fwd, z).
    Строятся и для лучей без отражения (нулевых точек)."""
    n = len(x)
    if n % (2 * COLUMN_HEIGHT):
        raise ValueError(f'облако не из столбцов по {COLUMN_HEIGHT} каналов парами: {n} точек')
    ncol = n // COLUMN_HEIGHT
    X, Y, Z = (np.asarray(a, np.float64).reshape(ncol, COLUMN_HEIGHT) for a in (x, y, z))
    valid = (X != 0) | (Y != 0) | (Z != 0)
    if valid.sum(axis=0).min() == 0:
        raise ValueError('в кадре есть каналы без единого отражения -- углы канала не восстановить')
    R = np.sqrt(X ** 2 + Y ** 2 + Z ** 2)
    az = np.where(valid, np.degrees(np.arctan2(X, -Y)), np.nan)
    el = np.where(valid, np.degrees(np.arcsin(np.clip(Z / np.maximum(R, 1e-9), -1, 1))), np.nan)

    wrap = lambda a: (a + 180) % 360 - 180
    col_med = np.nanmedian(az, axis=1)
    off = np.nanmedian(wrap(az - col_med[:, None]), axis=0)          # сдвиг азимута канала
    el_ring = np.nanmedian(el, axis=0)                               # угол места канала
    base = np.nanmedian(wrap(az - off[None, :]), axis=1)             # азимут пары столбцов
    pair = np.arange(ncol) // 2
    ok = np.isfinite(base)
    a0 = np.median(base[ok] + AZ_STEP_DEG * pair[ok])  # base = a0 - 0.1 * pair
    az_all = np.radians(wrap(a0 - AZ_STEP_DEG * pair[:, None] + off[None, :]))
    el_all = np.radians(np.broadcast_to(el_ring, (ncol, COLUMN_HEIGHT)))
    d = np.stack([np.cos(el_all) * np.sin(az_all), np.cos(el_all) * np.cos(az_all), np.sin(el_all)], axis=-1)
    return d.reshape(-1, 3)


# --- вставка ------------------------------------------------------------------

def inject(data, point_step, objects, fields=None, dirs=None, range_noise=0.02, dropout=0.1,
           max_range=250.0, intensity=30.0, rng=None):
    """Вставляет объекты в сырой буфер PointCloud2. Возвращает (новые байты, info).

    range_noise -- шум дальности, м (СКО); dropout -- доля лучей, попавших в объект,
    но не вернувших отражение (тёмная поверхность): такая точка становится нулевой,
    тень за объектом при этом остаётся. dirs -- направления лучей, если уже посчитаны
    (beam_directions); иначе считаются по этому кадру."""
    rng = np.random.default_rng() if rng is None else rng
    buf = np.frombuffer(data, dtype=np.uint8).reshape(-1, point_step).copy()
    v = _xyz_views(buf, fields)
    x, y, z = v['x'], v['y'], v['z']
    if dirs is None:
        dirs = beam_directions(x, y, z)
    r = np.sqrt(x.astype(np.float64) ** 2 + y ** 2 + z ** 2)
    valid = r > 0

    t = np.full(len(x), np.inf)
    per_obj = []
    for ob in objects:
        # считаем пересечение только для лучей в угловой окрестности объекта
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
    x[lost] = y[lost] = z[lost] = 0.0  # попал в объект, но отражения нет -- как "нет возврата"

    # dual return: второй столбец пары дублирует первый -> делим пополам для счёта
    first = lambda m: int(m.reshape(-1, 2, COLUMN_HEIGHT)[:, 0].sum())
    info = {'rays_on_object': per_obj, 'points_replaced': int(ret.sum()), 'rays_dropped': int(lost.sum()),
            'points_on_object': first(ret),
            # лучи, геометрически попадающие в объект, и те из них, что не заслонены сценой:
            # visible << geometric -- объект стоит за стеной/платформой (неверное размещение)
            'rays_geometric': first(t < max_range), 'rays_visible': first(blocked)}
    return buf.tobytes(), info


def on_track(det, fwd, lat_offset=0.0):
    """Положение на найденной детектором оси пути: (lat, z0 = высота головки рельса).
    det должен уже обработать хотя бы один кадр (check_frame/detect)."""
    f = np.array([float(fwd)])
    return float(det._center_of_fwd(f)[0] + lat_offset), float(det._tor_at(f, det._bed)[0])


# --- демонстрация -------------------------------------------------------------

def main():
    import argparse
    import sys
    import warnings
    from pathlib import Path
    warnings.simplefilter('ignore')
    sys.path.insert(0, str(Path(__file__).parent))
    from rosbags.highlevel import AnyReader
    from rosbags.typesys import Stores, get_typestore
    import detector_interface as di
    import make_labeling_set as mls

    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--bag', required=True)
    ap.add_argument('--start', type=int, default=100, help='первый кадр со вставкой')
    ap.add_argument('--frames', type=int, default=8, help='сколько кадров подряд со вставкой')
    ap.add_argument('--shape', default='box', choices=['box', 'cylinder', 'sphere'])
    ap.add_argument('--dims', default='0.5,0.5,1.0', help='box: длина,ширина,высота; cylinder: радиус,высота; sphere: радиус')
    ap.add_argument('--fwd', type=float, default=60.0, help='дальность до объекта, м (постоянная)')
    ap.add_argument('--lat', type=float, default=0.0, help='смещение от оси пути, м')
    ap.add_argument('--yaw', type=float, default=0.0)
    ap.add_argument('--seed', type=int, default=0)
    args = ap.parse_args()
    dims = [float(s) for s in args.dims.split(',')]

    rng = np.random.default_rng(args.seed)
    ref = di.ObstacleDetector()   # чистый поток: путь для размещения объекта и базовый результат
    test = di.ObstacleDetector()  # поток со вставкой
    warm = 20
    ts = get_typestore(Stores.ROS2_HUMBLE)
    last = None
    print(f'{args.shape} {dims} на {args.fwd:.0f}м, смещение {args.lat:+.2f}м от оси; кадры {args.start}..{args.start + args.frames - 1}')
    print(f'{"кадр":>5} {"точек на объекте":>17} {"без вставки":>14} {"со вставкой":>14}  объект около {args.fwd:.0f}м')
    with AnyReader([mls.bag_path(args.bag)], default_typestore=ts) as reader:
        conn = [c for c in reader.connections if c.msgtype == 'sensor_msgs/msg/PointCloud2'][0]
        for i, (c, t, raw) in enumerate(reader.messages(connections=[conn])):
            if i < args.start - warm:
                continue
            if i >= args.start + args.frames:
                break
            msg = reader.deserialize(raw, c.msgtype)
            x, y, z = di.parse_pointcloud2(msg.data, msg.point_step, msg.fields)
            res_ref = ref.detect(x, y, z, refit_path=True)
            if i < args.start:
                test.detect(x, y, z, refit_path=True)
                continue
            lat, z0 = on_track(ref, args.fwd, args.lat)
            ob = make_shape(args.shape, dims, args.fwd, lat, z0, args.yaw)
            data, info = inject(msg.data, msg.point_step, [ob], msg.fields, rng=rng)
            xi, yi, zi = di.parse_pointcloud2(data, msg.point_step, msg.fields)
            res = test.detect(xi, yi, zi, refit_path=True)
            near = [o for o in res['objects'] if abs(o['distance_m'] - args.fwd) < 3.0]
            fmt = lambda r: f"тревога {r['distance_m']:.1f}м" if r['obstacle'] else 'нет тревоги'
            nd = (f"{len(near)} кл., {sum(o['n_points'] for o in near)} т., "
                  f"{'подтверждён' if any(o['confirmed'] for o in near) else 'не подтверждён'}"
                  + (', за осью пути' if any(o['beyond_path'] for o in near) else '')) if near else 'не виден'
            print(f"{i:>5} {info['points_on_object']:>17} {fmt(res_ref):>14} {fmt(res):>14}  {nd}")
            last = (i, x, y, z, xi, yi, zi, mls._snapshot(ref), mls._snapshot(test))

    i, x, y, z, xi, yi, zi, s_ref, s_test = last
    out = Path(__file__).parent / 'out'
    out.mkdir(exist_ok=True)
    tag = f"{args.bag}_f{i}_{args.shape}_{args.dims.replace(',', 'x')}_{args.fwd:.0f}m"
    mls.render_frame(out / f'inject_{tag}_before.png', x, y, z, s_ref, f'{args.bag} кадр {i}: без вставки')
    mls.render_frame(out / f'inject_{tag}_after.png', xi, yi, zi, s_test,
                     f'{args.bag} кадр {i}: {args.shape} {args.dims} м на {args.fwd:.0f} м, {args.lat:+.1f} м от оси')
    print('saved', out / f'inject_{tag}_before.png', 'и _after.png')


if __name__ == '__main__':
    main()
