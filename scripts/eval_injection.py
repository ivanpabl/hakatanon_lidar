"""Массовая оценка детектора на синтетических препятствиях (inject.py) в реальных кадрах.

Для каждой записи берутся несколько окон по FRAMES кадров подряд. В каждое окно
по очереди вставляется объект из сетки (фигура x дальность x смещение от оси) на
постоянной дистанции от поезда, и детектор прогоняется по окну. Тот же детектор
на тех же кадрах без вставки -- базовая линия: если он и сам видит что-то в том
же месте, испытание помечается как неоднозначное и в долю найденных не входит.

    python scripts/eval_injection.py                 # все записи, полная сетка (~10 мин на 7 ядрах)
    python scripts/eval_injection.py --quick         # 1 окно на запись, меньше дальностей
    python scripts/eval_injection.py --bags roundT_doubleT doubleT_platform

Пишет scripts/out/eval_injection.csv (строка = испытание) и eval_injection.png,
печатает таблицы: доля найденных по фигуре и дальности, число точек на объекте,
тревоги от объектов вне коридора.
"""
import argparse
import copy
import csv
import sys
import warnings
import zlib
from multiprocessing import Pool
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).parent))
import detector_interface as di  # noqa: E402
import inject as inj  # noqa: E402
import make_labeling_set as mls  # noqa: E402

OUT = Path(__file__).parent / 'out'
BAGS = ['doubleT_obstacle', 'doubleT_platform', 'roundT_doubleT', 'roundT_pressureGate_roundT',
        'roundT_squareT_pressureGate_squareT', 'squareT_platform_squareT_switch', 'new_data']
FRAMES = 8           # кадров со вставкой в испытании
WARMUP = 20          # кадров прогрева детектора перед окном
WINDOW_POS = (0.25, 0.5, 0.75)  # где в записи брать окна (доля длительности)
DISTANCES = (10, 20, 30, 40, 60, 80, 100, 130, 160, 200)

# фигура: (вид, размеры) -- размеры как в inject.make_shape
SHAPES = {
    'куб 0.2':         ('box', (0.2, 0.2, 0.2)),
    'куб 0.4':         ('box', (0.4, 0.4, 0.4)),
    'куб 0.7':         ('box', (0.7, 0.7, 0.7)),
    'куб 1.0':         ('box', (1.0, 1.0, 1.0)),
    'человек стоит':   ('cylinder', (0.25, 1.7)),
    'человек лежит':   ('box', (1.7, 0.5, 0.3)),
    'плоский 0.12':    ('box', (0.6, 0.6, 0.12)),
    'балка поперёк':   ('box', (0.2, 2.0, 0.2)),
}
IN_LAT = 0.0
OUT_LAT = 1.6        # вне коридора (полуширина 1.0м): тревоги быть не должно
OUT_SHAPES = ('куб 0.4', 'куб 1.0', 'человек стоит')


def front_of(kind, dims, fwd):
    """Ближняя к лидару дальность объекта -- её детектор и отдаёт как distance_m."""
    return fwd - (dims[0] / 2 if kind == 'box' else dims[0])


def near_objects(res, front, lat, tol_lat=0.8):
    tol = 1.5 + 0.02 * front
    return [o for o in res['objects']
            if abs(o['distance_m'] - front) < tol and abs(o['lateral_m'] - lat) < tol_lat]


def read_windows(bag, positions, need):
    """Кадры окон: список окон, окно = список (msg_data, point_step, fields) длиной need."""
    from rosbags.highlevel import AnyReader
    from rosbags.typesys import Stores, get_typestore
    ts = get_typestore(Stores.ROS2_HUMBLE)
    windows = []
    with AnyReader([mls.bag_path(bag)], default_typestore=ts) as reader:
        conn = [c for c in reader.connections if c.msgtype == 'sensor_msgs/msg/PointCloud2'][0]
        t0, dur = reader.start_time, reader.duration
        for p in positions:
            start = t0 + int(dur * p)
            frames = []
            for c, t, raw in reader.messages(connections=[conn], start=start):
                m = reader.deserialize(raw, c.msgtype)
                frames.append((bytes(m.data), m.point_step, m.fields, (t - t0) / 1e9))
                if len(frames) == need:
                    break
            if len(frames) == need:
                windows.append(frames)
    return windows


def run_bag(job):
    bag, positions, distances, seed = job
    warnings.simplefilter('ignore')
    rows = []
    windows = read_windows(bag, positions, WARMUP + FRAMES)
    trials = [(name, IN_LAT) for name in SHAPES] + [(name, OUT_LAT) for name in OUT_SHAPES]
    for w, frames in enumerate(windows):
        # чистый проход: прогрев, затем по кадрам окна -- размещение объектов и базовый результат
        ref = di.ObstacleDetector()
        for data, step, fields, _ in frames[:WARMUP]:
            ref.detect(*di.parse_pointcloud2(data, step, fields), refit_path=True)
        start_state = copy.deepcopy(ref)
        prep = []
        for data, step, fields, t in frames[WARMUP:]:
            xyz = di.parse_pointcloud2(data, step, fields)
            res = ref.detect(*xyz, refit_path=True)
            buf = np.frombuffer(data, np.uint8).reshape(-1, step)
            v = inj._xyz_views(buf, fields)
            dirs = inj.beam_directions(v['x'], v['y'], v['z'])
            place = {d: inj.on_track(ref, d) for d in distances}
            prep.append((data, step, fields, dirs, res, place, ref._path_range))
        t_win = frames[WARMUP][3]

        for name, lat in trials:
            kind, dims = SHAPES[name]
            for d in distances:
                rng = np.random.default_rng(zlib.crc32(repr((seed, bag, w, name, lat, d)).encode()))
                front = front_of(kind, dims, d)
                test = copy.deepcopy(start_state)
                det, amb, beyond, alarm_on, pts, errs, in_path = [], 0, 0, 0, [], [], []
                geo = vis = 0
                for data, step, fields, dirs, res_ref, place, prange in prep:
                    lat_axis, z0 = place[d]
                    ob = inj.make_shape(kind, dims, d, lat_axis + lat, z0)
                    new, info = inj.inject(data, step, [ob], fields, dirs=dirs, rng=rng)
                    res = test.detect(*di.parse_pointcloud2(new, step, fields), refit_path=True)
                    pts.append(info['points_on_object'])
                    geo += info['rays_geometric']; vis += info['rays_visible']
                    in_path.append(prange is not None and front <= prange + test.path_margin)
                    near = near_objects(res, front, lat)
                    conf = [o for o in near if o['confirmed']]
                    hit = any(not o['beyond_path'] for o in conf)
                    det.append(hit)
                    beyond += bool(conf) and not hit
                    amb += any(o['confirmed'] for o in near_objects(res_ref, front, lat))
                    if hit:
                        errs.append(min(o['distance_m'] for o in conf) - front)
                    alarm_on += bool(res['obstacle'] and res['distance_m'] is not None
                                     and abs(res['distance_m'] - front) < 1.5 + 0.02 * front)
                first = next((k for k, h in enumerate(det) if h), -1)
                rows.append({
                    'bag': bag, 'window': w, 't_start_s': round(t_win, 1), 'shape': name, 'kind': kind,
                    'dims': 'x'.join(map(str, dims)), 'fwd_m': d, 'lat_m': lat, 'front_m': round(front, 2),
                    'frames': len(det), 'points_on_object': round(float(np.mean(pts)), 1),
                    'detected': int(first >= 0), 'first_frame': first,
                    'frames_detected': int(sum(det)), 'frames_alarm_on_object': alarm_on,
                    'frames_beyond_path': beyond, 'ambiguous': int(amb > 0),
                    'in_known_path': round(float(np.mean(in_path)), 2),
                    # объект заслонён сценой: ось пути здесь -- догадка, и она ушла в стену
                    'occluded': int(geo > 0 and vis < 0.5 * geo),
                    'dist_err_m': round(float(np.median(errs)), 2) if errs else '',
                })
        print(f'  {bag}: окно {w + 1}/{len(windows)} (t={t_win:.0f}с) готово', flush=True)
    return rows


def summarize(rows, distances):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    def table(title, value_fn, subset):
        print(f'\n{title}')
        print(f'{"":<16}' + ''.join(f'{d:>7}м' for d in distances))
        for name in SHAPES:
            cells = []
            for d in distances:
                rs = [r for r in subset if r['shape'] == name and r['fwd_m'] == d]
                cells.append(value_fn(rs) if rs else '   -')
            if any(c != '   -' for c in cells):
                print(f'{name:<16}' + ''.join(f'{c:>8}' for c in cells))

    inside = [r for r in rows if r['lat_m'] == IN_LAT and not r['occluded']]
    clean = [r for r in inside if not r['ambiguous']]
    occl = [r for r in rows if r['lat_m'] == IN_LAT and r['occluded']]
    print(f'\nИсключено испытаний: объект заслонён сценой (поставлен по догадке оси в стену) -- '
          f'{len(occl)}/{len(occl) + len(inside)}; все дальше известной оси: '
          f'{all(r["in_known_path"] < 0.5 for r in occl) if occl else "-"}')
    pct = lambda rs: f'{100 * np.mean([r["detected"] for r in rs]):.0f}%' if rs else '   -'
    table('Доля найденных (объект на оси пути, хотя бы в одном из 8 кадров; без неоднозначных):', pct, clean)
    table('То же, только где объект в пределах известной оси пути:', pct,
          [r for r in clean if r['in_known_path'] >= 0.5])
    table('Точек на объекте (медиана по испытаниям, после вставки, один возврат):',
          lambda rs: f'{np.median([r["points_on_object"] for r in rs]):.0f}', inside)
    table('Причина пропуска "за осью пути" (объект подтверждён, но дальше известной оси + 10м), доля испытаний:',
          lambda rs: f'{100 * np.mean([r["frames_beyond_path"] > 0 and not r["detected"] for r in rs]):.0f}%', clean)

    det = [r for r in clean if r['detected']]
    if det:
        print(f'\nЗадержка до подтверждения: медиана {np.median([r["first_frame"] + 1 for r in det]):.0f} кадр., '
              f'ошибка дистанции: медиана {np.median([abs(r["dist_err_m"]) for r in det]):.2f}м, '
              f'95% {np.percentile([abs(r["dist_err_m"]) for r in det], 95):.2f}м')
    print(f'Неоднозначных испытаний (детектор и без вставки видит объект в том же месте): '
          f'{sum(r["ambiguous"] for r in inside)}/{len(inside)}')
    outside = [r for r in rows if r['lat_m'] == OUT_LAT and not r['ambiguous'] and not r['occluded']]
    if outside:
        fa = [r for r in outside if r['detected']]
        print(f'Объекты вне коридора ({OUT_LAT}м от оси): тревога по объекту в {len(fa)}/{len(outside)} испытаниях'
              + (f' -- {", ".join(sorted({r["shape"] + " " + str(r["fwd_m"]) + "м" for r in fa}))}' if fa else ''))

    fig, axes = plt.subplots(1, 2, figsize=(15, 5.5))
    for name in SHAPES:
        xs, ys, ps = [], [], []
        for d in distances:
            rs = [r for r in clean if r['shape'] == name and r['fwd_m'] == d]
            ra = [r for r in inside if r['shape'] == name and r['fwd_m'] == d]
            if rs:
                xs.append(d); ys.append(100 * np.mean([r['detected'] for r in rs]))
            if ra:
                ps.append((d, np.median([r['points_on_object'] for r in ra])))
        axes[0].plot(xs, ys, 'o-', label=name)
        if ps:
            axes[1].plot(*zip(*ps), 'o-', label=name)
    axes[0].set_xlabel('дальность, м'); axes[0].set_ylabel('найдено, %'); axes[0].set_ylim(-3, 103)
    axes[0].set_title('Доля найденных синтетических объектов на оси пути'); axes[0].grid(alpha=0.3)
    axes[1].set_xlabel('дальность, м'); axes[1].set_ylabel('точек на объекте'); axes[1].set_yscale('symlog')
    axes[1].set_title('Сколько точек даёт объект (медиана)'); axes[1].grid(alpha=0.3)
    axes[0].legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(OUT / 'eval_injection.png', dpi=120)
    print('\nsaved', OUT / 'eval_injection.png')


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--bags', nargs='+', default=BAGS)
    ap.add_argument('--quick', action='store_true', help='1 окно на запись, дальности 20/40/80/130/200')
    ap.add_argument('--workers', type=int, default=None)
    ap.add_argument('--seed', type=int, default=0)
    args = ap.parse_args()
    positions = (0.5,) if args.quick else WINDOW_POS
    distances = (20, 40, 80, 130, 200) if args.quick else DISTANCES

    OUT.mkdir(exist_ok=True)
    jobs = [(b, positions, distances, args.seed) for b in args.bags]
    with Pool(args.workers or min(len(jobs), 7)) as pool:
        rows = [r for part in pool.imap_unordered(run_bag, jobs) for r in part]
    rows.sort(key=lambda r: (r['bag'], r['window'], r['lat_m'], r['shape'], r['fwd_m']))
    with open(OUT / 'eval_injection.csv', 'w', newline='', encoding='utf-8') as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)
    print(f'\n{len(rows)} испытаний -> {OUT / "eval_injection.csv"}')
    summarize(rows, distances)


if __name__ == '__main__':
    main()
