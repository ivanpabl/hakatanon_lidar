"""Массовая оценка детектора на синтетических препятствиях (inject.py) в реальных кадрах.

Для каждой записи берутся несколько окон по FRAMES кадров подряд. В каждое окно
по очереди вставляется объект из сетки (фигура x дальность x смещение от оси) на
постоянной дистанции от поезда, и детектор прогоняется по окну. Тот же детектор
на тех же кадрах без вставки -- базовая линия: если он и сам видит что-то в том
же месте, испытание помечается как неоднозначное и в долю найденных не входит.

    python tools/eval_injection.py                    # все записи, полная сетка (~20-30 мин на 2 процессах)
    python tools/eval_injection.py --quick            # 1 окно на запись, меньше дальностей
    python tools/eval_injection.py --bags roundT_doubleT doubleT_platform
    python tools/eval_injection.py --corridor gauge   # габарит вместо прямоугольника +-1м
    python tools/eval_injection.py --minpts off       # без порога числа точек

Пишет runs/eval_injection_<вариант>.csv (строка = испытание) и .png,
печатает таблицы: доля найденных по фигуре и дальности, число точек на объекте,
тревоги от объектов вне коридора.
"""
import argparse
import os
for _v in ('OMP_NUM_THREADS', 'OPENBLAS_NUM_THREADS', 'MKL_NUM_THREADS'):
    os.environ.setdefault(_v, '1')  # numpy по одному потоку на процесс: иначе каждый процесс берёт все ядра
import copy
import csv
import warnings
import zlib
from multiprocessing import Pool

import numpy as np

from tunnel_od import GAUGE_METRO, ObstacleDetector, parse_pointcloud2
from tunnel_od.pointcloud import beam_directions, xyz_views
from tunnel_od.sim.inject import inject, on_track
from tunnel_od.sim.shapes import make_shape

from bags import BAGS, RUNS as OUT, open_cloud_bag
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
# размещение объекта поперёк пути: ближний к оси край объекта, м
#   axis    -- центр объекта на оси пути;
#   edge    -- край в 1.1м от оси: задевает вагон (кузов ~1.35м), но вне старого коридора 1.0м;
#   outside -- край в 1.6м: вне габарита, тревоги быть не должно
PLACEMENTS = {'edge': 1.1, 'outside': 1.6}
SIDE_SHAPES = ('куб 0.4', 'куб 1.0', 'человек стоит')
CORRIDORS = {'rect': None, 'gauge': GAUGE_METRO}
MINPTS = {'on': {}, 'off': {'min_points_k': 0.0, 'min_points_floor': 1}}  # порог точек по дальности  # rect -- прежний прямоугольник +-1м x 0.15-2м


def half_lat(kind, dims):
    return dims[1] / 2 if kind == 'box' else dims[0]


def front_of(kind, dims, fwd):
    """Ближняя к лидару дальность объекта -- её детектор и отдаёт как distance_m."""
    return fwd - (dims[0] / 2 if kind == 'box' else dims[0])


def near_objects(res, front, lat, tol_lat=0.8):
    tol = 1.5 + 0.02 * front
    return [o for o in res['objects']
            if abs(o['distance_m'] - front) < tol and abs(o['lateral_m'] - lat) < tol_lat]


def read_windows(bag, positions, need):
    """Кадры окон: список окон, окно = список (msg_data, point_step, fields) длиной need."""
    windows = []
    with open_cloud_bag(bag) as (reader, conn):
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
    bag, positions, distances, seed, corridor, minpts = job
    make_det = lambda: ObstacleDetector(zone=CORRIDORS[corridor], **MINPTS[minpts])
    warnings.simplefilter('ignore')
    rows = []
    windows = read_windows(bag, positions, WARMUP + FRAMES)
    trials = [(name, 'axis', 0.0) for name in SHAPES]
    for place, edge in PLACEMENTS.items():
        trials += [(name, place, edge + half_lat(*SHAPES[name])) for name in SIDE_SHAPES]
    for w, frames in enumerate(windows):
        # чистый проход: прогрев, затем по кадрам окна -- размещение объектов и базовый результат
        ref = make_det()
        for data, step, fields, _ in frames[:WARMUP]:
            ref.detect(*parse_pointcloud2(data, step, fields), refit_path=True)
        start_state = copy.deepcopy(ref)
        prep = []
        for data, step, fields, t in frames[WARMUP:]:
            xyz = parse_pointcloud2(data, step, fields)
            res = ref.detect(*xyz, refit_path=True)
            buf = np.frombuffer(data, np.uint8).reshape(-1, step)
            v = xyz_views(buf, fields)
            dirs = beam_directions(v['x'], v['y'], v['z'])
            path = ref.track_path()
            place = {d: on_track(path, d) for d in distances}
            prep.append((data, step, fields, dirs, res, place, ref.path_range))
        t_win = frames[WARMUP][3]

        for name, placement, lat in trials:
            kind, dims = SHAPES[name]
            for d in distances:
                rng = np.random.default_rng(zlib.crc32(repr((seed, bag, w, name, placement, d)).encode()))
                front = front_of(kind, dims, d)
                test = copy.deepcopy(start_state)
                det, amb, beyond, alarm_on, pts, errs, in_path = [], 0, 0, 0, [], [], []
                geo = vis = 0
                for data, step, fields, dirs, res_ref, place, prange in prep:
                    lat_axis, z0 = place[d]
                    ob = make_shape(kind, dims, d, lat_axis + lat, z0)
                    new, info = inject(data, step, [ob], fields, dirs=dirs, rng=rng)
                    res = test.detect(*parse_pointcloud2(new, step, fields), refit_path=True)
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
                    'corridor': corridor, 'minpts': minpts, 'placement': placement, 'bag': bag, 'window': w, 't_start_s': round(t_win, 1), 'shape': name, 'kind': kind,
                    'dims': 'x'.join(map(str, dims)), 'fwd_m': d, 'lat_m': round(lat, 2), 'front_m': round(front, 2),
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


def summarize(rows, distances, tag):
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

    inside = [r for r in rows if r['placement'] == 'axis' and not r['occluded']]
    clean = [r for r in inside if not r['ambiguous']]
    occl = [r for r in rows if r['placement'] == 'axis' and r['occluded']]
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
    for place, edge in PLACEMENTS.items():
        rs = [r for r in rows if r['placement'] == place and not r['ambiguous'] and not r['occluded']]
        if not rs:
            continue
        hit = [r for r in rs if r['detected']]
        what = 'найдено (объект задевает вагон)' if place == 'edge' else 'тревога по объекту (не должно быть)'
        print(f'Край объекта в {edge}м от оси: {what} -- {len(hit)}/{len(rs)} испытаний')
        for name in SIDE_SHAPES:
            line = []
            for d in distances:
                x = [r['detected'] for r in rs if r['shape'] == name and r['fwd_m'] == d]
                line.append(f'{100 * np.mean(x):.0f}%' if x else '-')
            print(f'    {name:<16}' + ''.join(f'{c:>8}' for c in line))

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
    fig.savefig(OUT / f'eval_injection_{tag}.png', dpi=120)
    print('\nsaved', OUT / f'eval_injection_{tag}.png')


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--bags', nargs='+', default=BAGS)
    ap.add_argument('--quick', action='store_true', help='1 окно на запись, дальности 20/40/80/130/200')
    ap.add_argument('--workers', type=int, default=2)
    ap.add_argument('--seed', type=int, default=0)
    ap.add_argument('--minpts', default='on', choices=list(MINPTS), help='порог числа точек по дальности')
    ap.add_argument('--corridor', default='rect', choices=list(CORRIDORS),
                    help='rect -- прямоугольник +-1м x 0.15-2м (по умолчанию в детекторе), gauge -- габарит GAUGE_METRO')
    args = ap.parse_args()
    positions = (0.5,) if args.quick else WINDOW_POS
    distances = (20, 40, 80, 130, 200) if args.quick else DISTANCES

    OUT.mkdir(exist_ok=True)
    tag = args.corridor + ('' if args.minpts == 'on' else '_nominpts') + ('_quick' if args.quick else '')
    jobs = [(b, positions, distances, args.seed, args.corridor, args.minpts) for b in args.bags]
    with Pool(min(len(jobs), args.workers)) as pool:
        rows = [r for part in pool.imap_unordered(run_bag, jobs) for r in part]
    rows.sort(key=lambda r: (r['bag'], r['window'], r['placement'], r['shape'], r['fwd_m']))
    with open(OUT / f'eval_injection_{tag}.csv', 'w', newline='', encoding='utf-8') as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)
    print(f'\nКоридор: {args.corridor}; {len(rows)} испытаний -> {OUT / f"eval_injection_{tag}.csv"}')
    summarize(rows, distances, tag)


if __name__ == '__main__':
    main()
