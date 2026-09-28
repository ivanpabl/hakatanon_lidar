"""Дальность обнаружения при подъезде: синтетический объект неподвижен в мире, поезд к нему едет.

В отличие от eval_injection.py (объект на постоянной дистанции от поезда), здесь
дистанция до объекта в каждом кадре уменьшается на пройденный поездом путь. Это то,
что оценивает заказчик (ТЗ §8.2): на какой дальности объект впервые уверенно найден.

Пройденный путь за кадр берётся из оценки скорости по сцене (EgoMotion) на тех же
кадрах без вставки. Окна выбираются там, где поезд едет (скорость > MIN_SPEED).

    python -m evaluation approach --tag base
    python -m evaluation approach --tag acc --det '{"method": "background"}'

Пишет runs/approach_<tag>.csv (строка = испытание) и печатает сводку:
доля найденных, медиана дальности первого подтверждения, ошибка дистанции,
посторонние тревоги в окне (подтверждённые объекты не у вставки).
"""
import argparse
import os
for _v in ('OMP_NUM_THREADS', 'OPENBLAS_NUM_THREADS', 'MKL_NUM_THREADS'):
    os.environ.setdefault(_v, '1')
import copy
import csv
import json
import warnings
import zlib
from multiprocessing import Pool

import numpy as np

from tunnel_od import ObstacleDetector
from tunnel_od.pointcloud import beam_directions, xyz_views
from tunnel_od.sim.inject import inject, on_track
from tunnel_od.sim.shapes import make_shape

from bags import RUNS as OUT, cloud_parser, open_cloud_bag
from eval_injection import front_of

BAGS = ['doubleT_platform', 'roundT_doubleT', 'roundT_pressureGate_roundT',
        'roundT_squareT_pressureGate_squareT', 'squareT_platform_squareT_switch']
WARMUP = 20
FRAMES = 50
MIN_SPEED = 4.0
N_WINDOWS = 3
START = (90, 130, 170, 220)
SHAPES = {
    'куб 0.2': ('box', (0.2, 0.2, 0.2)),
    'куб 0.4': ('box', (0.4, 0.4, 0.4)),
    'куб 0.7': ('box', (0.7, 0.7, 0.7)),
    'человек стоит': ('cylinder', (0.25, 1.7)),
    'человек лежит': ('box', (1.7, 0.5, 0.3)),
}


parse_pointcloud2 = cloud_parser('python')


def pick_windows(bag, n_windows, det_kwargs):
    """Проход по записи с оценкой скорости: старты окон, где поезд едет всё окно."""
    det = ObstacleDetector(ego_motion=True)
    speeds = []
    with open_cloud_bag(bag) as (reader, conn):
        t0 = reader.start_time
        for c, t, raw in reader.messages(connections=[conn]):
            m = reader.deserialize(raw, c.msgtype)
            res = det.detect(*parse_pointcloud2(m.data, m.point_step, m.fields),
                             refit_path=len(speeds) % 10 == 0, stamp=(t - t0) / 1e9)
            speeds.append(res['speed_mps'] or 0.0)
    speeds = np.array(speeds)
    need = WARMUP + FRAMES
    ok = np.array([speeds[i:i + need].min() > MIN_SPEED for i in range(max(0, len(speeds) - need))])
    cand = np.where(ok)[0]
    if not len(cand):
        return []
    starts = []
    for q in np.linspace(0.15, 0.85, n_windows):
        s = int(cand[int(q * (len(cand) - 1))])
        if all(abs(s - p) > need for p in starts):
            starts.append(s)
    return starts


def read_frames(bag, start, need):
    frames = []
    with open_cloud_bag(bag) as (reader, conn):
        t0 = reader.start_time
        for i, (c, t, raw) in enumerate(reader.messages(connections=[conn])):
            if i < start:
                continue
            m = reader.deserialize(raw, c.msgtype)
            frames.append((bytes(m.data), m.point_step, m.fields, (t - t0) / 1e9))
            if len(frames) == need:
                break
    return frames


def match(objs, front, tol_lat=0.8):
    tol = 2.0 + 0.03 * front
    return [o for o in objs if abs(o['distance_m'] - front) < tol and abs(o['lateral_m']) < tol_lat]


def run_bag(job):
    global parse_pointcloud2
    bag, det_kwargs, seed, parser = job
    warnings.simplefilter('ignore')
    parse_pointcloud2 = cloud_parser(parser)
    rows = []
    for w, start in enumerate(pick_windows(bag, N_WINDOWS, det_kwargs)):
        frames = read_frames(bag, start, WARMUP + FRAMES)
        ego = ObstacleDetector(ego_motion=True)
        ref = ObstacleDetector(**det_kwargs)
        for data, step, fields, t in frames[:WARMUP]:
            xyz = parse_pointcloud2(data, step, fields)
            ego.detect(*xyz, refit_path=True, stamp=t)
            ref.detect(*xyz, refit_path=True, stamp=t)
        start_state = copy.deepcopy(ref)
        prep, travelled = [], 0.0
        for k, (data, step, fields, t) in enumerate(frames[WARMUP:]):
            xyz = parse_pointcloud2(data, step, fields)
            e = ego.detect(*xyz, refit_path=True, stamp=t)
            if k:
                travelled += e['displacement_m'] or 0.0
            r = ref.detect(*xyz, refit_path=True, stamp=t)
            buf = np.frombuffer(data, np.uint8).reshape(-1, step)
            v = xyz_views(buf, fields)
            prep.append((data, step, fields, t, beam_directions(v['x'], v['y'], v['z']),
                         ref.track_path(), travelled, r))
        speed = travelled / max(prep[-1][3] - prep[0][3], 1e-3)

        for name, (kind, dims) in SHAPES.items():
            for d0 in START:
                rng = np.random.default_rng(zlib.crc32(repr((seed, bag, w, name, d0)).encode()))
                test = copy.deepcopy(start_state)
                first_d, err, n_hit, n_vis, n_frames, other = None, None, 0, 0, 0, 0
                for data, step, fields, t, dirs, path, trav, r_ref in prep:
                    d = d0 - trav
                    if d < 8:
                        break
                    n_frames += 1
                    lat, z0 = on_track(path, d)
                    ob = make_shape(kind, dims, d, lat, z0)
                    new, info = inject(data, step, [ob], fields, dirs=dirs, rng=rng)
                    n_vis += info['rays_visible'] >= 0.5 * max(info['rays_geometric'], 1) and info['rays_geometric'] > 0
                    res = test.detect(*parse_pointcloud2(new, step, fields), refit_path=True, stamp=t)
                    front = front_of(kind, dims, d)
                    conf = [o for o in res['objects'] if o['confirmed'] and not o.get('beyond_path')]
                    near = match(conf, front)
                    if near:
                        n_hit += 1
                        if first_d is None:
                            first_d = front
                            err = min(o['distance_m'] for o in near) - front
                    other += bool([o for o in conf if o not in near])
                rows.append({
                    'bag': bag, 'window': w, 'start_frame': start, 'speed_mps': round(speed, 1),
                    'shape': name, 'start_m': d0, 'frames': n_frames, 'visible_frames': int(n_vis),
                    'detected': int(first_d is not None),
                    'first_m': round(first_d, 1) if first_d is not None else '',
                    'dist_err_m': round(err, 2) if err is not None else '',
                    'frames_hit': n_hit, 'frames_other_alarm': other,
                })
        print(f'  {bag}: окно {w + 1} (кадр {start}, {speed:.1f} м/с) готово', flush=True)
    return rows


def summarize(rows):
    print(f'\n{"":<16}' + ''.join(f'{"старт " + str(d):>12}' for d in START) + f'{"все":>10}')
    for name in SHAPES:
        cells = []
        for d0 in list(START) + [None]:
            rs = [r for r in rows if r['shape'] == name and (d0 is None or r['start_m'] == d0)]
            if not rs:
                cells.append('-'); continue
            det = [r for r in rs if r['detected']]
            med = f'{np.median([r["first_m"] for r in det]):.0f}м' if det else '--'
            cells.append(f'{100 * len(det) / len(rs):.0f}%/{med}')
        print(f'{name:<16}' + ''.join(f'{c:>12}' for c in cells[:-1]) + f'{cells[-1]:>10}')
    det = [r for r in rows if r['detected']]
    if det:
        print(f'\nОшибка дистанции при первом обнаружении: медиана {np.median([abs(r["dist_err_m"]) for r in det]):.2f}м, '
              f'95% {np.percentile([abs(r["dist_err_m"]) for r in det], 95):.2f}м')
    fr = sum(r['frames'] for r in rows)
    print(f'Кадров с посторонней тревогой (не у вставки): {100 * sum(r["frames_other_alarm"] for r in rows) / max(fr, 1):.1f}%')
    print('Ячейка: доля найденных / медиана дальности первого подтверждения (до передней грани).')


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--tag', default='v0')
    ap.add_argument('--bags', nargs='+', default=BAGS)
    ap.add_argument('--det', default='{}', help='kwargs ObstacleDetector в JSON')
    ap.add_argument('--workers', type=int, default=3)
    ap.add_argument('--seed', type=int, default=0)
    ap.add_argument('--parser', default='python', choices=['python', 'cpp'],
                    help='разбор облака: python -- parse_pointcloud2, cpp -- как C++-узел приёма (tunnel_od_preproc)')
    args = ap.parse_args()
    det_kwargs = json.loads(args.det)
    OUT.mkdir(exist_ok=True)
    with Pool(min(len(args.bags), args.workers)) as pool:
        rows = [r for part in pool.imap_unordered(run_bag, [(b, det_kwargs, args.seed, args.parser) for b in args.bags]) for r in part]
    rows.sort(key=lambda r: (r['bag'], r['window'], r['shape'], r['start_m']))
    path = OUT / f'approach_{args.tag}.csv'
    with open(path, 'w', newline='', encoding='utf-8') as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)
    print(f'\n{args.tag}: {det_kwargs}; {len(rows)} испытаний -> {path}')
    summarize(rows)


if __name__ == '__main__':
    main()
