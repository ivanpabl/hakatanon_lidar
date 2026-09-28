"""Прогон детектора по реальным записям без вставок: сколько тревог он поднимает.

Сохраняет результат по каждому кадру, чтобы после изменений детектора сравнить
версии на тех же кадрах (какие тревоги появились / исчезли).

    python -m evaluation alarms --tag v2                       # все записи -> runs/alarms_v2.csv
    python -m evaluation alarms --tag v2g --corridor gauge     # габарит вместо прямоугольника +-1м
    python -m evaluation alarms --tag v2n --minpts off         # без порога числа точек
    python -m evaluation alarms --tag test --bags roundT_doubleT

Эпизод тревоги -- подряд идущие кадры с тревогой (разрывы до GAP кадров склеиваются).
Какие из тревог ложные, скрипт не знает: заведомо чистых записей нет
(в doubleT_obstacle есть объект на ~56м).
"""
import argparse
import os
for _v in ('OMP_NUM_THREADS', 'OPENBLAS_NUM_THREADS', 'MKL_NUM_THREADS'):
    os.environ.setdefault(_v, '1')
import csv
import json
import warnings
from multiprocessing import Pool

import numpy as np

from tunnel_od import GAUGE_METRO, ObstacleDetector

from bags import BAGS, RUNS as OUT, cloud_parser, open_cloud_bag
GAP = 2


CORRIDORS = {'rect': None, 'gauge': GAUGE_METRO}
MINPTS = {'on': {}, 'off': {'min_points_k': 0.0, 'min_points_floor': 1}}


def _stream(reader, conn, segments, seglen):
    """(номер кадра, t, сообщение): вся запись или segments отрезков по seglen кадров,
    равномерно по времени (детектор перезапускается в начале каждого отрезка)."""
    if not segments:
        for i, (c, t, raw) in enumerate(reader.messages(connections=[conn])):
            yield i, t, c, raw, i == 0
        return
    t0, dur = reader.start_time, reader.duration
    for k in range(segments):
        start = t0 + int(dur * (k + 0.5) / segments)
        for j, (c, t, raw) in enumerate(reader.messages(connections=[conn], start=start)):
            if j == seglen:
                break
            yield k * seglen + j, t, c, raw, j == 0


def run_bag(job):
    bag, corridor, minpts, method, det_kwargs, segments, seglen, parser = job
    warnings.simplefilter('ignore')
    parse_pointcloud2 = cloud_parser(parser)
    make = lambda: ObstacleDetector(zone=CORRIDORS[corridor], method=method, **{**MINPTS[minpts], **det_kwargs})
    rows = []
    with open_cloud_bag(bag) as (reader, conn):
        t0 = reader.start_time
        for i, t, c, raw, fresh in _stream(reader, conn, segments, seglen):
            if fresh:
                det = make()
            m = reader.deserialize(raw, c.msgtype)
            res = det.detect(*parse_pointcloud2(m.data, m.point_step, m.fields), refit_path=True, stamp=t / 1e9)
            conf = [o for o in res['objects'] if o['confirmed'] and not o['beyond_path']]
            near = min(conf, key=lambda o: o['distance_m']) if conf else None
            rows.append({'bag': bag, 'frame': i, 't_s': round((t - t0) / 1e9, 2), 'alarm': int(res['obstacle']),
                         'distance_m': round(res['distance_m'], 1) if res['obstacle'] else '',
                         'lateral_m': round(near['lateral_m'], 2) if near else '',
                         'height_m': round(near['height_m'], 2) if near else '',
                         'n_points': near['n_points'] if near else '',
                         'n_confirmed': len(conf),
                         'path_range_m': round(res['path_range_m'], 0) if res['path_range_m'] else ''})
            if i % 2000 == 0 and i:
                print(f'  {bag}: {i} кадров', flush=True)
    return rows


def episodes(alarm):
    idx = np.where(alarm)[0]
    if not len(idx):
        return []
    return np.split(idx, np.where(np.diff(idx) > GAP + 1)[0] + 1)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--tag', default='v0', help='метка версии детектора для имени файла')
    ap.add_argument('--bags', nargs='+', default=BAGS)
    ap.add_argument('--corridor', default='rect', choices=list(CORRIDORS))
    ap.add_argument('--minpts', default='on', choices=list(MINPTS), help='порог числа точек по дальности')
    ap.add_argument('--method', default='zone', choices=['zone', 'background'], help='zone -- точки в зоне; background -- остаток до фона тоннеля')
    ap.add_argument('--workers', type=int, default=2, help='параллельных процессов (каждый -- ядро и до ~0.5ГБ)')
    ap.add_argument('--det', default='{}', help='доп. kwargs ObstacleDetector в JSON')
    ap.add_argument('--segments', type=int, default=0, help='не вся запись, а столько отрезков (для длинной new_data)')
    ap.add_argument('--seglen', type=int, default=400, help='кадров в отрезке')
    ap.add_argument('--parser', default='python', choices=['python', 'cpp'],
                    help='разбор облака: python -- parse_pointcloud2, cpp -- как C++-узел приёма (tunnel_od_preproc)')
    args = ap.parse_args()
    OUT.mkdir(exist_ok=True)
    with Pool(min(len(args.bags), args.workers)) as pool:
        parts = pool.map(run_bag, [(b, args.corridor, args.minpts, args.method, json.loads(args.det),
                                    args.segments, args.seglen, args.parser) for b in args.bags])
    rows = [r for p in parts for r in p]
    path = OUT / f'alarms_{args.tag}.csv'
    with open(path, 'w', newline='', encoding='utf-8') as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)

    print(f'\n{"запись":<37}{"кадров":>7}{"мин":>6}{"кадров с тревогой":>19}{"эпизодов":>10}{"эпизодов/ч":>11}'
          f'{"дист. медиана":>14}{"1-2 точки":>11}')
    tot = [0, 0, 0, 0.0]
    for bag, part in zip(args.bags, parts):
        a = np.array([r['alarm'] for r in part])
        dur = len(part) / 10.0 / 3600
        ep = episodes(a)
        d = [r['distance_m'] for r in part if r['alarm']]
        small = [r for r in part if r['alarm'] and r['n_points'] != '' and r['n_points'] <= 2]
        print(f'{bag:<37}{len(a):>7}{60 * dur:>6.1f}{a.sum():>9} ({100 * a.mean():4.1f}%){len(ep):>10}'
              f'{len(ep) / dur:>11.0f}{(f"{np.median(d):.0f}м" if d else "-"):>14}'
              f'{(f"{100 * len(small) / max(a.sum(), 1):.0f}%"):>11}')
        tot = [tot[0] + len(a), tot[1] + a.sum(), tot[2] + len(ep), tot[3] + dur]
    print(f'{"ВСЕГО":<37}{tot[0]:>7}{60 * tot[3]:>6.1f}{tot[1]:>9} ({100 * tot[1] / tot[0]:4.1f}%){tot[2]:>10}'
          f'{tot[2] / tot[3]:>11.0f}')
    print(f'\n"1-2 точки" -- доля кадров с тревогой, где ближайший подтверждённый объект из 1-2 точек.')
    print(f'Метод: {args.method}, коридор: {args.corridor}, порог точек: {args.minpts}. Записано: {path}')


if __name__ == '__main__':
    main()
