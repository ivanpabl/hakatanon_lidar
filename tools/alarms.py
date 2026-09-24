"""Прогон детектора по реальным записям без вставок: сколько тревог он поднимает.

Сохраняет результат по каждому кадру, чтобы после изменений детектора сравнить
версии на тех же кадрах (какие тревоги появились / исчезли).

    python tools/alarms.py --tag v2                       # все записи -> runs/alarms_v2.csv
    python tools/alarms.py --tag v2g --corridor gauge     # габарит вместо прямоугольника +-1м
    python tools/alarms.py --tag v2n --minpts off         # без порога числа точек
    python tools/alarms.py --tag test --bags roundT_doubleT

Эпизод тревоги -- подряд идущие кадры с тревогой (разрывы до GAP кадров склеиваются).
Какие из тревог ложные, скрипт не знает: заведомо чистых записей нет
(в doubleT_obstacle есть объект на ~56м).
"""
import argparse
import os
for _v in ('OMP_NUM_THREADS', 'OPENBLAS_NUM_THREADS', 'MKL_NUM_THREADS'):
    os.environ.setdefault(_v, '1')  # numpy по одному потоку на процесс: иначе каждый процесс берёт все ядра
import csv
import warnings
from multiprocessing import Pool

import numpy as np

from tunnel_od import GAUGE_METRO, ObstacleDetector, parse_pointcloud2

from bags import BAGS, RUNS as OUT, open_cloud_bag
GAP = 2


CORRIDORS = {'rect': None, 'gauge': GAUGE_METRO}
MINPTS = {'on': {}, 'off': {'min_points_k': 0.0, 'min_points_floor': 1}}  # порог точек по дальности  # rect -- прежний прямоугольник +-1м x 0.15-2м


def run_bag(job):
    bag, corridor, minpts = job
    warnings.simplefilter('ignore')
    det = ObstacleDetector(zone=CORRIDORS[corridor], **MINPTS[minpts])
    rows = []
    with open_cloud_bag(bag) as (reader, conn):
        t0 = None
        for i, (c, t, raw) in enumerate(reader.messages(connections=[conn])):
            t0 = t if t0 is None else t0
            m = reader.deserialize(raw, c.msgtype)
            res = det.detect(*parse_pointcloud2(m.data, m.point_step, m.fields), refit_path=True)
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
    ap.add_argument('--workers', type=int, default=2, help='параллельных процессов (каждый -- ядро и до ~0.5ГБ)')
    args = ap.parse_args()
    OUT.mkdir(exist_ok=True)
    with Pool(min(len(args.bags), args.workers)) as pool:
        parts = pool.map(run_bag, [(b, args.corridor, args.minpts) for b in args.bags])
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
        dur = part[-1]['t_s'] / 3600
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
    print(f'Коридор: {args.corridor}, порог точек: {args.minpts}. Записано: {path}')


if __name__ == '__main__':
    main()
