"""Истина по синтетическим объектам бэга организаторов cloud_with_fake_obj -- прямо из облака.

Генератор организаторов дописывает точки объектов В КОНЕЦ буфера PointCloud2: после последней
нулевой точки (0,0,0) реального облака идут только синтетические точки, все с intensity = 1.0
(перекрытые объектом реальные точки удалены, поэтому число точек в кадре != 307200). Скрипт
берёт этот хвост в каждом кадре, режет на кластеры (связность вокселей 0,5 м) и сводит
кластеры в объекты по координате X = travel_m + ближняя дистанция кластера (travel_m -- из
прогона alarms.py с тем же тегом, что у eval_fake_obj.py).

    python research/fake_obj_truth.py --tag fo_base0            # таблица объектов по X
    python research/fake_obj_truth.py --tag fo_base0 --csv runs/fake_obj_truth.csv   # + по кадрам

Столбцы таблицы: X (середина 5-95 % по кадрам, где объект на 2-150 м), разброс X, кадры, где
есть точки объекта, габариты в системе лидара (x -- вбок, z -- вверх) на 2-25 м. Так построен
эталон reference/cloud_with_fake_obj/objects.yaml (задача 8).
"""
import argparse
import csv
import sys
from pathlib import Path

import numpy as np
from scipy import ndimage

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / 'tools'))
from bags import FAKE_OBJ, RUNS, open_cloud_bag          # noqa: E402
from tunnel_od.pointcloud import xyz_views              # noqa: E402

VOXEL = 0.5
GAP_X = 15.0


def synthetic_tail(m):
    """(fwd, lat, z) синтетических точек кадра: хвост буфера после последней нулевой точки."""
    buf = np.frombuffer(m.data, np.uint8).reshape(-1, m.point_step)
    v = xyz_views(buf, m.fields)
    zero = (v['x'] == 0) & (v['y'] == 0) & (v['z'] == 0)
    idx = np.nonzero(zero)[0]
    start = int(idx[-1]) + 1 if len(idx) else len(zero)
    tail = slice(start, None)
    if not np.all(v['intensity'][tail] == 1.0):
        print(f'  внимание: в хвосте есть intensity != 1 ({m.header.stamp})', file=sys.stderr)
    return np.c_[-v['y'][tail], v['x'][tail], v['z'][tail]].astype(np.float64)


def clusters(p):
    """Кластеры точек p (N, 3): связность вокселей VOXEL с одним шагом расширения."""
    if not len(p):
        return []
    v = np.floor(p / VOXEL).astype(int)
    v -= v.min(0)
    grid = np.zeros(v.max(0) + 1, bool)
    grid[tuple(v.T)] = True
    lab, _ = ndimage.label(ndimage.binary_dilation(grid), structure=np.ones((3, 3, 3)))
    pl = lab[tuple(v.T)]
    return [p[pl == c] for c in np.unique(pl)]


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--tag', required=True, help='тег прогона alarms.py (берётся travel_m по кадрам)')
    ap.add_argument('--csv', default=None, help='записать кластеры по кадрам (frame, obj, dist_m, n, x/z-габариты)')
    args = ap.parse_args()
    travel = {int(r['frame']): float(r['travel_m'])
              for r in csv.DictReader(open(RUNS / f'alarms_{args.tag}.csv', encoding='utf-8')) if r['bag'] == FAKE_OBJ}
    rows = []
    with open_cloud_bag(FAKE_OBJ) as (reader, conn):
        for i, (c, _, raw) in enumerate(reader.messages(connections=[conn])):
            for q in clusters(synthetic_tail(reader.deserialize(raw, c.msgtype))):
                d = float(q[:, 0].min())
                rows.append({'frame': i, 'X': travel[i] + d, 'dist_m': d, 'n': len(q),
                             'x_min': q[:, 1].min(), 'x_max': q[:, 1].max(), 'z_min': q[:, 2].min(), 'z_max': q[:, 2].max()})
    rows.sort(key=lambda r: r['X'])
    groups, obj = [], []
    for r in rows:
        if obj and r['X'] - obj[-1]['X'] > GAP_X:
            groups.append(obj)
            obj = []
        obj.append(r)
    if obj:
        groups.append(obj)
    print(f'{"#":>3}{"X, м":>9}{"шаг":>7}{"X мин-макс":>16}{"кадры":>11}  габариты на 2-25 м (x; z лидара)')
    prev = None
    for k, g in enumerate(groups, 1):
        use = [r for r in g if 2 <= r['dist_m'] <= 150 and r['n'] >= 2] or g
        xs = np.array([r['X'] for r in use])
        p5, p95 = np.percentile(xs, [5, 95])
        x = (p5 + p95) / 2
        near = [r for r in g if 2 <= r['dist_m'] <= 25] or g
        med = lambda key: float(np.median([r[key] for r in near]))
        fr = [r['frame'] for r in g]
        print(f'{k:>3}{x:>9.1f}{"" if prev is None else f"{x - prev:.0f}":>7}{f"{xs.min():.1f}-{xs.max():.1f}":>16}'
              f'{f"{min(fr)}-{max(fr)}":>11}  x {med("x_min"):+.2f}..{med("x_max"):+.2f}; z {med("z_min"):+.2f}..{med("z_max"):+.2f}')
        for r in g:
            r['obj'] = k
        prev = x
    if args.csv:
        with open(args.csv, 'w', newline='', encoding='utf-8') as f:
            w = csv.DictWriter(f, fieldnames=['frame', 'obj', 'X', 'dist_m', 'n', 'x_min', 'x_max', 'z_min', 'z_max'])
            w.writeheader()
            for r in sorted(rows, key=lambda r: (r['frame'], r['dist_m'])):
                w.writerow({k: (round(v, 3) if isinstance(v, float) else v) for k, v in r.items()})
        print(f'по кадрам: {args.csv}')


if __name__ == '__main__':
    main()
