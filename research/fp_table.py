"""Ложные тревоги на пустых записях по вариантам: python research/fp_table.py A_base B_far_m0 ..."""
import csv, sys
import numpy as np
for tag in sys.argv[1:]:
    R = [r for r in csv.DictReader(open(f'runs/alarms_{tag}.csv')) if r['bag'] != 'doubleT_obstacle']
    ob = [r for r in csv.DictReader(open(f'runs/alarms_{tag}.csv')) if r['bag'] == 'doubleT_obstacle']
    a = np.array([r['alarm'] == '1' for r in R])
    ep = sum(1 for i in range(len(a)) if a[i] and (i == 0 or not a[i - 1]))
    d = np.array([float(r['distance_m']) for r in R if r['alarm'] == '1'])
    gt = sum(r['alarm'] == '1' and abs(float(r['distance_m']) - 56) < 2 for r in ob)
    print(f'{tag:<16} пустые: {100 * a.mean():5.1f}% кадров, эпизодов {ep:3d} ({ep / (len(a) / 36000):4.0f}/ч), '
          f'дист. <60/60-100/100+: {(d < 60).sum()}/{((d >= 60) & (d < 100)).sum()}/{(d >= 100).sum()};  '
          f'doubleT_obstacle: тревога у 56м в {gt}/201 кадрах')
