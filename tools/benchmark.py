"""Время обработки кадра по реальным записям: разбор облака, пересчёт пути, проверка кадра.
Без ROS 2 -- чисто алгоритмический замер (критерий ТЗ 8.3: скорость).

    python tools/benchmark.py                       # 6 записей из data/Датасет, по 15 кадров
    python tools/benchmark.py --bags new_data --frames 50
"""
import os
for _v in ('OMP_NUM_THREADS', 'OPENBLAS_NUM_THREADS', 'MKL_NUM_THREADS'):
    os.environ.setdefault(_v, '1')
import argparse
import time
import warnings

import numpy as np

from tunnel_od import ObstacleDetector, parse_pointcloud2

from bags import BAGS, open_cloud_bag

WARMUP = 3


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--bags', nargs='+', default=[b for b in BAGS if b != 'new_data'])
    ap.add_argument('--frames', type=int, default=15, help='кадров на запись (после прогрева)')
    args = ap.parse_args()
    warnings.simplefilter('ignore')

    print(f'{"запись":<38} {"точек/кадр":>10} {"разбор мс":>10} {"проверка мс":>12} {"путь мс":>9} {"FPS*":>6}')
    print('-' * 90)
    overall = []
    for bag in args.bags:
        det = ObstacleDetector()
        t_parse, t_check, t_refit, npts = [], [], [], []
        with open_cloud_bag(bag) as (reader, conn):
            for i, (c, t, raw) in enumerate(reader.messages(connections=[conn])):
                if i >= WARMUP + args.frames:
                    break
                msg = reader.deserialize(raw, c.msgtype)
                t0 = time.perf_counter()
                x, y, z = parse_pointcloud2(msg.data, msg.point_step, msg.fields)
                t1 = time.perf_counter()
                det.update_path(x, y, z)
                t2 = time.perf_counter()
                det.check_frame(x, y, z)
                t3 = time.perf_counter()
                if i >= WARMUP:
                    t_parse.append(t1 - t0); t_refit.append(t2 - t1); t_check.append(t3 - t2)
                    npts.append(len(x))
        p_ms, c_ms, r_ms = (np.mean(t) * 1000 for t in (t_parse, t_check, t_refit))
        print(f'{bag:<38} {int(np.mean(npts)):>10} {p_ms:>10.1f} {c_ms:>12.1f} {r_ms:>9.1f} {1000 / (p_ms + c_ms + r_ms):>6.0f}')
        overall.append((p_ms, c_ms, r_ms, np.mean(npts)))

    p_ms, c_ms, r_ms, n = np.mean(overall, axis=0)
    print('-' * 90)
    print(f'{"СРЕДНЕЕ":<38} {int(n):>10} {p_ms:>10.1f} {c_ms:>12.1f} {r_ms:>9.1f} {1000 / (p_ms + c_ms + r_ms):>6.0f}')
    print('\n* FPS при пересчёте пути на КАЖДОМ кадре; "точек/кадр" -- после очистки от нулей и дублей.')


if __name__ == '__main__':
    main()
