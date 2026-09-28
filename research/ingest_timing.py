"""Стоимость приёма кадра без ROS 2 (медиана/p95 по кадрам): разбор parse_pointcloud2,
передача x, y, z в дочерний процесс так, как это делает path_worker (pickle через
multiprocessing.Pipe), и detect (sync, путь в каждом кадре).

    python research/ingest_timing.py doubleT_obstacle doubleT_platform --frames 30
"""
import argparse
import multiprocessing as mp
import pickle
import sys
import time
import warnings
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / 'tools'))
warnings.simplefilter('ignore')
import numpy as np

from tunnel_od import ObstacleDetector, parse_pointcloud2
from bags import open_cloud_bag


def _echo(conn):
    while True:
        job = conn.recv()
        if job is None:
            return
        conn.send(time.perf_counter())


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('bags', nargs='+')
    ap.add_argument('--frames', type=int, default=30)
    ap.add_argument('--skip', type=int, default=20)
    args = ap.parse_args()
    ctx = mp.get_context('spawn')
    a, b = ctx.Pipe()
    proc = ctx.Process(target=_echo, args=(b,), daemon=True)
    proc.start()
    for bag in args.bags:
        T = {}
        tm = lambda k, v: T.setdefault(k, []).append(1e3 * v)
        det = ObstacleDetector()
        with open_cloud_bag(bag) as (reader, conn):
            for i, (c, t, raw) in enumerate(reader.messages(connections=[conn])):
                if i < args.skip:
                    continue
                if i >= args.skip + args.frames:
                    break
                msg = reader.deserialize(raw, c.msgtype)
                t0 = time.perf_counter()
                x, y, z = parse_pointcloud2(msg.data, msg.point_step, msg.fields)
                t1 = time.perf_counter()
                tm('parse', t1 - t0)
                t0 = time.perf_counter(); blob = pickle.dumps((i, x, y, z), protocol=pickle.HIGHEST_PROTOCOL)
                tm('pickle.dumps', time.perf_counter() - t0)
                t0 = time.perf_counter(); a.send((i, x, y, z)); t_child = a.recv()
                tm('pipe send->child recv', t_child - t0)
                t0 = time.perf_counter(); det.detect(x, y, z, refit_path=True, stamp=t / 1e9)
                tm('detect(refit)', time.perf_counter() - t0)
                t0 = time.perf_counter(); det.check_frame(x, y, z, stamp=t / 1e9 + 0.1)
                tm('check_frame', time.perf_counter() - t0)
                T.setdefault('_n_in', []).append(msg.width * msg.height)
                T.setdefault('_n_out', []).append(len(x))
                T.setdefault('_mb', []).append(len(msg.data) / 1e6)
        print(f'== {bag}: {np.mean(T.pop("_mb")):.1f} МБ, точек {int(np.mean(T.pop("_n_in")))} -> '
              f'{int(np.mean(T.pop("_n_out")))}, pickle {len(blob) / 1e6:.1f} МБ')
        for k, v in T.items():
            print(f'   {k:<24} p50 {np.median(v):7.1f} мс   p95 {np.percentile(v, 95):7.1f} мс')
    a.send(None)
    proc.join()


if __name__ == '__main__':
    main()
