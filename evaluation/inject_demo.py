# © 2026 команда «Голуби», github.com/ivanpabl/hakatanon_lidar. Все права защищены, условия — в файле LICENSE. GLB-K5-4660c47a8c6c
"""Вставка синтетического объекта в несколько кадров записи подряд: как его видит детектор,
плюс картинки кадра до и после вставки (output/runs/inject_*.png).

    python -m evaluation inject-demo --bag roundT_doubleT --start 100 --shape box --dims 0.5,0.5,1.45 --fwd 56
    python -m evaluation inject-demo --bag roundT_doubleT --shape cylinder --dims 0.25,1.7 --fwd 120
"""
import argparse
import warnings

import numpy as np

from tunnel_od import ObstacleDetector, parse_pointcloud2
from tunnel_od.sim.inject import inject, on_track
from tunnel_od.sim.shapes import make_shape

from bags import RUNS, open_cloud_bag
from render import render_frame

WARMUP = 20


def main():
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
    warnings.simplefilter('ignore')

    rng = np.random.default_rng(args.seed)
    ref = ObstacleDetector()
    test = ObstacleDetector()
    last = None
    print(f'{args.shape} {dims} на {args.fwd:.0f}м, смещение {args.lat:+.2f}м от оси; кадры {args.start}..{args.start + args.frames - 1}')
    print(f'{"кадр":>5} {"точек на объекте":>17} {"без вставки":>14} {"со вставкой":>14}  объект около {args.fwd:.0f}м')
    with open_cloud_bag(args.bag) as (reader, conn):
        for i, (c, t, raw) in enumerate(reader.messages(connections=[conn])):
            if i < args.start - WARMUP:
                continue
            if i >= args.start + args.frames:
                break
            msg = reader.deserialize(raw, c.msgtype)
            x, y, z = parse_pointcloud2(msg.data, msg.point_step, msg.fields)
            res_ref = ref.detect(x, y, z, refit_path=True)
            if i < args.start:
                test.detect(x, y, z, refit_path=True)
                continue
            lat, z0 = on_track(ref.track_path(), args.fwd, args.lat)
            ob = make_shape(args.shape, dims, args.fwd, lat, z0, args.yaw)
            data, info = inject(msg.data, msg.point_step, [ob], msg.fields, rng=rng)
            xi, yi, zi = parse_pointcloud2(data, msg.point_step, msg.fields)
            res = test.detect(xi, yi, zi, refit_path=True)
            near = [o for o in res['objects'] if abs(o['distance_m'] - args.fwd) < 3.0]
            fmt = lambda r: f"тревога {r['distance_m']:.1f}м" if r['obstacle'] else 'нет тревоги'
            nd = (f"{len(near)} кл., {sum(o['n_points'] for o in near)} т., "
                  f"{'подтверждён' if any(o['confirmed'] for o in near) else 'не подтверждён'}"
                  + (', за осью пути' if any(o['beyond_path'] for o in near) else '')) if near else 'не виден'
            print(f"{i:>5} {info['points_on_object']:>17} {fmt(res_ref):>14} {fmt(res):>14}  {nd}")
            last = (i, x, y, z, xi, yi, zi, ref.track_path(), test.track_path())

    i, x, y, z, xi, yi, zi, p_ref, p_test = last
    RUNS.mkdir(exist_ok=True)
    tag = f"{args.bag}_f{i}_{args.shape}_{args.dims.replace(',', 'x')}_{args.fwd:.0f}m"
    render_frame(RUNS / f'inject_{tag}_before.png', x, y, z, p_ref, f'{args.bag} кадр {i}: без вставки')
    render_frame(RUNS / f'inject_{tag}_after.png', xi, yi, zi, p_test,
                 f'{args.bag} кадр {i}: {args.shape} {args.dims} м на {args.fwd:.0f} м, {args.lat:+.1f} м от оси')
    print('saved', RUNS / f'inject_{tag}_before.png', 'и _after.png')


if __name__ == '__main__':
    main()
