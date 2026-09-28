"""Запись ros2 bag с синтетическим объектом: испытание подъезда из eval_approach, сохранённое как запись.

    python -m evaluation inject-bag --bag roundT_squareT_pressureGate_squareT --window 1 --start-m 220 \\
        --out data/synthetic/person_220m

Повторяет испытание eval_approach.py один в один: то же окно записи (--window -- номер окна, как в
approach_*.csv), WARMUP кадров без объекта, затем объект неподвижен в мире -- дистанция каждый кадр
уменьшается на путь поезда (оценка скорости по сцене), объект стоит на оси пути, найденной детектором,
его точки просчитываются лучами датчика, с тенью. Первые FRAMES кадров побитно совпадают с оценкой;
--frames продлевает подъезд для видео.

В записи: облако в исходном формате и топике и /synthetic/object (std_msgs/String, JSON) --
истинная дистанция до передней грани объекта на каждом кадре. Узел детектора этот топик не читает.
"""
import argparse
import copy
import json
import os
import warnings
import zlib
from pathlib import Path

for _v in ('OMP_NUM_THREADS', 'OPENBLAS_NUM_THREADS', 'MKL_NUM_THREADS'):
    os.environ.setdefault(_v, '1')

import numpy as np

from tunnel_od import ObstacleDetector
from tunnel_od.pointcloud import beam_directions, xyz_views
from tunnel_od.sim.inject import inject, on_track
from tunnel_od.sim.shapes import make_shape

from bags import open_cloud_bag
from eval_approach import FRAMES, SHAPES, WARMUP, parse_pointcloud2, pick_windows
from eval_injection import front_of
from to_contract_v1 import to_humble_metadata

TRUTH_TOPIC = '/synthetic/object'


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--bag', required=True, help='запись-источник (имя, см. evaluation/bags.py)')
    ap.add_argument('--window', type=int, required=True, help='номер окна движения, как в approach_*.csv')
    ap.add_argument('--shape', default='человек стоит', choices=list(SHAPES))
    ap.add_argument('--start-m', type=float, required=True, help='дистанция до объекта в начале подъезда, м')
    ap.add_argument('--frames', type=int, default=FRAMES, help=f'кадров с объектом (в оценке -- {FRAMES})')
    ap.add_argument('--min-m', type=float, default=8.0, help='остановиться, когда объект ближе, м')
    ap.add_argument('--seed', type=int, default=0, help='как --seed в eval_approach')
    ap.add_argument('--out', required=True, help='каталог новой записи (не должен существовать)')
    args = ap.parse_args()
    warnings.simplefilter('ignore')

    from rosbags.interfaces import Qos, QosDurability, QosHistory, QosLiveliness, QosReliability, QosTime
    from rosbags.rosbag2 import Writer
    from rosbags.typesys import Stores, get_typestore

    starts = pick_windows(args.bag, 3, {})
    if args.window >= len(starts):
        raise SystemExit(f'{args.bag}: окон движения {len(starts)}, окна {args.window} нет')
    start = starts[args.window]
    kind, dims = SHAPES[args.shape]
    rng = np.random.default_rng(zlib.crc32(repr((args.seed, args.bag, args.window, args.shape,
                                                 int(args.start_m))).encode()))
    print(f'{args.bag}: окно {args.window} с кадра {start}, {args.shape} со {args.start_m:.0f} м')

    ts = get_typestore(Stores.ROS2_HUMBLE)
    String = ts.types['std_msgs/msg/String']
    inf = QosTime(sec=9223372036, nsec=854775807)
    qos = [Qos(QosHistory.KEEP_LAST, 10, QosReliability.RELIABLE, QosDurability.VOLATILE, inf, inf,
               QosLiveliness.AUTOMATIC, inf, False)]

    ego, ref = ObstacleDetector(ego_motion=True), ObstacleDetector()
    travelled, k, written = 0.0, 0, 0
    out = Path(args.out)
    with open_cloud_bag(args.bag) as (reader, conn), Writer(out, version=8) as w:
        c_cloud = w.add_connection(conn.topic, conn.msgtype, typestore=ts, offered_qos_profiles=qos)
        c_truth = w.add_connection(TRUTH_TOPIC, String.__msgtype__, typestore=ts, offered_qos_profiles=qos)
        t0 = reader.start_time
        for i, (c, t_bag, raw) in enumerate(reader.messages(connections=[conn])):
            if i < start:
                continue
            m = reader.deserialize(raw, c.msgtype)
            data, step, fields, t = bytes(m.data), m.point_step, m.fields, (t_bag - t0) / 1e9
            xyz = parse_pointcloud2(data, step, fields)
            if i < start + WARMUP:
                ego.detect(*xyz, refit_path=True, stamp=t)
                ref.detect(*xyz, refit_path=True, stamp=t)
                w.write(c_cloud, t_bag, raw)
                written += 1
                continue
            e = ego.detect(*xyz, refit_path=True, stamp=t)
            if k:
                travelled += e['displacement_m'] or 0.0
            ref.detect(*xyz, refit_path=True, stamp=t)
            d = args.start_m - travelled
            if d < args.min_m or k >= args.frames:
                break
            buf = np.frombuffer(data, np.uint8).reshape(-1, step)
            v = xyz_views(buf, fields)
            lat, z0 = on_track(ref.track_path(), d)
            new, info = inject(data, step, [make_shape(kind, dims, d, lat, z0)], fields,
                               dirs=beam_directions(v['x'], v['y'], v['z']), rng=rng)
            m.data = np.frombuffer(new, np.uint8)
            w.write(c_cloud, t_bag, ts.serialize_cdr(m, conn.msgtype))
            truth = {'shape': args.shape, 'distance_m': round(front_of(kind, dims, d), 2),
                     'lateral_m': round(float(lat), 2), 'points': int(info.get('rays_visible', 0)),
                     'frame': k}
            w.write(c_truth, t_bag, ts.serialize_cdr(String(data=json.dumps(truth, ensure_ascii=False)),
                                                    String.__msgtype__))
            k += 1
            written += 1
    to_humble_metadata(out)
    print(f'{written} кадров ({WARMUP} без объекта + {k} с объектом, {args.start_m:.0f} -> '
          f'{args.start_m - travelled:.0f} м) -> {out}')


if __name__ == '__main__':
    main()
