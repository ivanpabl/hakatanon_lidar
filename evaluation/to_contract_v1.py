"""Кадр записи (legacy Hesai) -> облако в контракте v1 (docs/input_format.md) и короткая
запись ros2 bag в контракте v1 для e2e-прогона.

Контракт v1: упорядоченное 128 x W (строка = ring), пустой луч -- NaN, оси REP-103
(x вперёд, y влево, z вверх), return_id (второе отражение отдельным столбцом, NaN там,
где совпадает с первым), t_offset_ns от header.stamp, point_step 24, поля перечислены
не в порядке смещений (адаптер обязан брать смещения из fields).

Кадр строится так, чтобы адаптер выдал из него то же каноническое облако, что
parse_pointcloud2 из исходного кадра: пустые лучи и дубли второго отражения
(правило parse_pointcloud2: < 1 см по x, y, z в float32) -> NaN; x_rep = -y, y_rep = x.

    python -m evaluation contract --bag doubleT_platform --out data/contract_v1_doubleT_platform
    python -m evaluation contract --bag doubleT_obstacle --frames 60 --speed --out ...

В записи: /lidar/points (облако), /tf_static (base_link -> lidar), /lidar/description
(M6, латченый JSON), с --speed -- /train/twist (S1). Скорость в --speed СИНТЕТИЧЕСКАЯ:
оценка EgoMotion ядра по этим же кадрам, только чтобы проверить проводку S1.
Высота в /tf_static -- оценка по кадру (головка рельса на 5 м), а не калибровка.
"""
import argparse
import json
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))

COLUMN_HEIGHT = 128
DUP_M = 0.01
TOPIC = '/lidar/points'
FRAME = 'lidar'
DESCRIPTION_TOPIC = '/lidar/description'
SPEED_TOPIC = '/train/twist'
BASE = 'base_link'

U8, U16, U32, F32, F64 = 2, 4, 6, 7, 8
LAYOUT = np.dtype({'names': ['x', 'y', 'z', 'intensity', 'return_id', 'ring', 't_offset_ns'],
                   'formats': ['<f4', '<f4', '<f4', 'u1', 'u1', '<u2', '<u4'],
                   'offsets': [0, 4, 8, 12, 13, 14, 16], 'itemsize': 24})
DATATYPE = {'x': F32, 'y': F32, 'z': F32, 'intensity': U8, 'return_id': U8, 'ring': U16, 't_offset_ns': U32}
FIELD_ORDER = ['t_offset_ns', 'ring', 'return_id', 'intensity', 'z', 'y', 'x']
_NP = {F32: '<f4', F64: '<f8', U16: '<u2', U8: 'u1', U32: '<u4', 3: '<i2', 5: '<i4', 1: 'i1'}


def legacy_fields(data, point_step, fields):
    """Поля кадра записи как структурированный массив (смещения -- из fields)."""
    dt = np.dtype({'names': [f.name for f in fields], 'formats': [_NP[f.datatype] for f in fields],
                   'offsets': [f.offset for f in fields], 'itemsize': point_step})
    return np.frombuffer(data, dtype=dt)


def legacy_to_contract(data, point_step, fields, stamp_ns):
    """Байты legacy-кадра -> (байты, height, width, point_step, fields[(name, offset, datatype)]).
    stamp_ns -- исходный header.stamp (время точки отсчитывается от него)."""
    a = legacy_fields(data, point_step, fields)
    n = len(a)
    if n % (2 * COLUMN_HEIGHT):
        raise ValueError(f'{n} точек: не целое число пар столбцов по {COLUMN_HEIGHT}')
    x, y, z = (np.ascontiguousarray(a[k]) for k in 'xyz')
    valid = np.isfinite(x) & np.isfinite(y) & np.isfinite(z) & ((x != 0) | (y != 0) | (z != 0))
    cols = lambda v: v.reshape(-1, 2, COLUMN_HEIGHT)
    xa, ya, za = cols(x), cols(y), cols(z)
    dup = ((np.abs(xa[:, 1] - xa[:, 0]) < DUP_M) & (np.abs(ya[:, 1] - ya[:, 0]) < DUP_M)
           & (np.abs(za[:, 1] - za[:, 0]) < DUP_M))
    valid.reshape(-1, 2, COLUMN_HEIGHT)[:, 1] &= ~dup

    width = n // COLUMN_HEIGHT
    out = np.zeros(n, LAYOUT)
    nan = np.float32(np.nan)
    out['x'] = np.where(valid, -y, nan)
    out['y'] = np.where(valid, x, nan)
    out['z'] = np.where(valid, z, nan)
    if 'intensity' in a.dtype.names:
        out['intensity'] = np.clip(np.rint(a['intensity']), 0, 255).astype(np.uint8)
    idx = np.arange(n)
    out['return_id'] = (idx // COLUMN_HEIGHT) % 2
    out['ring'] = idx % COLUMN_HEIGHT
    if 'timestamp' in a.dtype.names:
        t = (a['timestamp'] - stamp_ns // 1_000_000_000) * 1e9 - stamp_ns % 1_000_000_000
        out['t_offset_ns'] = np.clip(np.rint(t), 0, 2 ** 32 - 1).astype(np.uint32)
    ordered = out.reshape(width, COLUMN_HEIGHT).T.copy()
    flds = [(name, LAYOUT.fields[name][1], DATATYPE[name]) for name in FIELD_ORDER]
    return ordered.tobytes(), COLUMN_HEIGHT, width, LAYOUT.itemsize, flds


def description(frame):
    """M6: описание датчика (JSON). Таблица каналов -- из tunnel_od/sim/ring_table.json."""
    import tunnel_od.sim as sim
    table = json.loads((Path(sim.__file__).parent / 'ring_table.json').read_text())
    rings = [{'ring': int(k), 'elevation_deg': float(v)} for k, v in sorted(table.items(), key=lambda kv: int(kv[0]))]
    return json.dumps({'model': 'Hesai Pandar128E3X', 'firmware': 'unknown (converted from legacy recording)',
                       'rpm': 600, 'return_mode': 'dual', 'frame_id': frame, 'azimuth_step_deg': 0.1,
                       'channels': rings}, ensure_ascii=False)


def main():
    from rosbags.interfaces import Qos, QosDurability, QosHistory, QosLiveliness, QosReliability, QosTime
    from rosbags.rosbag2 import Writer
    from rosbags.typesys import Stores, get_typestore

    from bags import open_cloud_bag

    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--bag', required=True, help='запись-источник (имя, см. evaluation/bags.py)')
    ap.add_argument('--out', required=True, help='каталог новой записи (не должен существовать)')
    ap.add_argument('--start', type=int, default=0, help='с какого кадра')
    ap.add_argument('--frames', type=int, default=0, help='сколько кадров (0 -- до конца)')
    ap.add_argument('--speed', action='store_true', help='добавить /train/twist (синтетика из EgoMotion)')
    args = ap.parse_args()

    ts = get_typestore(Stores.ROS2_HUMBLE)
    T = ts.types
    PointCloud2, PointField = T['sensor_msgs/msg/PointCloud2'], T['sensor_msgs/msg/PointField']
    Header, Time = T['std_msgs/msg/Header'], T['builtin_interfaces/msg/Time']
    String, TFMessage = T['std_msgs/msg/String'], T['tf2_msgs/msg/TFMessage']
    TransformStamped, Transform = T['geometry_msgs/msg/TransformStamped'], T['geometry_msgs/msg/Transform']
    Vector3, Quaternion = T['geometry_msgs/msg/Vector3'], T['geometry_msgs/msg/Quaternion']
    TwistStamped, Twist = T['geometry_msgs/msg/TwistStamped'], T['geometry_msgs/msg/Twist']

    inf = QosTime(sec=9223372036, nsec=854775807)
    qos = lambda durability, depth: [Qos(QosHistory.KEEP_LAST, depth, QosReliability.RELIABLE, durability, inf, inf,
                                         QosLiveliness.AUTOMATIC, inf, False)]
    stamp_of = lambda ns: Time(sec=int(ns // 1_000_000_000), nanosec=int(ns % 1_000_000_000))

    det = None
    if args.speed:
        import warnings
        warnings.simplefilter('ignore')
        from tunnel_od import ObstacleDetector, parse_pointcloud2
        det = ObstacleDetector()

    out = Path(args.out)
    written = 0
    with open_cloud_bag(args.bag) as (reader, conn), Writer(out, version=8) as w:
        c_cloud = w.add_connection(TOPIC, PointCloud2.__msgtype__, typestore=ts,
                                   offered_qos_profiles=qos(QosDurability.VOLATILE, 10))
        c_tf = w.add_connection('/tf_static', TFMessage.__msgtype__, typestore=ts,
                                offered_qos_profiles=qos(QosDurability.TRANSIENT_LOCAL, 1))
        c_desc = w.add_connection(DESCRIPTION_TOPIC, String.__msgtype__, typestore=ts,
                                  offered_qos_profiles=qos(QosDurability.TRANSIENT_LOCAL, 1))
        c_speed = (w.add_connection(SPEED_TOPIC, TwistStamped.__msgtype__, typestore=ts,
                                    offered_qos_profiles=qos(QosDurability.VOLATILE, 10)) if args.speed else None)
        shift = None
        for i, (c, t_bag, raw) in enumerate(reader.messages(connections=[conn])):
            if i < args.start:
                continue
            if args.frames and written >= args.frames:
                break
            m = reader.deserialize(raw, c.msgtype)
            stamp = int(m.header.stamp.sec) * 1_000_000_000 + int(m.header.stamp.nanosec)
            if shift is None:
                shift = t_bag - stamp
                w.write(c_desc, t_bag, ts.serialize_cdr(String(data=description(FRAME)), String.__msgtype__))
                height = 1.5
                if det is None:
                    from tunnel_od import ObstacleDetector, parse_pointcloud2
                    probe = ObstacleDetector()
                else:
                    probe = det
                xyz = parse_pointcloud2(m.data, m.point_step, m.fields)
                probe.detect(*xyz, refit_path=True, stamp=stamp / 1e9)
                tor = probe.track_path().rail_top_at(5.0)
                if np.isfinite(tor[0]):
                    height = float(-tor[0])
                tf = TFMessage(transforms=[TransformStamped(
                    header=Header(stamp=stamp_of(t_bag), frame_id=BASE), child_frame_id=FRAME,
                    transform=Transform(translation=Vector3(x=0.0, y=0.0, z=height),
                                        rotation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)))])
                w.write(c_tf, t_bag, ts.serialize_cdr(tf, TFMessage.__msgtype__))
            data, h, wd, ps, flds = legacy_to_contract(m.data, m.point_step, m.fields, stamp)
            msg = PointCloud2(
                header=Header(stamp=stamp_of(stamp + shift), frame_id=FRAME), height=h, width=wd,
                fields=[PointField(name=nm, offset=off, datatype=dt, count=1) for nm, off, dt in flds],
                is_bigendian=False, point_step=ps, row_step=ps * wd, data=np.frombuffer(data, np.uint8),
                is_dense=False)
            w.write(c_cloud, t_bag, ts.serialize_cdr(msg, PointCloud2.__msgtype__))
            if det is not None:
                if written:
                    det.detect(*parse_pointcloud2(m.data, m.point_step, m.fields), refit_path=True, stamp=stamp / 1e9)
                if det.speed is not None:
                    tw = TwistStamped(header=Header(stamp=stamp_of(stamp + shift), frame_id=BASE),
                                      twist=Twist(linear=Vector3(x=float(det.speed), y=0.0, z=0.0),
                                                  angular=Vector3(x=0.0, y=0.0, z=0.0)))
                    w.write(c_speed, t_bag + 1, ts.serialize_cdr(tw, TwistStamped.__msgtype__))
            written += 1
            if written % 50 == 0:
                print(f'  {written} кадров', flush=True)
    to_humble_metadata(out)
    print(f'{args.bag}: {written} кадров -> {out} (топик {TOPIC}, frame_id {FRAME})')


def to_humble_metadata(out):
    """rosbags пишет metadata.yaml версии 8; ros2 bag в Humble ждёт версию 5 -- переписываем."""
    from ruamel.yaml import YAML
    yaml = YAML(typ='safe')
    path = Path(out) / 'metadata.yaml'
    info = yaml.load(path.read_text())['rosbag2_bagfile_information']
    topics = [{'topic_metadata': {k: t['topic_metadata'][k] for k in
                                  ('name', 'type', 'serialization_format', 'offered_qos_profiles')},
               'message_count': t['message_count']} for t in info['topics_with_message_count']]
    files = [{k: f[k] for k in ('path', 'starting_time', 'duration', 'message_count')} for f in info['files']]
    v5 = {'rosbag2_bagfile_information': {
        'version': 5, 'storage_identifier': info['storage_identifier'], 'duration': info['duration'],
        'starting_time': info['starting_time'], 'message_count': info['message_count'],
        'topics_with_message_count': topics, 'compression_format': '', 'compression_mode': '',
        'relative_file_paths': info['relative_file_paths'], 'files': files}}
    yaml.default_flow_style = False
    with path.open('w') as fh:
        yaml.dump(v5, fh)


if __name__ == '__main__':
    main()
