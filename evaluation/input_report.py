# © 2026 команда «Голуби», github.com/ivanpabl/hakatanon_lidar. Все права защищены, условия — в файле LICENSE. GLB-K5-4660c47a8c6c
"""Проверка входного потока записей на контракт v1 (docs/input_format.md) без ROS 2.

Тот же код, что в C++-узле tunnel_od_preproc (InputMonitor через ctypes): каждое облако
разбирается, а /tf_static, описание датчика и скорость поезда, если они есть в записи,
подаются в проверку как в узле.

    python -m evaluation input                              # все 7 записей
    python -m evaluation input --bags doubleT_platform contract_v1/doubleT_platform
    python -m evaluation input --max-frames 2000 --out output/runs/input_report.json
"""
import argparse
import json
import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / 'ros2_ws' / 'src' / 'tunnel_od_preproc'))
sys.path.insert(0, str(Path(__file__).resolve().parent))

from bags import BAGS, RUNS, bag_path, open_cloud_bag, safe_messages

CLOUD = 'sensor_msgs/msg/PointCloud2'
SPEED_TYPES = ('geometry_msgs/msg/TwistStamped', 'nav_msgs/msg/Odometry')
DESCRIPTION_TOPIC = '/lidar/description'


def _rpy_deg(q):
    r = math.atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x ** 2 + q.y ** 2))
    p = math.asin(max(-1.0, min(1.0, 2 * (q.w * q.y - q.z * q.x))))
    y = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y ** 2 + q.z ** 2))
    return [math.degrees(v) for v in (r, p, y)]


def monitor_bag(name, max_frames=None, progress=False):
    """Сводка проверки входа по записи (dict, как summary_json узла)."""
    from rosbags.highlevel import AnyReader
    from rosbags.typesys import Stores, get_typestore

    from tunnel_od_preproc.build_host import ensure
    ensure()
    from tunnel_od_preproc import native

    with AnyReader([bag_path(name)], default_typestore=get_typestore(Stores.ROS2_HUMBLE)) as reader:
        speed_conns = [c for c in reader.connections if c.msgtype in SPEED_TYPES]
        mon = native.Monitor(speed_expected=bool(speed_conns), description_expected=True)
        clouds = 0
        errors = []
        for c, t, raw in safe_messages(reader.messages(), errors):
            if c.msgtype == CLOUD:
                if max_frames and clouds >= max_frames:
                    break
                m = reader.deserialize(raw, c.msgtype)
                try:
                    native.parse_msg(m, monitor=mon, recv_s=t * 1e-9)
                except ValueError:
                    pass
                clouds += 1
                if progress and clouds % 500 == 0:
                    print(f'  {name}: {clouds} кадров', flush=True)
            elif c.topic == '/tf_static':
                for tr in reader.deserialize(raw, c.msgtype).transforms:
                    v = tr.transform.translation
                    mon.tf(tr.header.frame_id.lstrip('/'), tr.child_frame_id.lstrip('/'), (v.x, v.y, v.z),
                           _rpy_deg(tr.transform.rotation))
            elif c.topic == DESCRIPTION_TOPIC:
                mon.description(reader.deserialize(raw, c.msgtype).data)
            elif c in speed_conns:
                m = reader.deserialize(raw, c.msgtype)
                v = m.twist.linear.x if c.msgtype.endswith('TwistStamped') else m.twist.twist.linear.x
                mon.speed(m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, v)
        summary = mon.summary()
        summary['read_errors'] = errors
        return summary


def cloud_stats(name, max_frames=None):
    """Облако глазами ядра: point_step и поля, доля кадров из столбцов по 128 (кратно 256),
    доля дублей (ячейка 1 см), промежутки между кадрами по времени записи, побитовые повторы."""
    import numpy as np
    from tunnel_od.pointcloud import COLUMN_HEIGHT, FrameRepeat, dedupe_rounded, parse_pointcloud2
    steps, fields, ordered, dups, stamps, repeats, frames, errors = set(), None, 0, [], [], 0, 0, []
    rep = FrameRepeat()
    with open_cloud_bag(name) as (reader, conn):
        for c, t, raw in safe_messages(reader.messages(connections=[conn]), errors):
            if max_frames and frames >= max_frames:
                break
            m = reader.deserialize(raw, c.msgtype)
            frames += 1
            stamps.append(t * 1e-9)
            steps.add(int(m.point_step))
            fields = fields or [(f.name, int(f.offset), int(f.datatype)) for f in m.fields]
            ordered += (len(m.data) // m.point_step) % (2 * COLUMN_HEIGHT) == 0
            repeats += rep.check(m.data)
            if frames % 10 == 1:
                x, y, z = parse_pointcloud2(m.data, m.point_step, m.fields, dedupe_dual_return=False)
                if len(x):
                    dups.append(1.0 - len(dedupe_rounded(x, y, z)[0]) / len(x))
    gaps = np.diff(stamps)
    return {'frames_read': frames, 'read_errors': errors, 'point_step': sorted(steps), 'fields': fields,
            'ordered_share': ordered / max(frames, 1),
            'dup_share_median': float(np.median(dups)) if dups else None,
            'gap_s': {str(p): round(float(np.percentile(gaps, p)), 3) for p in (5, 50, 95, 99)} if len(gaps) else None,
            'gap_max_s': round(float(gaps.max()), 3) if len(gaps) else None,
            'gaps_over_0_15s': int((gaps > 0.15).sum()),
            'bitwise_repeats': repeats}


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--bags', nargs='+', default=BAGS)
    ap.add_argument('--max-frames', type=int, default=0, help='не больше стольких кадров на запись (0 -- все)')
    ap.add_argument('--no-native', action='store_true', help='только Python-сводка облака')
    ap.add_argument('--out', default=str(RUNS / 'input_report.json'))
    args = ap.parse_args()

    report = {}
    for bag in args.bags:
        s = {} if args.no_native else monitor_bag(bag, args.max_frames or None, progress=True)
        s['cloud'] = cloud_stats(bag, args.max_frames or None)
        report[bag] = s
        if s.get('checks'):
            print(f'\n== {bag}: {s["frames"]} кадров, нарушено: {", ".join(s["violations"]) or "ничего"}')
            for cid, ch in s['checks'].items():
                print(f'   {cid:<7}{ch["level"]:<8}{ch["message"]}')
        print(f'   облако: {json.dumps(s["cloud"], ensure_ascii=False)}')
    Path(args.out).parent.mkdir(parents=True, exist_ok=True)
    Path(args.out).write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding='utf-8')
    print(f'\nзаписано: {args.out}')


if __name__ == '__main__':
    main()
