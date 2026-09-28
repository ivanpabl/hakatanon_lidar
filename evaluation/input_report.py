"""Проверка входного потока записей на контракт v1 (docs/INPUT_FORMAT.md) без ROS 2.

Тот же код, что в C++-узле tunnel_od_preproc (InputMonitor через ctypes): каждое облако
разбирается, а /tf_static, описание датчика и скорость поезда, если они есть в записи,
подаются в проверку как в узле.

    python -m evaluation input                              # все 7 записей
    python -m evaluation input --bags doubleT_platform contract_v1/doubleT_platform
    python -m evaluation input --max-frames 2000 --out runs/input_report.json
"""
import argparse
import json
import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / 'ros2_ws' / 'src' / 'tunnel_od_preproc'))
sys.path.insert(0, str(Path(__file__).resolve().parent))

from bags import BAGS, RUNS, bag_path

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
        for c, t, raw in reader.messages():
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
        return mon.summary()


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--bags', nargs='+', default=BAGS)
    ap.add_argument('--max-frames', type=int, default=0, help='не больше стольких кадров на запись (0 -- все)')
    ap.add_argument('--out', default=str(RUNS / 'input_report.json'))
    args = ap.parse_args()

    report = {}
    for bag in args.bags:
        s = monitor_bag(bag, args.max_frames or None, progress=True)
        report[bag] = s
        print(f'\n== {bag}: {s["frames"]} кадров, нарушено: {", ".join(s["violations"]) or "ничего"}')
        for cid, ch in s['checks'].items():
            print(f'   {cid:<7}{ch["level"]:<8}{ch["message"]}')
    Path(args.out).parent.mkdir(parents=True, exist_ok=True)
    Path(args.out).write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding='utf-8')
    print(f'\nзаписано: {args.out}')


if __name__ == '__main__':
    main()
