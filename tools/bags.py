"""Где лежат записи и как читать из них облака (офлайн-инструменты, rosbags).

Данные -- в data/ (не в git): data/Датасет/archive/for_hackathon/<запись> и data/new_data.
Результаты инструментов -- в runs/ (не в git).
"""
from contextlib import contextmanager
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
DATA = ROOT / 'data'
DATASET = DATA / 'Датасет' / 'archive' / 'for_hackathon'
RUNS = ROOT / 'runs'
BAGS = ['doubleT_obstacle', 'doubleT_platform', 'roundT_doubleT', 'roundT_pressureGate_roundT',
        'roundT_squareT_pressureGate_squareT', 'squareT_platform_squareT_switch', 'new_data']


def bag_path(name):
    """Запись прямо в data/ (new_data, cloud_with_fake_obj, ...) или из data/Датасет/archive/for_hackathon."""
    return DATA / name if (DATA / name).is_dir() else DATASET / name


@contextmanager
def open_cloud_bag(name):
    """(reader, connection) топика PointCloud2 записи. Топик ищется по типу, а не по
    имени: в записях он называется по-разному (/lidar_points, /sensing/lidar/...)."""
    from rosbags.highlevel import AnyReader
    from rosbags.typesys import Stores, get_typestore
    with AnyReader([bag_path(name)], default_typestore=get_typestore(Stores.ROS2_HUMBLE)) as reader:
        conn = [c for c in reader.connections if c.msgtype == 'sensor_msgs/msg/PointCloud2'][0]
        yield reader, conn
