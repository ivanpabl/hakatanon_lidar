"""Что внутри записи ros2 bag: топик облака, frame_id, длительность. Нужно launch-файлам play и demo
(RViz должен знать frame_id облака, плеер -- когда запись кончится)."""
from pathlib import Path

import yaml

CLOUD_TYPE = 'sensor_msgs/msg/PointCloud2'


def bag_info(path):
    """(имя записи, топик PointCloud2, frame_id первого кадра, длительность в секундах).
    Имя -- по файлу .db3 (new_data_0.db3 -> new_data): каталог в контейнере всегда /bag."""
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import PointCloud2

    meta = yaml.safe_load((Path(path) / 'metadata.yaml').read_text())['rosbag2_bagfile_information']
    duration = meta['duration']['nanoseconds'] / 1e9
    files = meta.get('relative_file_paths') or []
    name = Path(files[0]).stem.rsplit('_', 1)[0] if files else Path(str(path).rstrip('/')).name
    topics = [t['topic_metadata']['name'] for t in meta['topics_with_message_count']
              if t['topic_metadata']['type'] == CLOUD_TYPE]
    if not topics:
        raise RuntimeError(f'{path}: в записи нет топика {CLOUD_TYPE}')
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(path), storage_id=meta.get('storage_identifier', 'sqlite3')),
                rosbag2_py.ConverterOptions('cdr', 'cdr'))
    reader.set_filter(rosbag2_py.StorageFilter(topics=[topics[0]]))
    _, data, _ = reader.read_next()
    return name, topics[0], deserialize_message(data, PointCloud2).header.frame_id, duration
