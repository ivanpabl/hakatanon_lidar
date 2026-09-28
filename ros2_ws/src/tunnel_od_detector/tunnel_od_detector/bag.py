"""Что внутри записи ros2 bag: топик облака, frame_id, длительность, QoS записи. Нужно launch-файлам
play и demo (RViz должен знать frame_id облака, плеер -- когда запись кончится)."""
from pathlib import Path

import yaml

CLOUD_TYPE = 'sensor_msgs/msg/PointCloud2'
PREFERRED_TOPIC = '/lidar_points'
BEST_EFFORT = 2   # rmw_qos_reliability_policy_t в offered_qos_profiles


def pick_topic(counts, topic=''):
    """Топик облака из {имя: число сообщений} топиков PointCloud2 записи.
    topic задан -- он (нет такого -- ошибка со списком); один топик -- он, с любым именем;
    несколько -- /lidar_points, иначе самый длинный. Возвращает (топик, предупреждение или '')."""
    if not counts:
        raise RuntimeError(f'в записи нет топика {CLOUD_TYPE}')
    if topic:
        if topic not in counts:
            raise RuntimeError(f'топика {topic} нет среди {CLOUD_TYPE} записи: {", ".join(sorted(counts))}')
        return topic, ''
    if len(counts) == 1:
        return next(iter(counts)), ''
    best = PREFERRED_TOPIC if PREFERRED_TOPIC in counts else max(sorted(counts), key=lambda t: counts[t])
    return best, (f'в записи {len(counts)} топика {CLOUD_TYPE} ({", ".join(sorted(counts))}), взят {best}; '
                  f'другой -- topic:=<имя> или TOPIC=<имя>')


def _best_effort(qos):
    """offered_qos_profiles: YAML-строка списка профилей (metadata v4-v5) или сам список."""
    try:
        profiles = yaml.safe_load(qos) if isinstance(qos, str) else qos
    except yaml.YAMLError:
        return False
    return any(isinstance(p, dict) and p.get('reliability') in (BEST_EFFORT, 'best_effort')
               for p in profiles or [])


def bag_info(path, topic=''):
    """dict: name (имя записи), topic (PointCloud2, см. pick_topic), frame (frame_id первого кадра),
    duration (с), best_effort (записан с QoS best_effort), warning.
    Имя -- по файлу .db3 (new_data_0.db3 -> new_data): каталог в контейнере всегда /bag."""
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import PointCloud2

    meta = yaml.safe_load((Path(path) / 'metadata.yaml').read_text())['rosbag2_bagfile_information']
    duration = meta['duration']['nanoseconds'] / 1e9
    files = meta.get('relative_file_paths') or []
    name = Path(files[0]).stem.rsplit('_', 1)[0] if files else Path(str(path).rstrip('/')).name
    clouds = {t['topic_metadata']['name']: t for t in meta['topics_with_message_count']
              if t['topic_metadata']['type'] == CLOUD_TYPE}
    try:
        topic, warning = pick_topic({k: int(v.get('message_count') or 0) for k, v in clouds.items()}, topic)
    except RuntimeError as e:
        raise RuntimeError(f'{path}: {e}') from None
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(path), storage_id=meta.get('storage_identifier', 'sqlite3')),
                rosbag2_py.ConverterOptions('cdr', 'cdr'))
    reader.set_filter(rosbag2_py.StorageFilter(topics=[topic]))
    _, data, _ = reader.read_next()
    return {'name': name, 'topic': topic, 'frame': deserialize_message(data, PointCloud2).header.frame_id,
            'duration': duration, 'warning': warning,
            'best_effort': _best_effort(clouds[topic]['topic_metadata'].get('offered_qos_profiles'))}
