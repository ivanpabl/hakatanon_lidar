"""Где лежат записи и как читать из них облака (офлайн-инструменты, rosbags).

Данные -- в data/ (не в git): data/Датасет/archive/for_hackathon/<запись> и data/new_data.
Другой каталог данных (например, на отдельном диске под Windows) -- переменная TUNNEL_OD_DATA.
Результаты инструментов -- в runs/ (не в git), другой каталог -- TUNNEL_OD_RUNS.
"""
import os
from contextlib import contextmanager
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
DATA = Path(os.environ.get('TUNNEL_OD_DATA') or ROOT / 'data')
DATASET = DATA / 'Датасет' / 'archive' / 'for_hackathon'
RUNS = Path(os.environ.get('TUNNEL_OD_RUNS') or ROOT / 'runs')
BAGS = ['doubleT_obstacle', 'doubleT_platform', 'roundT_doubleT', 'roundT_pressureGate_roundT',
        'roundT_squareT_pressureGate_squareT', 'squareT_platform_squareT_switch', 'new_data']
EMPTY_BAGS = [b for b in BAGS if b not in ('doubleT_obstacle', 'new_data')]   # 5 записей без препятствий
FAKE_OBJ = 'cloud_with_fake_obj'           # бэг организаторов с 10 синтетическими объектами (#437)


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


def safe_messages(it, errors):
    """Сообщения из reader.messages(): ошибка чтения (битый архив, обрыв zstd) останавливает
    поток, текст ошибки -- в errors. Прочитанное до ошибки остаётся в работе."""
    try:
        yield from it
    except Exception as e:
        errors.append(f'{type(e).__name__}: {e}')


CROP = (2.0, 250.0)       # обрезка по дальности, как в tunnel_od_preproc (config/detector.yaml)


def cloud_parser(kind='python'):
    """Разбор облака (data, point_step, fields) -> (x, y, z).
    python -- parse_pointcloud2 (ядро); cpp -- библиотека tunnel_od_preproc с обрезкой по
    дальности, облако взято представлениями над одним буфером x, y, z -- ровно как в узле
    детектора при use_cpp_preproc:=true."""
    if kind == 'python':
        from tunnel_od import parse_pointcloud2
        return parse_pointcloud2
    if kind != 'cpp':
        raise ValueError(kind)
    import sys
    import numpy as np
    from tunnel_od.pointcloud import COLUMN_HEIGHT, dedupe_rounded
    sys.path.insert(0, str(ROOT / 'ros2_ws' / 'src' / 'tunnel_od_preproc'))
    from tunnel_od_preproc.build_host import ensure
    if ensure() is None:
        raise RuntimeError('не собрать libtunnel_od_canonical')
    from tunnel_od_preproc import native

    def parse(data, point_step, fields):
        x, y, z = native.parse_fast(data, point_step, fields, crop=CROP)
        if (len(data) // point_step) % (2 * COLUMN_HEIGHT):
            x, y, z = dedupe_rounded(x, y, z)          # C++ снимает дубли только в облаке из столбцов
        buf = np.empty((len(x), 3), np.float32)
        buf[:, 0], buf[:, 1], buf[:, 2] = x, y, z
        return buf[:, 0], buf[:, 1], buf[:, 2]
    return parse
