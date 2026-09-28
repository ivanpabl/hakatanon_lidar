"""Где лежат записи и как читать из них облака (офлайн-инструменты, rosbags).

Данные -- в data/ (не в git): data/Датасет/archive/for_hackathon/<запись> и data/new_data.
Другой каталог данных (например, на отдельном диске под Windows) -- переменная TUNNEL_OD_DATA.
Результаты инструментов -- в output/runs (не в git), другой каталог -- TUNNEL_OD_RUNS.
"""
import os
from contextlib import contextmanager
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
DATA = Path(os.environ.get('TUNNEL_OD_DATA') or ROOT / 'data')
DATASET = DATA / 'Датасет' / 'archive' / 'for_hackathon'
RUNS = Path(os.environ.get('TUNNEL_OD_RUNS') or ROOT / 'output' / 'runs')
BAGS = ['doubleT_obstacle', 'doubleT_platform', 'roundT_doubleT', 'roundT_pressureGate_roundT',
        'roundT_squareT_pressureGate_squareT', 'squareT_platform_squareT_switch', 'new_data']
EMPTY_BAGS = [b for b in BAGS if b not in ('doubleT_obstacle', 'new_data')]
FAKE_OBJ = 'cloud_with_fake_obj'
FULL_FRAMES = {FAKE_OBJ: 1510, 'new_data': 11271}


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


CROP = (2.0, 250.0)


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
    from tunnel_od.pointcloud import dedupe_rounded, hesai_columns
    sys.path.insert(0, str(ROOT / 'ros2_ws' / 'src' / 'tunnel_od_preproc'))
    from tunnel_od_preproc.build_host import ensure
    if ensure() is None:
        raise RuntimeError('не собрать libtunnel_od_canonical')
    from tunnel_od_preproc import native

    def parse(data, point_step, fields):
        x, y, z = native.parse_fast(data, point_step, fields, crop=CROP)
        if not hesai_columns(len(data) // point_step, fields):
            x, y, z = dedupe_rounded(x, y, z)
        buf = np.empty((len(x), 3), np.float32)
        buf[:, 0], buf[:, 1], buf[:, 2] = x, y, z
        return buf[:, 0], buf[:, 1], buf[:, 2]
    return parse


def read_problems(tag, runs=None):
    """Недочитанные записи прогона alarms.py --tag tag: ['<запись>: прочитано N из M кадров (ошибка)'].
    Сведения -- runs/alarms_<tag>_read.json; у старых прогонов без него -- число кадров по CSV."""
    import csv
    import json
    runs = RUNS if runs is None else Path(runs)
    meta = runs / f'alarms_{tag}_read.json'
    if meta.exists():
        info = json.loads(meta.read_text(encoding='utf-8'))
        segments, bags = info.get('segments', 0), info['bags']
    else:
        path = runs / f'alarms_{tag}.csv'
        if not path.exists():
            return [f'alarms_{tag}: нет сведений о чтении']
        segments, bags = 0, {}
        for r in csv.DictReader(open(path, encoding='utf-8')):
            bags.setdefault(r['bag'], {'frames': 0, 'error': None})['frames'] += 1
    out = []
    for bag, b in bags.items():
        full = FULL_FRAMES.get(bag)
        short = not segments and full is not None and b['frames'] < full
        if b.get('error') or short:
            out.append(f'{bag}: прочитано {b["frames"]}' + (f' из {full}' if full else '') + ' кадров'
                       + (f' ({b["error"]})' if b.get('error') else ''))
    return out
