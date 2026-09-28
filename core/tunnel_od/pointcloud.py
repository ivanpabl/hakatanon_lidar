"""Сырой PointCloud2 лидара: разбор в x, y, z и геометрия лучей датчика.

Layout в записях: x,y,z,intensity (float32), ring (uint16), timestamp (float64)
-> point_step=26 байт. Если передать fields (msg.fields), смещения берутся из них.
Работает и с сообщением из rosbags (offline), и с sensor_msgs/msg/PointCloud2 из rclpy.

Устройство облака (проверено на всех 7 записях):
- облако идёт столбцами по 128 каналов; пара столбцов (2k, 2k+1) -- dual return
  одного залпа: в 97-98% пар точки совпадают;
- 38-62% точек -- нули (0,0,0): луч без отражения;
- азимут пары столбцов = A0 - 0.1 град * k, у каждого канала постоянные угол
  места и сдвиг азимута (до +-7.8 град).

Система координат: x -- вбок, вперёд = -y (не +y!), z -- вверх. Во всём пакете
продольная координата fwd = -y, поперечная lat = x.

Синтетика организаторов (cloud_with_fake_obj) -- 16-байтные точки x, y, z, intensity, без поля
timestamp; это не Hesai-облако из столбцов, даже если число точек кратно 256 (замер на
cloud_with_fake_obj, кадр 2: 307200 точек, из них парный дедуп -- 117781, округление -- 89343;
объекты синтетики не dual-return пары, дедуп по столбцам их не снимает и оставляет мусор). Дубли
снимаются запасным способом (hesai_columns=False): одна точка на ячейку 1 см.
"""
import numpy as np

COLUMN_HEIGHT = 128
DUAL_RETURN_DUP_M = 0.01
AZ_STEP_DEG = 0.1
DEDUPE_CELL_M = 0.01

_DEFAULT_OFFSETS = {'x': 0, 'y': 4, 'z': 8, 'intensity': 12}


def xyz_views(buf, fields=None):
    """Изменяемые float32-представления полей x, y, z, intensity над буфером (N, point_step)."""
    offs = dict(_DEFAULT_OFFSETS)
    if fields is not None:
        offs.update({f.name: f.offset for f in fields if f.name in offs})
    return {k: buf[:, o:o + 4].view(np.float32)[:, 0] for k, o in offs.items()}


def hesai_columns(n, fields) -> bool:
    """Облако -- пары столбцов Hesai dual return: точек кратно 2*COLUMN_HEIGHT и есть поле
    timestamp (fields=None -- запись без явных полей, это старый 26-байтный layout по
    умолчанию, тоже Hesai). Иначе (в т.ч. синтетика организаторов) -- не Hesai, даже если
    число точек случайно делится на 256: дедуп по столбцам тогда снимает не то."""
    if n % (2 * COLUMN_HEIGHT) != 0:
        return False
    if fields is None:
        return True
    return any(f.name == 'timestamp' for f in fields)


def parse_pointcloud2(data: bytes, point_step: int, fields=None, dedupe_dual_return=True):
    """Байты PointCloud2 -> (x, y, z) без нулевых точек и дублей dual return.
    Без очистки дальше обрабатывалось бы в 3-5 раз больше точек, чем есть на самом деле."""
    buf = np.frombuffer(data, dtype=np.uint8).reshape(-1, point_step)
    offs = {'x': 0, 'y': 4, 'z': 8}
    if fields is not None:
        offs.update({f.name: f.offset for f in fields if f.name in offs})
    x, y, z = (buf[:, o:o + 4].view(np.float32).ravel() for o in (offs['x'], offs['y'], offs['z']))
    valid = np.isfinite(x) & np.isfinite(y) & np.isfinite(z) & ((x != 0) | (y != 0) | (z != 0))

    n = len(x)
    if dedupe_dual_return and hesai_columns(n, fields):
        cols = lambda a: a.reshape(-1, 2, COLUMN_HEIGHT)
        xa, ya, za = cols(x), cols(y), cols(z)
        dup = ((np.abs(xa[:, 1] - xa[:, 0]) < DUAL_RETURN_DUP_M)
               & (np.abs(ya[:, 1] - ya[:, 0]) < DUAL_RETURN_DUP_M)
               & (np.abs(za[:, 1] - za[:, 0]) < DUAL_RETURN_DUP_M))
        valid.reshape(-1, 2, COLUMN_HEIGHT)[:, 1] &= ~dup
        return x[valid], y[valid], z[valid]
    x, y, z = x[valid], y[valid], z[valid]
    if dedupe_dual_return:
        return dedupe_rounded(x, y, z)
    return x, y, z


def dedupe_rounded(x, y, z, cell=DEDUPE_CELL_M):
    """Дубли облака не из столбцов: точки, совпавшие после округления до cell, -- одна точка.
    Порядок оставшихся точек -- как во входе (первое вхождение)."""
    if len(x) == 0:
        return x, y, z
    q = lambda a: (np.round(np.asarray(a, np.float64) / cell).astype(np.int64) + 32768) & 0xFFFF
    key = (q(x) << 32) | (q(y) << 16) | q(z)          # +-327 м по каждой оси в 16 битах
    _, first = np.unique(key, return_index=True)
    keep = np.sort(first)
    return x[keep], y[keep], z[keep]


class FrameRepeat:
    """Облако, побитово совпадающее с предыдущим: в бэге организаторов кадры повторяются
    («объекты замирают»), и повтор нельзя подавать в детектор как новый кадр -- трекер и
    оценка скорости посчитают его кадром с нулевым движением."""

    def __init__(self):
        self._prev = None

    def check(self, data) -> bool:
        cur = np.frombuffer(data, np.uint8)
        same = self._prev is not None and len(cur) == len(self._prev) and np.array_equal(cur, self._prev)
        if not same:
            self._prev = cur.copy()
        return same


def beam_directions(x, y, z):
    """Единичные направления всех лучей упорядоченного облака, (N, 3) в (lat, fwd, z).
    Строятся и для лучей без отражения (нулевых точек). Ошибка < 0.001 град."""
    n = len(x)
    if n % (2 * COLUMN_HEIGHT):
        raise ValueError(f'облако не из столбцов по {COLUMN_HEIGHT} каналов парами: {n} точек')
    ncol = n // COLUMN_HEIGHT
    X, Y, Z = (np.asarray(a, np.float64).reshape(ncol, COLUMN_HEIGHT) for a in (x, y, z))
    valid = (X != 0) | (Y != 0) | (Z != 0)
    if valid.sum(axis=0).min() == 0:
        raise ValueError('в кадре есть каналы без единого отражения -- углы канала не восстановить')
    R = np.sqrt(X ** 2 + Y ** 2 + Z ** 2)
    az = np.where(valid, np.degrees(np.arctan2(X, -Y)), np.nan)
    el = np.where(valid, np.degrees(np.arcsin(np.clip(Z / np.maximum(R, 1e-9), -1, 1))), np.nan)

    wrap = lambda a: (a + 180) % 360 - 180
    col_med = np.nanmedian(az, axis=1)
    off = np.nanmedian(wrap(az - col_med[:, None]), axis=0)
    el_ring = np.nanmedian(el, axis=0)
    base = np.nanmedian(wrap(az - off[None, :]), axis=1)
    pair = np.arange(ncol) // 2
    ok = np.isfinite(base)
    a0 = np.median(base[ok] + AZ_STEP_DEG * pair[ok])
    az_all = np.radians(wrap(a0 - AZ_STEP_DEG * pair[:, None] + off[None, :]))
    el_all = np.radians(np.broadcast_to(el_ring, (ncol, COLUMN_HEIGHT)))
    d = np.stack([np.cos(el_all) * np.sin(az_all), np.cos(el_all) * np.cos(az_all), np.sin(el_all)], axis=-1)
    return d.reshape(-1, 3)
