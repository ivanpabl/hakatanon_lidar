# © 2026 команда «Голуби», github.com/ivanpabl/hakatanon_lidar. Все права защищены, условия — в файле LICENSE. GLB-K5-4660c47a8c6c
"""Разбор PointCloud2 и проверка входного потока из Python через ctypes.

Тот же исходник, что в C++-узле preproc_node (src/canonical.cpp, src/monitor.cpp):
библиотека libtunnel_od_canonical. ctypes отпускает GIL на время вызова; вход
читается прямо из буфера сообщения, выход пишется в заранее выделенный массив.

    from tunnel_od_preproc import native
    if native.available():
        x, y, z = native.parse_msg(msg)                  # как parse_pointcloud2, но любой формат

Библиотека ищется: $TUNNEL_OD_PREPROC_LIB, затем <prefix пакета>/lib (ament),
затем рядом с этим файлом.
"""
import ctypes as C
import json
import os
import sys
from pathlib import Path

import numpy as np

FORMATS = {'auto': 0, 'legacy_hesai': 1, 'contract_v1': 2, 'generic': 3}
AXES = {'auto': 0, 'legacy': 1, 'rep103': 2}
LIB_NAME = 'libtunnel_od_canonical' + {'darwin': '.dylib', 'win32': '.dll'}.get(sys.platform, '.so')
API_VERSION = 1


class _Field(C.Structure):
    _fields_ = [('name', C.c_char_p), ('offset', C.c_uint32), ('datatype', C.c_uint8), ('count', C.c_uint32)]


class _Cloud(C.Structure):
    _fields_ = [('height', C.c_uint32), ('width', C.c_uint32), ('point_step', C.c_uint32),
                ('row_step', C.c_uint32), ('is_bigendian', C.c_int32), ('is_dense', C.c_int32),
                ('stamp_ns', C.c_int64), ('frame_id', C.c_char_p)]


class _Options(C.Structure):
    _fields_ = [('format', C.c_int32), ('axes', C.c_int32), ('dedupe', C.c_int32), ('crop_enabled', C.c_int32),
                ('collect_stats', C.c_int32), ('crop_min', C.c_double), ('crop_max', C.c_double)]


_F = C.POINTER(C.c_float)


def _candidates():
    env = os.environ.get('TUNNEL_OD_PREPROC_LIB')
    if env:
        yield Path(env)
    try:
        from ament_index_python.packages import get_package_prefix
        yield Path(get_package_prefix('tunnel_od_preproc')) / 'lib' / LIB_NAME
    except Exception:
        pass
    yield Path(__file__).resolve().parent / LIB_NAME


def _load():
    errors = []
    for p in _candidates():
        if not p.is_file():
            continue
        try:
            lib = C.CDLL(str(p))
        except OSError as e:
            errors.append(f'{p}: {e}')
            continue
        if lib.tod_api_version() != API_VERSION:
            errors.append(f'{p}: версия API {lib.tod_api_version()} != {API_VERSION}')
            continue
        lib.tod_parse.restype = C.c_int64
        lib.tod_parse.argtypes = [C.c_void_p, C.c_size_t, C.POINTER(_Cloud), C.POINTER(_Field), C.c_size_t,
                                  C.POINTER(_Options), _F, _F, _F, C.c_size_t, C.c_void_p, C.c_double,
                                  C.c_char_p, C.c_size_t, C.c_char_p, C.c_size_t]
        lib.tod_monitor_new.restype = C.c_void_p
        lib.tod_monitor_new.argtypes = [C.c_int32, C.c_int32]
        lib.tod_monitor_free.argtypes = [C.c_void_p]
        lib.tod_monitor_tf.argtypes = [C.c_void_p, C.c_char_p, C.c_char_p, C.POINTER(C.c_double),
                                       C.POINTER(C.c_double)]
        lib.tod_monitor_speed.argtypes = [C.c_void_p, C.c_double, C.c_double]
        lib.tod_monitor_description.argtypes = [C.c_void_p, C.c_char_p]
        for f in (lib.tod_monitor_summary, lib.tod_monitor_new_warnings):
            f.restype = C.c_size_t
            f.argtypes = [C.c_void_p, C.c_char_p, C.c_size_t]
        return lib, None
    return None, '; '.join(errors) or f'{LIB_NAME} не найдена'


_lib, load_error = _load()


def available():
    return _lib is not None


def _require():
    if _lib is None:
        raise RuntimeError(f'tunnel_od_preproc: нативная библиотека недоступна ({load_error})')
    return _lib


def _get_str(fn, handle):
    n = fn(handle, None, 0)
    buf = C.create_string_buffer(n)
    fn(handle, buf, n)
    return buf.value.decode('utf-8')


class Monitor:
    """Проверка входного потока на соответствие контракту v1 (как в узле)."""

    def __init__(self, speed_expected=False, description_expected=True):
        self._lib = _require()
        self.handle = self._lib.tod_monitor_new(int(speed_expected), int(description_expected))

    def __del__(self):
        if getattr(self, 'handle', None):
            self._lib.tod_monitor_free(self.handle)
            self.handle = None

    def tf(self, parent, child, xyz, rpy_deg):
        a = (C.c_double * 3)(*xyz)
        b = (C.c_double * 3)(*rpy_deg)
        self._lib.tod_monitor_tf(self.handle, parent.encode(), child.encode(), a, b)

    def speed(self, stamp_s, speed_mps):
        self._lib.tod_monitor_speed(self.handle, float(stamp_s), float(speed_mps))

    def description(self, text):
        self._lib.tod_monitor_description(self.handle, text.encode())

    def summary(self):
        return json.loads(_get_str(self._lib.tod_monitor_summary, self.handle))

    def new_warnings(self):
        return [w for w in _get_str(self._lib.tod_monitor_new_warnings, self.handle).split('\n') if w]


def parse_fast(data, point_step, fields, height=1, width=None, row_step=None, *, is_bigendian=False,
               is_dense=False, stamp_ns=0, frame_id='', fmt='auto', axes='auto', dedupe=True,
               crop=None, monitor=None, recv_s=0.0, with_stats=False):
    """Байты PointCloud2 -> (x, y, z) в осях ядра (float32, без копии входа).
    fields -- последовательность объектов с name/offset/datatype(/count).
    crop -- (fwd_min, fwd_max) или None. with_stats -- добавить словарь статистики кадра."""
    lib = _require()
    buf = np.frombuffer(data, dtype=np.uint8)
    if width is None:
        width = len(buf) // point_step // max(height, 1)
    if row_step is None:
        row_step = width * point_step
    n = height * width
    names = [f.name.encode() for f in fields]
    farr = (_Field * len(fields))(*[_Field(nm, f.offset, getattr(f, 'datatype', 7), getattr(f, 'count', 1) or 1)
                                    for nm, f in zip(names, fields)])
    cloud = _Cloud(height, width, point_step, row_step, int(is_bigendian), int(is_dense), int(stamp_ns),
                   frame_id.encode())
    opt = _Options(FORMATS[fmt], AXES[axes], int(dedupe), int(crop is not None), int(with_stats),
                   float(crop[0]) if crop else 0.0, float(crop[1]) if crop else 0.0)
    out = np.empty((3, max(n, 1)), np.float32)
    stats = C.create_string_buffer(2048) if with_stats else None
    err = C.create_string_buffer(512)
    k = lib.tod_parse(buf.ctypes.data if len(buf) else None, len(buf), C.byref(cloud), farr, len(fields),
                      C.byref(opt), out[0].ctypes.data_as(_F), out[1].ctypes.data_as(_F),
                      out[2].ctypes.data_as(_F), 1, monitor.handle if monitor is not None else None,
                      float(recv_s), stats, 2048 if with_stats else 0, err, 512)
    if k < 0:
        raise ValueError(err.value.decode('utf-8'))
    xyz = (out[0, :k], out[1, :k], out[2, :k])
    return xyz + (json.loads(stats.value.decode('utf-8')),) if with_stats else xyz


def parse_msg(msg, **kw):
    """То же для sensor_msgs/PointCloud2 (rclpy или rosbags)."""
    st = msg.header.stamp
    return parse_fast(msg.data, msg.point_step, msg.fields, msg.height, msg.width, msg.row_step,
                      is_bigendian=bool(msg.is_bigendian), is_dense=bool(msg.is_dense),
                      stamp_ns=int(st.sec) * 1_000_000_000 + int(st.nanosec), frame_id=msg.header.frame_id, **kw)
