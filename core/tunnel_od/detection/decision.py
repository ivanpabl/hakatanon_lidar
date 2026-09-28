"""Решение по кадру поверх подтверждённых объектов: уровень каждого объекта, статус кадра
и докуда габарит проверен свободным. Чистые функции без состояния (состояние -- у трекера).

Уровень объекта (level, reason), первая подходящая строка:
    stop     in_gauge     подтверждён, в пределах оси, не едет с поездом
    caution  beyond_path  подтверждён, за концом известной оси
    caution  ego_carried  подтверждён, ЭГО-тест: едет вместе с поездом
    caution  pending      не подтверждён, трек >= 2 кадров и счёт >= pending_score
    None     None         остальное (мелкие одиночные, продольные конструкции у края)
Удержанный объект (held) наследует уровень трека из последнего кадра, где трек был виден.

Статус кадра: stop > unknown (нет оси -- габарит не проверить) > caution > clear.
"""
import math

import numpy as np

RANGE_CAP_M = 200.0
SIGHT_C_M = 1.5
MIN_CURVATURE = 1 / 5000


def object_level(obj, *, in_path, pending_score):
    if obj.get('held'):
        return obj.get('level', 'stop'), obj.get('reason', 'in_gauge')
    if obj.get('edge_line'):
        return None, None
    if obj.get('confirmed'):
        if in_path and not obj.get('ego_carried'):
            if obj.get('implausible'):
                return 'caution', 'surface_behind'
            return 'stop', 'in_gauge'
        return 'caution', ('ego_carried' if in_path else 'beyond_path')
    if obj.get('hits', 0) >= 2 and obj.get('evidence', 0.0) >= pending_score:
        return 'caution', 'pending'
    return None, None


def axis_curvature(fwd, cl, lo=40.0, hi=120.0):
    """Кривизна оси (1/R со знаком) по квадратичной аппроксимации cl(fwd) на [lo, hi]."""
    if fwd is None or cl is None:
        return 0.0
    fwd, cl = np.asarray(fwd, float), np.asarray(cl, float)
    m = (fwd >= lo) & (fwd <= hi)
    if m.sum() < 5 or fwd[m].max() - fwd[m].min() < 20.0:
        return 0.0
    return float(2.0 * np.polyfit(fwd[m], cl[m], 2)[0])


def sight_distance(path_range, curvature, c=SIGHT_C_M, cap=RANGE_CAP_M):
    """Докуда путь виден: ось, прямая видимость на кривой, паспортная дальность."""
    if path_range is None:
        return 0.0
    s = min(float(path_range), cap)
    if abs(curvature) >= MIN_CURVATURE:
        s = min(s, math.sqrt(8.0 * c / abs(curvature)))
    return s


def decide(objects, *, path_available, path_range, curvature, sight_c=SIGHT_C_M, range_cap=RANGE_CAP_M,
           pending_score=1.1):
    for o in objects:
        o['level'], o['reason'] = object_level(o, in_path=not o.get('beyond_path', False),
                                               pending_score=pending_score)
    stop = [o['distance_m'] for o in objects if o['level'] == 'stop']
    caution = [o['distance_m'] for o in objects if o['level'] == 'caution']
    if stop:
        status = 'stop'
    elif not path_available:
        status = 'unknown'
    elif caution:
        status = 'caution'
    else:
        status = 'clear'
    sight = sight_distance(path_range, curvature, sight_c, range_cap) if path_available else 0.0
    near = min(stop + caution, default=None)
    clear_to = 0.0 if not path_available else max(0.0, sight if near is None else min(sight, near))
    return {'status': status,
            'distance_m': min(stop) if stop else None,
            'caution_distance_m': min(caution) if caution else None,
            'sight_m': sight,
            'clear_to_m': clear_to}
