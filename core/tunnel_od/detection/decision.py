# © 2026 команда «Голуби», github.com/ivanpabl/hakatanon_lidar. Все права защищены, условия — в файле LICENSE. GLB-K5-4660c47a8c6c
"""Решение по кадру поверх подтверждённых объектов: уровень каждого объекта, статус кадра
и докуда габарит проверен свободным. Чистые функции без состояния (состояние -- у трекера).

Уровень объекта (level, reason), первая подходящая строка:
    stop     in_gauge     подтверждён, в пределах оси, не едет с поездом
    caution  surface_behind  подтверждён вдали, но за ним на его высоте идут точки (plausibility)
    caution  front_lift   [front_lift_m] перед объектом полотно/точки выше его низа на front_lift_m
                          и низ < front_lift_max_low_m: поднятое полотно, скан-линия, клаттер
    caution  far_unconfirmed  [stop_confirm_far_m] СТОП на d >= stop_confirm_far_m, но трек не был
                          СТОП в прошлом кадре (decide: prev_stop_ids); в следующем кадре -- stop
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


def object_level(obj, *, in_path, pending_score, front_lift_m=None, front_lift_max_low_m=2.0):
    if obj.get('held'):
        return obj.get('level', 'stop'), obj.get('reason', 'in_gauge')
    if obj.get('edge_line'):
        return None, None
    if obj.get('confirmed'):
        if in_path and not obj.get('ego_carried'):
            if obj.get('implausible'):
                return 'caution', 'surface_behind'
            if front_lifted(obj, front_lift_m, front_lift_max_low_m):
                return 'caution', 'front_lift'
            return 'stop', 'in_gauge'
        return 'caution', ('ego_carried' if in_path else 'beyond_path')
    if obj.get('hits', 0) >= 2 and obj.get('evidence', 0.0) >= pending_score:
        return 'caution', 'pending'
    return None, None


def front_lifted(obj, lift_m, max_low_m=2.0):
    """R2 (fp_autopsy): front_maxh >= low_m + lift_m и low_m < max_low_m. front_maxh -- максимум высоты
    точек в полосе объекта перед ним (plausibility.object_features): перед настоящим предметом
    поверхность не поднимается выше его низа; перед поднятым полотном / скан-линией -- поднимается.
    lift_m=None -- правило выключено; без признака (нет front_maxh) -- не срабатывает."""
    if lift_m is None:
        return False
    fm, low = obj.get('front_maxh'), obj.get('low_m')
    return fm is not None and low is not None and low < max_low_m and fm >= low + lift_m


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
           pending_score=1.1, min_sight_m=0.0, front_lift_m=None, front_lift_max_low_m=2.0,
           stop_confirm_far_m=None, prev_stop_ids=()):
    """min_sight_m -- путь виден ближе этого (ось оборвалась у лидара): габарит не проверен, статус
    unknown, как без оси. front_lift_m -- R2 (object_level). stop_confirm_far_m -- R3: объект уровня stop
    на d >= stop_confirm_far_m понижается до caution/far_unconfirmed, если его track_id нет в prev_stop_ids
    (треки уровня stop ДО этого понижения в прошлом кадре; см. 'stop_ids' в результате)."""
    for o in objects:
        o['level'], o['reason'] = object_level(o, in_path=not o.get('beyond_path', False),
                                               pending_score=pending_score, front_lift_m=front_lift_m,
                                               front_lift_max_low_m=front_lift_max_low_m)
    stop_ids = {o['track_id'] for o in objects if o['level'] == 'stop' and 'track_id' in o}
    if stop_confirm_far_m is not None:
        for o in objects:
            if (o['level'] == 'stop' and 'track_id' in o and o['distance_m'] >= stop_confirm_far_m
                    and o['track_id'] not in prev_stop_ids):
                o['level'], o['reason'] = 'caution', 'far_unconfirmed'
    stop = [o['distance_m'] for o in objects if o['level'] == 'stop']
    caution = [o['distance_m'] for o in objects if o['level'] == 'caution']
    sight = sight_distance(path_range, curvature, sight_c, range_cap) if path_available else 0.0
    if stop:
        status = 'stop'
    elif not path_available or sight < min_sight_m:
        status = 'unknown'
    elif caution:
        status = 'caution'
    else:
        status = 'clear'
    near = min(stop + caution, default=None)
    clear_to = 0.0 if not path_available else max(0.0, sight if near is None else min(sight, near))
    return {'status': status,
            'distance_m': min(stop) if stop else None,
            'caution_distance_m': min(caution) if caution else None,
            'sight_m': sight,
            'clear_to_m': clear_to,
            'stop_ids': stop_ids}
