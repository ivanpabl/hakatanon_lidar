"""Маркеры RViz по результату детектора: ось пути, коридор (зона проверки),
объекты, текст с решением. Координаты -- в системе лидара (frame_id из облака):
x -- вбок, вперёд = -y, z -- вверх."""
import numpy as np
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

from .util import LEVEL_COLOR, status_text

NS = 'tunnel_od'
NS_TEXT = 'tunnel_od_text'
AXIS_STEP_M = 2.0
CORRIDOR_STEP_M = 4.0
FRAME_EVERY_M = 20.0
DEFAULT_AXIS_M = 40.0
# Цвет зоны проверки по решению: зелёный -- свободно, жёлтый -- внимание, красный -- СТОП.
ZONE_COLOR = {'stop': (1.0, 0.15, 0.15, 0.9), 'caution': (1.0, 0.85, 0.1, 0.8),
              'clear': (0.2, 1.0, 0.3, 0.7), 'unknown': (0.6, 0.6, 0.6, 0.5)}
LABEL_SIZE_M = 2.4      # высота главной подписи решения
OBJ_LABEL_SIZE_M = 1.6  # подпись дистанции над объектом тревоги


def _color(r, g, b, a=1.0):
    return ColorRGBA(r=float(r), g=float(g), b=float(b), a=float(a))


def _pt(x, y, z):
    return Point(x=float(x), y=float(y), z=float(z))


def _marker(header, mid, mtype, color, scale=(0.1, 0.1, 0.1), ns=NS):
    m = Marker()
    m.header = header
    m.ns = ns
    m.id = mid
    m.type = mtype
    m.action = Marker.ADD
    m.pose.orientation.w = 1.0
    m.scale.x, m.scale.y, m.scale.z = (float(s) for s in scale)
    m.color = color
    return m


def _zone_bounds(det):
    """(низ, верх, полуширина) зоны детектора; при ступенчатой зоне -- огибающая."""
    zone = getattr(det, 'zone', None) or ((0.15, 2.0, 1.0),)
    lo = min(z[0] for z in zone)
    hi = max(z[1] for z in zone)
    hw = getattr(det, 'half_width', None) or max(z[2] for z in zone)
    return lo, hi, hw


def build_markers(result, track, det, header, max_objects=30):
    """result -- dict из detect(); track -- det.track_path(); det -- для границ зоны."""
    arr = MarkerArray()
    clear = Marker()
    clear.header = header
    clear.ns = NS
    clear.id = -1  # не совпадает с осью (id 0): иначе RViz -- «Duplicate Markers»
    clear.action = Marker.DELETEALL
    arr.markers.append(clear)

    near = float(getattr(det, 'near_cutoff', 2.0))
    path_range = result.get('path_range_m')
    end = float(path_range) if path_range else DEFAULT_AXIS_M
    fwd = np.arange(near, max(end, near + AXIS_STEP_M) + 1e-6, AXIS_STEP_M)
    cx = track.center_at(fwd)
    tor = track.rail_top_at(fwd)
    path_ok = bool(result.get('path_available')) and track.center is not None

    axis = _marker(header, 0, Marker.LINE_STRIP,
                   _color(0.1, 0.9, 0.2) if path_ok else _color(0.6, 0.6, 0.6, 0.6), (0.08, 0, 0))
    axis.points = [_pt(x, -f, z) for f, x, z in zip(fwd, cx, tor)]
    arr.markers.append(axis)

    lo, hi, hw = _zone_bounds(det)
    corr_col = _color(*ZONE_COLOR.get(result.get('status'), ZONE_COLOR['clear']))
    k = np.unique(np.r_[np.arange(0, len(fwd), max(1, int(CORRIDOR_STEP_M / AXIS_STEP_M))), len(fwd) - 1])
    for j, (side, h) in enumerate(((-1, lo), (-1, hi), (1, lo), (1, hi))):
        edge = _marker(header, 10 + j, Marker.LINE_STRIP, corr_col, (0.08, 0, 0))
        edge.points = [_pt(cx[i] + side * hw, -fwd[i], tor[i] + h) for i in k]
        arr.markers.append(edge)
    frames = _marker(header, 1, Marker.LINE_LIST, corr_col, (0.06, 0, 0))
    for i in range(0, len(fwd), max(1, int(FRAME_EVERY_M / AXIS_STEP_M))):
        c = [(cx[i] - hw, tor[i] + lo), (cx[i] + hw, tor[i] + lo),
             (cx[i] + hw, tor[i] + hi), (cx[i] - hw, tor[i] + hi)]
        for a, b in zip(c, c[1:] + c[:1]):
            frames.points += [_pt(a[0], -fwd[i], a[1]), _pt(b[0], -fwd[i], b[1])]
    arr.markers.append(frames)

    rank = {'stop': 0, 'caution': 1}
    shown = sorted(result.get('objects') or [], key=lambda o: (rank.get(o.get('level'), 2), o.get('distance_m', 1e9)))
    for i, o in enumerate(shown[:max_objects]):
        d = float(o.get('distance_m', 0.0))
        lat = float(o.get('lateral_m', 0.0) or 0.0)
        h = max(float(o.get('height_m', 0.3) or 0.3), 0.2)
        base = float(track.rail_top_at(d)[0])
        x = float(track.center_at(d)) + lat
        level = o.get('level')
        alarm = level in rank
        w = 1.2 if alarm else 0.6
        box = _marker(header, 100 + i, Marker.CUBE, _color(*LEVEL_COLOR.get(level, LEVEL_COLOR[None])), (w, w, h))
        box.pose.position = _pt(x, -(d + w / 2), base + h / 2)
        arr.markers.append(box)
        if alarm and i < 3:  # дистанция над объектом тревоги: видно, какой объект дал решение
            tag = _marker(header, 200 + i, Marker.TEXT_VIEW_FACING, _color(*LEVEL_COLOR[level][:3]),
                          (0, 0, OBJ_LABEL_SIZE_M))
            tag.pose.position = _pt(x, -(d + w / 2), base + h + 1.2)
            tag.text = f'{d:.0f} m'
            arr.markers.append(tag)

    text, rgb = status_text(result)
    label = _marker(header, 2, Marker.TEXT_VIEW_FACING, _color(*rgb), (0, 0, LABEL_SIZE_M), ns=NS_TEXT)
    label.pose.position = _pt(float(cx[0]), -8.0, float(tor[0]) + hi + 3.0)
    label.text = text
    arr.markers.append(label)
    if path_range:  # мелко под главной подписью: до скольких метров найдена ось пути
        sub = _marker(header, 3, Marker.TEXT_VIEW_FACING, _color(0.8, 0.8, 0.85), (0, 0, 0.8))
        sub.pose.position = _pt(float(cx[0]), -8.0, float(tor[0]) + hi + 1.3)
        sub.text = f'path axis {float(path_range):.0f} m'
        arr.markers.append(sub)
    return arr
