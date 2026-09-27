"""Маркеры RViz по результату детектора: ось пути, коридор (зона проверки),
объекты, текст с решением. Координаты -- в системе лидара (frame_id из облака):
x -- вбок, вперёд = -y, z -- вверх."""
import numpy as np
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

NS = 'tunnel_od'
AXIS_STEP_M = 2.0          # шаг точек оси; каждая точка -- объект Point (~40 мкс в rclpy)
CORRIDOR_STEP_M = 4.0
FRAME_EVERY_M = 20.0       # поперечные рамки коридора
DEFAULT_AXIS_M = 40.0      # длина рисуемой оси, если путь не найден


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
    clear.action = Marker.DELETEALL
    arr.markers.append(clear)

    near = float(getattr(det, 'near_cutoff', 2.0))
    path_range = result.get('path_range_m')
    end = float(path_range) if path_range else DEFAULT_AXIS_M
    fwd = np.arange(near, max(end, near + AXIS_STEP_M) + 1e-6, AXIS_STEP_M)
    cx = track.center_at(fwd)
    tor = track.rail_top_at(fwd)
    path_ok = bool(result.get('path_available')) and track.center is not None

    # ось пути: головка рельса по оси
    axis = _marker(header, 0, Marker.LINE_STRIP,
                   _color(0.1, 0.9, 0.2) if path_ok else _color(0.6, 0.6, 0.6, 0.6), (0.08, 0, 0))
    axis.points = [_pt(x, -f, z) for f, x, z in zip(fwd, cx, tor)]
    arr.markers.append(axis)

    # коридор: четыре продольных ребра зоны (LINE_STRIP) + поперечные рамки
    lo, hi, hw = _zone_bounds(det)
    corr_col = _color(0.2, 0.6, 1.0, 0.5)
    k = np.unique(np.r_[np.arange(0, len(fwd), max(1, int(CORRIDOR_STEP_M / AXIS_STEP_M))), len(fwd) - 1])
    for j, (side, h) in enumerate(((-1, lo), (-1, hi), (1, lo), (1, hi))):
        edge = _marker(header, 10 + j, Marker.LINE_STRIP, corr_col, (0.04, 0, 0))
        edge.points = [_pt(cx[i] + side * hw, -fwd[i], tor[i] + h) for i in k]
        arr.markers.append(edge)
    frames = _marker(header, 1, Marker.LINE_LIST, corr_col, (0.04, 0, 0))
    for i in range(0, len(fwd), max(1, int(FRAME_EVERY_M / AXIS_STEP_M))):
        c = [(cx[i] - hw, tor[i] + lo), (cx[i] + hw, tor[i] + lo),
             (cx[i] + hw, tor[i] + hi), (cx[i] - hw, tor[i] + hi)]
        for a, b in zip(c, c[1:] + c[:1]):
            frames.points += [_pt(a[0], -fwd[i], a[1]), _pt(b[0], -fwd[i], b[1])]
    arr.markers.append(frames)

    # объекты: подтверждённые (тревога) -- красные, прочие -- жёлтые, мелкие/за осью -- серые
    objects = sorted(result.get('objects') or [], key=lambda o: o.get('distance_m', 1e9))
    alarm_objs = [o for o in objects if o.get('confirmed') and not o.get('beyond_path')]
    alarm_ids = {id(o) for o in alarm_objs}
    shown = alarm_objs + [o for o in objects if id(o) not in alarm_ids]
    for i, o in enumerate(shown[:max_objects]):
        d = float(o.get('distance_m', 0.0))
        lat = float(o.get('lateral_m', 0.0) or 0.0)
        h = max(float(o.get('height_m', 0.3) or 0.3), 0.2)
        base = float(track.rail_top_at(d)[0])
        x = float(track.center_at(d)) + lat
        if id(o) in alarm_ids:
            col = _color(1.0, 0.1, 0.1, 0.9)
        elif o.get('too_small') or o.get('beyond_path'):
            col = _color(0.6, 0.6, 0.6, 0.5)
        else:
            col = _color(1.0, 0.85, 0.1, 0.7)
        box = _marker(header, 100 + i, Marker.CUBE, col, (0.6, 0.6, h))
        box.pose.position = _pt(x, -(d + 0.3), base + h / 2)
        arr.markers.append(box)

    # текст с решением над коридором
    if result.get('obstacle'):
        text = f"ПРЕПЯТСТВИЕ {float(result['distance_m']):.1f} м"
        col = _color(1.0, 0.2, 0.2)
    else:
        text = 'путь свободен'
        col = _color(0.3, 1.0, 0.3)
    extra = []
    if result.get('clear_to_m') is not None:
        extra.append(f"просмотр {float(result['clear_to_m']):.0f} м")
    if path_range:
        extra.append(f'ось {float(path_range):.0f} м')
    if extra:
        text += ' (' + ', '.join(extra) + ')'
    label = _marker(header, 2, Marker.TEXT_VIEW_FACING, col, (0, 0, 1.0))
    label.pose.position = _pt(float(cx[0]), -8.0, float(tor[0]) + hi + 1.5)
    label.text = text
    arr.markers.append(label)
    return arr
