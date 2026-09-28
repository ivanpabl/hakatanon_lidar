"""Части узла без зависимости от ROS 2: параметры детектора, JSON, статистика.
Вынесены отдельно, чтобы проверять их pytest-ом без rclpy."""
import dataclasses
import inspect
import json
import math
from collections import deque

import numpy as np



def _convert_value(key, value):
    """Значение ROS-параметра -> значение kwargs. ROS-параметры не умеют None и
    вложенные списки, поэтому:
    - строки 'none'/'null'/'' -> None;
    - zone: плоский список [низ, верх, полуширина, ...] -> ((низ, верх, полуширина), ...),
      или имя константы ядра ('GAUGE_METRO', 'RECT_DEFAULT');
    - списки -> tuple (ядро делает tuple(zone) и т.п.)."""
    if isinstance(value, str) and value.strip().lower() in ('none', 'null', ''):
        return None
    if key == 'zone':
        if isinstance(value, str):
            import tunnel_od
            if not hasattr(tunnel_od, value):
                raise ValueError(f'zone: в tunnel_od нет константы {value!r}')
            return getattr(tunnel_od, value)
        flat = list(value)
        if flat and not isinstance(flat[0], (list, tuple)):
            if len(flat) % 3:
                raise ValueError('zone: нужен плоский список троек (низ, верх, полуширина)')
            return tuple(tuple(float(v) for v in flat[i:i + 3]) for i in range(0, len(flat), 3))
        return tuple(tuple(float(v) for v in step) for step in flat)
    if isinstance(value, list):
        return tuple(value)
    return value


def build_detector_kwargs(ros_params: dict, json_override: str = '', target=None, warn=print):
    """kwargs для ObstacleDetector.

    ros_params -- {имя: значение} из ROS-параметров с префиксом 'detector.'
    (config/detector.yaml); json_override -- JSON-строка, перекрывает их и позволяет
    задать то, что ROS-параметрами не выразить (null, вложенные списки).
    Ключи, которых нет в сигнатуре target (ObstacleDetector.__init__), отбрасываются
    с предупреждением: конфиг может опережать или отставать от версии ядра."""
    kwargs = {k: _convert_value(k, v) for k, v in ros_params.items()}
    if json_override and json_override.strip():
        extra = json.loads(json_override)
        if not isinstance(extra, dict):
            raise ValueError('detector_json должен быть JSON-объектом')
        kwargs.update({k: _convert_value(k, v) if k == 'zone' and v is not None else v
                       for k, v in extra.items()})
    if target is not None:
        sig = inspect.signature(target)
        if not any(p.kind == p.VAR_KEYWORD for p in sig.parameters.values()):
            known = set(sig.parameters) - {'self'}
            for k in sorted(set(kwargs) - known):
                warn(f'параметр detector.{k} неизвестен этой версии ObstacleDetector -- пропущен')
                kwargs.pop(k)
    return kwargs



CANONICAL_FIELDS = (('x', 0, 7), ('y', 4, 7), ('z', 8, 7))


def is_canonical(msg):
    """Облако от tunnel_od_preproc: x, y, z float32 подряд, point_step 12."""
    return (msg.point_step == 12 and len(msg.fields) == 3 and not msg.is_bigendian
            and all((f.name, f.offset, f.datatype) == c for f, c in zip(msg.fields, CANONICAL_FIELDS)))


def canonical_xyz(msg):
    """Каноническое облако -> (x, y, z) -- представления над буфером сообщения, без копии.
    parse_pointcloud2 сюда нельзя: при числе точек, кратном 256, он удалил бы "дубли"
    уже очищенного облака."""
    if not is_canonical(msg):
        raise ValueError(f'не каноническое облако: point_step {msg.point_step}, '
                         f'fields {[(f.name, f.offset, f.datatype) for f in msg.fields]}')
    a = np.frombuffer(msg.data, dtype=np.float32).reshape(-1, 3)
    return a[:, 0], a[:, 1], a[:, 2]


def required_fwd_range(det_kwargs):
    """Какой диапазон дальности вперёд ядро может использовать при этих параметрах:
    (min, max). Обрезка облака в tunnel_od_preproc безопасна, только если
    crop_fwd_min <= min и crop_fwd_max >= max (см. docs/input_format.md, "Обрезка")."""
    from tunnel_od.detection.detector import ObstacleDetector
    from tunnel_od.geometry import bed, path, rails
    defaults = {k: p.default for k, p in inspect.signature(ObstacleDetector.__init__).parameters.items()}
    near = float(det_kwargs.get('near_cutoff', defaults['near_cutoff']))
    far = float(det_kwargs.get('max_range', defaults['max_range']))
    bed_sig = inspect.signature(bed.estimate_bed_profile).parameters
    floor_sig = inspect.signature(bed.estimate_floor_z).parameters
    ext_sig = inspect.signature(path.extend_path_by_walls).parameters
    lo = min(near, rails.NEAR, bed_sig['near'].default, floor_sig['near'].default)
    hi = max(far, rails.FAR, bed_sig['far'].default, ext_sig['far'].default, floor_sig['far'].default)
    return lo, hi


COLUMN_PAIR = 256

LEVEL_COLOR = {'stop': (1.0, 0.1, 0.1, 0.9), 'caution': (1.0, 0.85, 0.1, 0.8), None: (0.6, 0.6, 0.6, 0.5)}


def unordered_raw(n, hesai=True) -> bool:
    """Исходное облако не из пар столбцов Hesai (синтетика организаторов): C++ дубли не снимал,
    их снимает запасной способ (tunnel_od.pointcloud.dedupe_rounded). hesai=False -- облако
    не Hesai (нет поля timestamp), даже если точек кратно 256. n неизвестно (None) -- не трогаем."""
    return bool(n) and (not hesai or n % COLUMN_PAIR != 0)


def raw_point_count(n_msg, meta, canonical):
    """Число точек исходного облака: из сообщения или, за C++-приёмом, из его meta (parse.n_in)."""
    if canonical:
        return ((meta or {}).get('parse') or {}).get('n_in')
    return n_msg


def raw_hesai_format(meta) -> bool:
    """C++-приём распознал исходное облако как Hesai из столбцов (meta parse.format)."""
    return ((meta or {}).get('parse') or {}).get('format') == 'legacy_hesai'


def preproc_deduped(meta) -> bool:
    """C++-приём уже снял дубли по сетке 1 см (meta parse.dedupe_rounded, параметр dedupe_rounded
    узла tunnel_od_preproc) -- dedupe_rounded в узле не нужен. Нет meta или старый C++ без поля --
    False: дубли снимает узел (повтор безвреден, dedupe_rounded идемпотентен)."""
    return bool(((meta or {}).get('parse') or {}).get('dedupe_rounded'))


def status_text(result):
    """Строка решения для RViz и логов и её цвет. Латиница: шрифт RViz не рисует кириллицу."""
    s = result.get('status')
    state = result.get('node_state')
    if state == 'fault':
        return f"NO LIDAR DATA {float(result.get('since_last_cloud_s') or 0.0):.1f} s", (1.0, 0.2, 0.2)
    if s == 'stop':
        return f"STOP {float(result['distance_m']):.0f} m", (1.0, 0.2, 0.2)
    if state == 'warmup':
        return 'WARMING UP', (0.6, 0.7, 1.0)
    if s == 'caution':
        return f"CAUTION {float(result['caution_distance_m']):.0f} m", (1.0, 0.85, 0.1)
    if s == 'unknown':
        return 'PATH UNKNOWN', (0.7, 0.7, 0.7)
    return f"CLEAR to {float(result.get('clear_to_m') or 0.0):.0f} m", (0.3, 1.0, 0.3)


def limit_alarms(res, limit_m, end_m=None):
    """Тревога только у подтверждённых объектов не дальше limit_m (м).

    В режиме refit_mode: async путь приходит из фонового процесса с опозданием на кадр, а конец
    известной оси скачет от кадра к кадру (199 -> 153 -> 204 -> 131 м). С осью прошлого кадра
    дальний кандидат, который по оси текущего кадра был бы «за концом оси», поднимает ложную
    тревогу. Узел поэтому не поднимает тревогу у объектов у конца оси: дальше limit_m (минимум
    дальности оси по последним пересчётам) и дальше end_m (доля от максимума). Второе условие
    нужно, чтобы один короткий пересчёт (ось 52 м при объекте на 56 м) не гасил тревогу у объекта
    далеко от конца оси лишний кадр. Такие объекты -- beyond_path, как у ядра: уровень stop
    становится caution (beyond_path), статус, дистанции и clear_to_m пересчитываются так же, как
    в decision.decide. Объект без уровня (level) считается stop, если подтверждён и в пределах оси."""
    if limit_m is None:
        return res
    objects = res.get('objects') or []
    for o in objects:
        d = o['distance_m']
        if o.get('confirmed') and not o.get('beyond_path') and d > limit_m and (end_m is None or d > end_m):
            o['beyond_path'] = True
            o['beyond_recent_path'] = True
            if o.get('level') == 'stop':
                o['level'], o['reason'] = 'caution', 'beyond_path'

    def level(o):
        if 'level' in o:
            return o['level']
        return 'stop' if o.get('confirmed') and not o.get('beyond_path') else None

    stop = [o['distance_m'] for o in objects if level(o) == 'stop']
    caution = [o['distance_m'] for o in objects if level(o) == 'caution']
    res['obstacle'] = bool(stop)
    res['distance_m'] = min(stop) if stop else None
    if 'status' in res:
        if stop:
            res['status'] = 'stop'
        elif res['status'] != 'unknown':
            res['status'] = 'caution' if caution else 'clear'
        res['caution_distance_m'] = min(caution) if caution else None
        if res['status'] != 'unknown' and res.get('sight_m') is not None:
            near = min(stop + caution, default=None)
            sight = float(res['sight_m'])
            res['clear_to_m'] = max(0.0, sight if near is None else min(sight, near))
    res['alarm_range_m'] = limit_m
    return res


def to_jsonable(obj, _depth=0):
    """Любой результат детектора -> то, что берёт json.dumps без ошибок:
    numpy-скаляры и массивы, NaN/inf (-> None), tuple, dataclass, Path, неизвестные
    типы (-> str). Ключи словарей -> str."""
    if _depth > 20:
        return str(obj)
    if obj is None or isinstance(obj, (bool, str)):
        return obj
    if isinstance(obj, np.bool_):
        return bool(obj)
    if isinstance(obj, (int, np.integer)):
        return int(obj)
    if isinstance(obj, (float, np.floating)):
        f = float(obj)
        return f if math.isfinite(f) else None
    if isinstance(obj, np.ndarray):
        return to_jsonable(obj.tolist(), _depth + 1)
    if isinstance(obj, dict):
        return {str(k): to_jsonable(v, _depth + 1) for k, v in obj.items()}
    if isinstance(obj, (list, tuple, set, frozenset, deque)):
        return [to_jsonable(v, _depth + 1) for v in obj]
    if dataclasses.is_dataclass(obj) and not isinstance(obj, type):
        return to_jsonable(dataclasses.asdict(obj), _depth + 1)
    if hasattr(obj, 'item'):
        try:
            return to_jsonable(obj.item(), _depth + 1)
        except Exception:
            pass
    return str(obj)


def dumps(obj) -> str:
    return json.dumps(to_jsonable(obj), ensure_ascii=False, allow_nan=False, separators=(',', ':'))



def percentile(values, q):
    return float(np.percentile(values, q)) if len(values) else None


class Stats:
    """Счётчики узла. Задержки -- за всё время работы и за последний период сводки."""

    def __init__(self):
        self.received = 0
        self.processed = 0
        self.dropped = 0
        self.dropped_catchup = 0
        self.fault_snapshots = 0
        self.fault_episodes = 0
        self.warmup_frames = 0
        self.errors = 0
        self.alarm_frames = 0
        self.stamp_gaps = 0
        self.latency_ms = []
        self.proc_ms = []
        self.min_alarm_distance = None
        self._period = {'processed': 0, 'latency': [], 'received': 0}
        self._last_stamp = None
        self._period_dt = deque(maxlen=50)

    def on_receive(self, stamp):
        self.received += 1
        self._period['received'] += 1
        if stamp is not None and self._last_stamp is not None:
            dt = stamp - self._last_stamp
            if dt > 0:
                self._period_dt.append(dt)
                nominal = float(np.median(self._period_dt))
                if nominal > 0 and dt > 1.5 * nominal:
                    self.stamp_gaps += int(round(dt / nominal)) - 1
        self._last_stamp = stamp

    def on_processed(self, latency_ms, proc_ms, obstacle, distance):
        self.processed += 1
        self.latency_ms.append(latency_ms)
        self.proc_ms.append(proc_ms)
        self._period['processed'] += 1
        self._period['latency'].append(latency_ms)
        if obstacle:
            self.alarm_frames += 1
            if distance is not None and (self.min_alarm_distance is None or distance < self.min_alarm_distance):
                self.min_alarm_distance = float(distance)

    def take_period(self):
        p, self._period = self._period, {'processed': 0, 'latency': [], 'received': 0}
        return p

    def summary(self):
        return {
            'received': self.received,
            'processed': self.processed,
            'dropped_stale': self.dropped,
            'dropped_catchup': self.dropped_catchup,
            'dropped_total': self.dropped + self.dropped_catchup,
            'fault_snapshots': self.fault_snapshots,
            'fault_episodes': self.fault_episodes,
            'warmup_frames': self.warmup_frames,
            'stamp_gaps': self.stamp_gaps,
            'errors': self.errors,
            'alarm_frames': self.alarm_frames,
            'min_alarm_distance_m': self.min_alarm_distance,
            'latency_ms_p50': percentile(self.latency_ms, 50),
            'latency_ms_p95': percentile(self.latency_ms, 95),
            'latency_ms_max': max(self.latency_ms) if self.latency_ms else None,
            'proc_ms_p50': percentile(self.proc_ms, 50),
            'proc_ms_p95': percentile(self.proc_ms, 95),
        }
