"""Части узла без зависимости от ROS 2: параметры детектора, JSON, статистика.
Вынесены отдельно, чтобы проверять их pytest-ом без rclpy."""
import dataclasses
import inspect
import json
import math
from collections import deque

import numpy as np


# ---------------------------------------------------------------- параметры детектора

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


# ---------------------------------------------------------------- JSON

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
    if hasattr(obj, 'item'):          # прочие numpy-подобные скаляры
        try:
            return to_jsonable(obj.item(), _depth + 1)
        except Exception:
            pass
    return str(obj)


def dumps(obj) -> str:
    return json.dumps(to_jsonable(obj), ensure_ascii=False, allow_nan=False, separators=(',', ':'))


# ---------------------------------------------------------------- статистика

def percentile(values, q):
    return float(np.percentile(values, q)) if len(values) else None


class Stats:
    """Счётчики узла. Задержки -- за всё время работы и за последний период сводки."""

    def __init__(self):
        self.received = 0          # принято сообщений (колбэк подписки)
        self.processed = 0         # обработано детектором
        self.dropped = 0           # отброшено как устаревшие (пришёл более новый кадр)
        self.errors = 0            # исключения при обработке кадра
        self.alarm_frames = 0      # кадров с obstacle=True
        self.stamp_gaps = 0        # кадров, пропущенных по header.stamp (потери до узла или в записи)
        self.latency_ms = []       # приём -> публикация результата
        self.proc_ms = []          # разбор + детектор
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
