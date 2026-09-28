"""ROS 2-узел обнаружения препятствий: PointCloud2 -> ObstacleDetector -> результат.

Вход:  sensor_msgs/PointCloud2 (топик -- параметр `topic`; пусто = первый найденный
       топик этого типа, в записях это /lidar_points или /sensing/lidar/hesai128/pointcloud).
       canonical_input: true -- на входе каноническое облако от C++-узла tunnel_od_preproc
       (/tunnel_od/cloud: x, y, z float32, уже без пустых лучей и дублей, в осях ядра);
       разбор тогда -- np.frombuffer без копии, а из /tunnel_od/input_meta берутся время
       публикации исходного кадра, скорость поезда (S1) и сводка проверки входа.
       Без него облако разбирается здесь: нативной библиотекой tunnel_od_preproc (тот же
       код, что в C++-узле; parse_backend: auto|native) или parse_pointcloud2 (python).
Выход: /tunnel_od/result  -- std_msgs/String, JSON на каждый обработанный кадр: решение `status`
       (stop | caution | unknown | clear), `level`/`reason` объектов, `sight_m`, `clear_to_m`,
       `caution_distance_m`; побитовый повтор облака (skip_repeated) -- прошлый результат с
       `repeated: true`, детектор не вызывается;
       /tunnel_od/markers -- visualization_msgs/MarkerArray для RViz.

Кадры обрабатываются в отдельном потоке. Колбэк подписки только кладёт сообщение в
очередь на max_pending кадров; если она полна, самый старый кадр отбрасывается
(счётчик dropped_stale, предупреждение в лог). Так задержка не растёт без предела,
даже если детектор надолго не успевает. Догон (catch_up: auto): если в очереди ждёт
несколько кадров (стартовый всплеск ros2 bag play), берётся самый свежий, остальные
отбрасываются (dropped_catchup).

Защита по входу (input_guard.py): watchdog -- нет облаков дольше watchdog_timeout_s --
в /tunnel_od/result и маркер RViz идёт снимок `node_state: fault`, `reason: no_lidar_data`
(status unknown), пока данные не возобновятся (снимки -- только в топик, не в result_file);
прогрев -- первые warmup_frames кадров -- `node_state: warmup` (status unknown вместо clear).
Отброшенные кадры: dropped_stale (очередь полна), dropped_catchup (догон), dropped_total = сумма.

Путь (update_path, ~80 мс) по умолчанию пересчитывается в отдельном процессе по самому
свежему кадру (refit_mode: async, см. path_worker.py) -- тогда задержка кадра = разбор +
проверка. refit_mode: sync -- как в ядре: detect(refit_path=(i % refit_every == 0)).

Параметры ObstacleDetector -- всё, что лежит под `detector.` в config/detector.yaml,
плюс JSON-строка `detector_json` (перекрывает). Ничего не зашито в код.
"""
import json
import os
import threading
import time
from collections import OrderedDict, deque

import rclpy
import rclpy.executors
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
from visualization_msgs.msg import MarkerArray

from tunnel_od import ObstacleDetector, parse_pointcloud2
from tunnel_od.pointcloud import FrameRepeat, dedupe_rounded, hesai_columns

from .input_guard import Warmup, Watchdog, annotate, catch_up_enabled, fault_result, take_latest
from .markers import build_markers, build_state_markers
from .util import (Stats, build_detector_kwargs, canonical_xyz, dumps, limit_alarms, percentile,
                   preproc_deduped, raw_hesai_format, raw_point_count, unordered_raw)

CLOUD_TYPE = 'sensor_msgs/msg/PointCloud2'


def _stamp_sec(header):
    return header.stamp.sec + header.stamp.nanosec * 1e-9


END_OF_AXIS = 0.8


class DetectorNode(Node):
    def __init__(self):
        super().__init__('tunnel_od_detector',
                         automatically_declare_parameters_from_overrides=True)
        p = self._param
        self.topic = p('topic', '')
        self.refit_mode = str(p('refit_mode', 'async')).lower()
        if self.refit_mode not in ('async', 'sync'):
            raise ValueError(f'refit_mode: async | sync, получено {self.refit_mode!r}')
        self.refit_every = max(1, int(p('refit_every', 1)))
        self._recent_ranges = deque(maxlen=max(1, int(p('alarm_range_fits', 3))))
        self.stats_period = float(p('stats_period_s', 5.0))
        self.publish_markers = bool(p('publish_markers', True))
        self.max_marker_objects = int(p('marker_max_objects', 30))
        self.markers_every = max(1, int(p('markers_every', 1)))
        self.log_alarms = bool(p('log_alarms', True))
        self.result_file = p('result_file', '')
        self.stats_file = p('stats_file', '')
        depth = int(p('qos_depth', 5))
        reliability = str(p('qos_reliability', 'reliable')).lower()
        self.canonical_input = bool(p('canonical_input', False))
        self.meta_topic = p('meta_topic', '/tunnel_od/input_meta')
        self.meta_wait_s = max(0.0, float(p('meta_wait_s', 0.05)))
        self.max_pending = max(1, int(p('max_pending', 2)))
        self.parse_backend = str(p('parse_backend', 'auto')).lower()
        self.skip_repeated = bool(p('skip_repeated', True))
        self.catch_up = catch_up_enabled(p('catch_up', 'auto'), self.max_pending)
        self._watchdog = Watchdog(float(p('watchdog_timeout_s', 0.5)), float(p('watchdog_grace_s', 2.0)),
                                  t_start=time.monotonic())
        self._warmup = Warmup(int(p('warmup_frames', 3)))
        self._last_header = None
        p('detector_json', '')
        self._parse = self._make_parser()
        self._repeat = FrameRepeat()
        self._last_out = None

        det_params = {name: prm.value for name, prm in self.get_parameters_by_prefix('detector').items()}
        kwargs = build_detector_kwargs(det_params, self.get_parameter('detector_json').value,
                                       target=ObstacleDetector.__init__,
                                       warn=self.get_logger().warning)
        self.det = ObstacleDetector(**kwargs)
        self._warm_up(kwargs)
        self.fitter = None
        if self.refit_mode == 'async':
            from .path_worker import AsyncPathFitter
            self.fitter = AsyncPathFitter(kwargs)
        self.get_logger().info(f'ObstacleDetector({", ".join(f"{k}={v!r}" for k, v in sorted(kwargs.items()))})')

        self.qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=depth,
            reliability=ReliabilityPolicy.BEST_EFFORT if reliability == 'best_effort' else ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE)
        self.pub_result = self.create_publisher(String, '/tunnel_od/result', 10)
        self.pub_markers = self.create_publisher(MarkerArray, '/tunnel_od/markers', 2)

        self.stats = Stats()
        self._lock = threading.Lock()
        self._meta_cv = threading.Condition(self._lock)
        self._event = threading.Event()
        self._pending = deque()
        self._meta = OrderedDict()
        self._last_meta = None
        self._last_input = None
        self._e2e = []
        self._stop = False
        self._frame = 0
        self._sub = None
        self._t_start = time.monotonic()
        self._t_period = self._t_start
        self._result_fh = open(self.result_file, 'w', encoding='utf-8') if self.result_file else None
        self._file_lock = threading.Lock()

        if self.canonical_input:
            self.create_subscription(String, self.meta_topic, self._on_meta,
                                     QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=20,
                                                reliability=ReliabilityPolicy.RELIABLE))
        if self.topic:
            self._subscribe(self.topic)
        else:
            self._discover_timer = self.create_timer(0.5, self._discover)
            self.get_logger().info(f'топик не задан: жду первый топик типа {CLOUD_TYPE}')
        self.create_timer(self.stats_period, self._print_period)
        self.create_timer(self._watchdog.period_s(), self._watchdog_tick)
        self.get_logger().info(f'защита по входу: watchdog {self._watchdog.timeout_s:.2f} с '
                               f'(до первого кадра {self._watchdog.grace_s:.1f} с), прогрев {self._warmup.min_frames} '
                               f'кадр(ов), догон очереди {"вкл" if self.catch_up else "выкл"}')
        self._worker = threading.Thread(target=self._work_loop, name='detector', daemon=True)
        self._worker.start()

    def _warm_up(self, kwargs):
        """Прогрев на отдельном экземпляре и синтетическом облаке (пол + рельсы + стены):
        первый настоящий кадр иначе обрабатывается в 2-3 раза дольше, и на старте
        проигрывания теряются кадры."""
        try:
            import numpy as np
            t0 = time.monotonic()
            rng = np.random.default_rng(0)
            fwd = rng.uniform(2, 120, 60000).astype(np.float32)
            lat = rng.uniform(-3, 3, 60000).astype(np.float32)
            z = np.where(np.abs(np.abs(lat) - 0.76) < 0.04, -1.3, -1.5).astype(np.float32)
            z = np.where(np.abs(lat) > 2.5, rng.uniform(-1.5, 2.5, 60000), z).astype(np.float32)
            w = ObstacleDetector(**kwargs)
            for i in range(2):
                w.detect(lat, -fwd, z, refit_path=True, stamp=0.1 * i)
            self.get_logger().info(f'прогрев детектора {1e3 * (time.monotonic() - t0):.0f} мс')
        except Exception as e:
            self.get_logger().warning(f'прогрев не удался (не критично): {e}')

    def _make_parser(self):
        """Разбор входного облака. Возвращает функцию msg -> (x, y, z)."""
        if self.canonical_input:
            self.get_logger().info('вход: каноническое облако tunnel_od_preproc (np.frombuffer без копии)')
            self._parse_kind = 'canonical'
            return canonical_xyz
        if self.parse_backend not in ('auto', 'native', 'python'):
            raise ValueError(f'parse_backend: auto | native | python, получено {self.parse_backend!r}')
        if self.parse_backend != 'python':
            try:
                from tunnel_od_preproc import native
                if native.available():
                    self.get_logger().info('разбор облака: нативная библиотека tunnel_od_preproc (ctypes)')
                    self._parse_kind = 'native'
                    return native.parse_msg
                err = native.load_error
            except ImportError as e:
                err = str(e)
            if self.parse_backend == 'native':
                raise RuntimeError(f'parse_backend: native, но библиотека недоступна: {err}')
            self.get_logger().warning(f'нативный разбор недоступен ({err}) -- parse_pointcloud2')
        else:
            self.get_logger().info('разбор облака: parse_pointcloud2 (Python)')
        self._parse_kind = 'python'
        return lambda msg: parse_pointcloud2(msg.data, msg.point_step, msg.fields)

    def _param(self, name, default):
        if self.has_parameter(name):
            return self.get_parameter(name).value
        return self.declare_parameter(name, default).value

    def _discover(self):
        for name, types in sorted(self.get_topic_names_and_types()):
            if CLOUD_TYPE in types and not name.startswith('/tunnel_od/') and self.count_publishers(name):
                self._discover_timer.cancel()
                self._subscribe(name)
                return

    def _subscribe(self, topic):
        self.topic = topic
        self._sub = self.create_subscription(PointCloud2, topic, self._on_cloud, self.qos)
        how = ('в фоновом процессе по свежему кадру' if self.fitter
               else f'раз в {self.refit_every} кадр(ов) синхронно')
        self.get_logger().info(f'подписка на {topic} (QoS {self.qos.reliability.name}, depth {self.qos.depth}), '
                               f'пересчёт пути {how}')

    def _on_cloud(self, msg):
        t_recv = time.monotonic()
        dropped = False
        with self._lock:
            self.stats.on_receive(_stamp_sec(msg.header))
            self._last_header = msg.header
            recovered = self._watchdog.on_frame(t_recv)
            if recovered:
                self._warmup.reset()   # после провала входа ось надо построить заново
            if len(self._pending) >= self.max_pending:
                self._pending.popleft()
                self.stats.dropped += 1
                dropped = True
            self._pending.append((msg, t_recv))
        self._event.set()
        if dropped:
            self.get_logger().warning(f'детектор не успевает: отброшен кадр (всего {self.stats.dropped})',
                                      throttle_duration_sec=5.0)
        if recovered:
            self.get_logger().info('данные от лидара возобновились: прогрев заново')

    def _on_meta(self, msg):
        try:
            meta = json.loads(msg.data)
        except ValueError:
            return
        with self._lock:
            self._meta[(meta.get('stamp_sec'), meta.get('stamp_nanosec'))] = meta
            while len(self._meta) > 50:
                self._meta.popitem(last=False)
            self._last_meta = meta
            if 'input' in meta:
                self._last_input = meta['input']
            self._meta_cv.notify_all()

    def _work_loop(self):
        while not self._stop:
            if not self._event.wait(0.2):
                continue
            while True:
                skipped = 0
                with self._lock:
                    if self.catch_up:
                        item, skipped = take_latest(self._pending)
                        self.stats.dropped_catchup += skipped
                    else:
                        item = self._pending.popleft() if self._pending else None
                    if item is None:
                        self._event.clear()
                if item is None:
                    break
                if skipped:
                    self.get_logger().warning(f'догон: пропущено {skipped} старых кадр(ов) из очереди, '
                                              f'обрабатывается самый свежий (догоном всего {self.stats.dropped_catchup})',
                                              throttle_duration_sec=5.0)
                try:
                    self._process(*item)
                except Exception as e:
                    if not self.context.ok():   # SIGINT: rclpy закрыл контекст, кадр не ошибка -- останов
                        self._stop = True
                        break
                    self.stats.errors += 1
                    self.get_logger().error(f'ошибка обработки кадра: {type(e).__name__}: {e}')

    def _process(self, msg, t_recv):
        t0 = time.monotonic()
        stamp = _stamp_sec(msg.header)
        key = (msg.header.stamp.sec, msg.header.stamp.nanosec)
        with self._lock:
            # meta C++-узел публикует до облака, но доставка по двум топикам не упорядочена:
            # ждём её (до meta_wait_s), иначе решение о дедупе зависело бы от порядка доставки.
            if self.canonical_input and self.meta_wait_s > 0:
                self._meta_cv.wait_for(lambda: key in self._meta, timeout=self.meta_wait_s)
            meta = self._meta.pop(key, None)
            fmt_meta = meta or self._last_meta
        if self.skip_repeated and self._repeat.check(msg.data) and self._last_out is not None:
            out = dict(self._last_out, frame=self._frame, stamp=stamp, repeated=True,
                       queue_ms=(t0 - t_recv) * 1e3, latency_ms=(time.monotonic() - t_recv) * 1e3)
            text = dumps(out)
            self.pub_result.publish(String(data=text))
            with self._lock:
                self.stats.on_processed(out['latency_ms'], 0.0, out.get('obstacle'), out.get('distance_m'))
            self._frame += 1
            self._write_result(text)
            return
        x, y, z = self._parse(msg)
        if self._parse_kind != 'python':
            if self.canonical_input and fmt_meta is None:
                # meta ещё не было ни одной: формат неизвестен -- дедуп (для Hesai почти no-op)
                n_raw, hesai = msg.width * msg.height, False
            else:
                # meta кадра потерялась -- формат по прошлой meta (в потоке он не меняется)
                n_raw = raw_point_count(msg.width * msg.height, fmt_meta, self.canonical_input)
                hesai = raw_hesai_format(fmt_meta) if self.canonical_input else hesai_columns(n_raw, msg.fields)
            # C++-узел с dedupe_rounded: true снял дубли по сетке сам (meta parse.dedupe_rounded)
            if unordered_raw(n_raw, hesai) and not (self.canonical_input and preproc_deduped(fmt_meta)):
                x, y, z = dedupe_rounded(x, y, z)
        t1 = time.monotonic()
        path_updated = False
        if self.fitter:
            path_updated = self.fitter.apply(self.det)
            if self.fitter.last_error:
                self.get_logger().warning(f'пересчёт пути: {self.fitter.last_error}', throttle_duration_sec=10.0)
            refit = self._frame == 0
        else:
            refit = self._frame % self.refit_every == 0
        res = self.det.detect(x, y, z, refit_path=refit, stamp=stamp)
        if self.fitter:
            if (refit or path_updated) and res.get('path_available') and res.get('path_range_m') is not None:
                self._recent_ranges.append(res['path_range_m'])
            ranges = list(self._recent_ranges)
            if self._recent_ranges.maxlen > 1 and ranges and res.get('path_available'):
                lo = min(ranges) if len(ranges) == self._recent_ranges.maxlen else END_OF_AXIS * max(ranges)
                limit_alarms(res, lo + self.det.path_margin, END_OF_AXIS * max(ranges))
        t2 = time.monotonic()

        out = dict(res)
        out.update({
            'frame': self._frame,
            'frame_id': msg.header.frame_id,
            'topic': self.topic,
            'stamp': stamp,
            'n_points_cloud': int(len(x)),
            'refit_mode': self.refit_mode,
            'refit_path': refit or path_updated,
            'path_fit_ms': self.fitter.last_ms if self.fitter else None,
            'parse_ms': (t1 - t0) * 1e3,
            'detect_ms': (t2 - t1) * 1e3,
            'queue_ms': (t0 - t_recv) * 1e3,
            'repeated': False,
        })
        with self._lock:
            out['dropped_total'] = self.stats.dropped + self.stats.dropped_catchup
        if self.canonical_input:
            parse = (meta or {}).get('parse') or {}
            out.update({
                'train_speed_mps': meta.get('train_speed_mps') if meta else None,
                'preproc_ms': meta.get('preproc_ms') if meta else None,
                'transport_ms': meta.get('transport_ms') if meta else None,
                'n_points_raw': parse.get('n_in'),
                'n_points_valid': parse.get('n_valid'),
                'n_points_dedup': parse.get('n_dup_rounded'),
                'input_format': parse.get('format'),
            })
            if meta and meta.get('source_ts_ns'):
                out['e2e_ms'] = (time.time_ns() - int(meta['source_ts_ns'])) * 1e-6
                self._e2e.append(out['e2e_ms'])
        warmup_done = path_seen = False
        with self._lock:
            warming = self._warmup.update(res.get('path_available'))
            if warming:
                self.stats.warmup_frames += 1
                warmup_done = not self._warmup.active
                path_seen = self._warmup.path_seen
        if warming and warmup_done:
            self.get_logger().info(f'прогрев закончен ({self._warmup.frames} кадр(ов)): '
                                   f'{"ось построена" if path_seen else "ось не построена -- дальше unknown/no_path"}')
        annotate(out, warming)
        res['node_state'], res['reason'] = out['node_state'], out['reason']   # подпись в RViz
        if warming:
            res['status'] = out['status']
        out['latency_ms'] = (time.monotonic() - t_recv) * 1e3
        text = dumps(out)
        self._last_out = out
        self.pub_result.publish(String(data=text))
        with self._lock:
            self.stats.on_processed(out['latency_ms'], (t2 - t0) * 1e3, res.get('obstacle'), res.get('distance_m'))
        self._frame += 1

        if self.fitter:
            self.fitter.submit(getattr(self.det, '_frame', self._frame), x, y, z)
        self._write_result(text)
        if self.publish_markers and (self._frame - 1) % self.markers_every == 0:
            try:
                self.pub_markers.publish(build_markers(res, self.det.track_path(), self.det, msg.header,
                                                       self.max_marker_objects))
            except Exception as e:
                self.get_logger().warning(f'маркеры не построены: {e}', throttle_duration_sec=10.0)

        if self.log_alarms and res.get('obstacle'):
            n_stop = sum(1 for o in res.get('objects') or [] if o.get('level') == 'stop')
            pr = res.get('path_range_m')
            self.get_logger().warning(
                f'СТОП кадр {self._frame - 1}: препятствие {float(res["distance_m"]):.1f} м, '
                f'объектов stop {n_stop}, ось до {pr if pr is None else round(pr)} м, '
                f'свободно до {res.get("clear_to_m") or 0:.0f} м, задержка {out["latency_ms"]:.0f} мс')

    def _write_result(self, text):
        if self._result_fh:
            with self._file_lock:
                self._result_fh.write(text + '\n')

    def _watchdog_tick(self):
        """Таймер: нет облаков дольше watchdog_timeout_s -- снимок fault в топик результата и RViz,
        пока данные не возобновятся (решение -- input_guard.Watchdog). В result_file снимки не
        пишутся: это не кадры (после конца записи их было бы ~10 на каждый прогон)."""
        if self._stop:
            return
        try:
            self._watchdog_publish()
        except Exception as e:
            if not self.context.ok():   # SIGINT во время publish: не ошибка, узел останавливается
                return
            self.get_logger().warning(f'watchdog: {type(e).__name__}: {e}', throttle_duration_sec=10.0)

    def _watchdog_publish(self):
        now = time.monotonic()
        with self._lock:
            fault, age, changed = self._watchdog.check(now)
            if fault:
                self._watchdog.snapshots += 1
                self.stats.fault_snapshots += 1
                self.stats.fault_episodes = self._watchdog.episodes
            header = self._last_header
        if not fault:
            return
        out = fault_result(age, self._watchdog.timeout_s, topic=self.topic)
        self.pub_result.publish(String(data=dumps(out)))
        if self.publish_markers and header is not None:
            self.pub_markers.publish(build_state_markers(out, header))
        if changed:
            self.get_logger().warning(f'НЕТ ДАННЫХ от лидара {age:.1f} с (порог {self._watchdog.timeout_s:.2f} с): '
                                      f'статус fault, пока облака не возобновятся')
        else:
            self.get_logger().warning(f'нет данных от лидара уже {age:.1f} с', throttle_duration_sec=5.0)

    def _print_period(self):
        now = time.monotonic()
        dt, self._t_period = now - self._t_period, now
        with self._lock:
            p = self.stats.take_period()
            s = self.stats
            line = (f'{p["processed"] / dt:.1f} кадр/с (принято {p["received"] / dt:.1f}/с), '
                    f'задержка p50 {percentile(p["latency"], 50) or 0:.0f} / p95 {percentile(p["latency"], 95) or 0:.0f} мс; '
                    f'всего: принято {s.received}, обработано {s.processed}, отброшено {s.dropped + s.dropped_catchup} '
                    f'(не успел {s.dropped}, догон {s.dropped_catchup}), пропуски по stamp {s.stamp_gaps}, ошибок {s.errors}, '
                    f'кадров с тревогой {s.alarm_frames}, провалов входа {s.fault_episodes}')
        if p['received'] or p['processed']:
            self.get_logger().info(line)

    def final_summary(self):
        with self._lock:
            s = self.stats.summary()
        s.update({'topic': self.topic, 'refit_mode': self.refit_mode, 'refit_every': self.refit_every,
                  'path_fits_async': self.fitter.fits if self.fitter else None,
                  'wall_s': time.monotonic() - self._t_start, 'max_pending': self.max_pending,
                  'catch_up': self.catch_up, 'watchdog_timeout_s': self._watchdog.timeout_s,
                  'canonical_input': self.canonical_input})
        if self.canonical_input:
            with self._lock:
                meta, inp = self._last_meta, self._last_input
            s.update({'e2e_ms_p50': percentile(self._e2e, 50), 'e2e_ms_p95': percentile(self._e2e, 95),
                      'e2e_ms_max': max(self._e2e) if self._e2e else None,
                      'preproc': (meta or {}).get('counters'), 'input': inp})
        self.get_logger().info('ИТОГ ' + json.dumps(s, ensure_ascii=False))
        if self.stats_file:
            with open(self.stats_file, 'w', encoding='utf-8') as fh:
                json.dump(s, fh, ensure_ascii=False, indent=2)
        return s

    def shutdown(self):
        self._stop = True
        self._event.set()
        self._worker.join(timeout=2.0)
        if self.fitter:
            self.fitter.close()
        if self._result_fh:
            self._result_fh.close()


def main(args=None):
    os.environ.setdefault('OMP_NUM_THREADS', '1')
    rclpy.init(args=args)
    node = DetectorNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.shutdown()
        node.final_summary()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
