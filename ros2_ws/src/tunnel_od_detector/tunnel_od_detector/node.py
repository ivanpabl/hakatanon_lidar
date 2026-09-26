"""ROS 2-узел обнаружения препятствий: PointCloud2 -> ObstacleDetector -> результат.

Вход:  sensor_msgs/PointCloud2 (топик -- параметр `topic`; пусто = первый найденный
       топик этого типа, в записях это /lidar_points или /sensing/lidar/hesai128/pointcloud).
Выход: /tunnel_od/result  -- std_msgs/String, JSON на каждый обработанный кадр;
       /tunnel_od/markers -- visualization_msgs/MarkerArray для RViz.

Кадры обрабатываются в отдельном потоке. Колбэк подписки только кладёт сообщение в
«слот»; если прошлый кадр ещё не взят в работу, он заменяется новым (счётчик
dropped_stale). Так задержка не растёт, даже если детектор временно не успевает.

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

import rclpy
import rclpy.executors
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
from visualization_msgs.msg import MarkerArray

from tunnel_od import ObstacleDetector, parse_pointcloud2

from .markers import build_markers
from .util import Stats, build_detector_kwargs, dumps, percentile

CLOUD_TYPE = 'sensor_msgs/msg/PointCloud2'


def _stamp_sec(header):
    return header.stamp.sec + header.stamp.nanosec * 1e-9


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
        self.stats_period = float(p('stats_period_s', 5.0))
        self.publish_markers = bool(p('publish_markers', True))
        self.max_marker_objects = int(p('marker_max_objects', 30))
        self.markers_every = max(1, int(p('markers_every', 1)))
        self.log_alarms = bool(p('log_alarms', True))
        self.result_file = p('result_file', '')
        self.stats_file = p('stats_file', '')
        depth = int(p('qos_depth', 5))
        reliability = str(p('qos_reliability', 'reliable')).lower()
        p('detector_json', '')

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
        self._event = threading.Event()
        self._pending = None           # (msg, t_recv)
        self._stop = False
        self._frame = 0
        self._sub = None
        self._t_start = time.monotonic()
        self._t_period = self._t_start
        self._result_fh = open(self.result_file, 'w', encoding='utf-8') if self.result_file else None

        if self.topic:
            self._subscribe(self.topic)
        else:
            self._discover_timer = self.create_timer(0.5, self._discover)
            self.get_logger().info(f'топик не задан: жду первый топик типа {CLOUD_TYPE}')
        self.create_timer(self.stats_period, self._print_period)
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

    # ------------------------------------------------------------------ параметры
    def _param(self, name, default):
        if self.has_parameter(name):
            return self.get_parameter(name).value
        return self.declare_parameter(name, default).value

    # ------------------------------------------------------------------ подписка
    def _discover(self):
        own = {'/tunnel_od/result', '/tunnel_od/markers'}
        for name, types in sorted(self.get_topic_names_and_types()):
            if CLOUD_TYPE in types and name not in own:
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
        with self._lock:
            self.stats.on_receive(_stamp_sec(msg.header))
            if self._pending is not None:
                self.stats.dropped += 1
            self._pending = (msg, t_recv)
        self._event.set()

    # ------------------------------------------------------------------ обработка
    def _work_loop(self):
        while not self._stop:
            if not self._event.wait(0.2):
                continue
            with self._lock:
                item, self._pending = self._pending, None
                self._event.clear()
            if item is None:
                continue
            try:
                self._process(*item)
            except Exception as e:  # кадр с ошибкой не должен ронять узел
                self.stats.errors += 1
                self.get_logger().error(f'ошибка обработки кадра: {type(e).__name__}: {e}')

    def _process(self, msg, t_recv):
        t0 = time.monotonic()
        x, y, z = parse_pointcloud2(msg.data, msg.point_step, msg.fields)
        t1 = time.monotonic()
        stamp = _stamp_sec(msg.header)
        path_updated = False
        if self.fitter:
            path_updated = self.fitter.apply(self.det)
            if self.fitter.last_error:
                self.get_logger().warning(f'пересчёт пути: {self.fitter.last_error}', throttle_duration_sec=10.0)
            refit = self._frame == 0          # первый кадр -- синхронно, чтобы сразу был путь
        else:
            refit = self._frame % self.refit_every == 0
        res = self.det.detect(x, y, z, refit_path=refit, stamp=stamp)
        t2 = time.monotonic()

        # 1) результат -- сразу после детектора: задержка = приём -> публикация решения
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
        })
        with self._lock:
            out['dropped_total'] = self.stats.dropped
        out['latency_ms'] = (time.monotonic() - t_recv) * 1e3
        text = dumps(out)
        self.pub_result.publish(String(data=text))
        with self._lock:
            self.stats.on_processed(out['latency_ms'], (t2 - t0) * 1e3, res.get('obstacle'), res.get('distance_m'))
        self._frame += 1

        # 2) фоновый пересчёт пути по этому кадру, если процесс свободен
        if self.fitter:
            self.fitter.submit(getattr(self.det, '_frame', self._frame), x, y, z)
        # 3) файл и маркеры -- после решения, на задержку не влияют
        if self._result_fh:
            self._result_fh.write(text + '\n')
        if self.publish_markers and (self._frame - 1) % self.markers_every == 0:
            try:
                self.pub_markers.publish(build_markers(res, self.det.track_path(), self.det, msg.header,
                                                       self.max_marker_objects))
            except Exception as e:
                self.get_logger().warning(f'маркеры не построены: {e}', throttle_duration_sec=10.0)

        if self.log_alarms and res.get('obstacle'):
            n_conf = sum(1 for o in res.get('objects') or [] if o.get('confirmed') and not o.get('beyond_path'))
            pr = res.get('path_range_m')
            self.get_logger().warning(
                f'ТРЕВОГА кадр {self._frame - 1}: препятствие {float(res["distance_m"]):.1f} м, '
                f'подтверждённых объектов {n_conf}, ось до {pr if pr is None else round(pr)} м, '
                f'задержка {out["latency_ms"]:.0f} мс')

    # ------------------------------------------------------------------ сводки
    def _print_period(self):
        now = time.monotonic()
        dt, self._t_period = now - self._t_period, now
        with self._lock:
            p = self.stats.take_period()
            s = self.stats
            line = (f'{p["processed"] / dt:.1f} кадр/с (принято {p["received"] / dt:.1f}/с), '
                    f'задержка p50 {percentile(p["latency"], 50) or 0:.0f} / p95 {percentile(p["latency"], 95) or 0:.0f} мс; '
                    f'всего: принято {s.received}, обработано {s.processed}, отброшено {s.dropped}, '
                    f'пропуски по stamp {s.stamp_gaps}, ошибок {s.errors}, кадров с тревогой {s.alarm_frames}')
        if p['received'] or p['processed']:
            self.get_logger().info(line)

    def final_summary(self):
        with self._lock:
            s = self.stats.summary()
        s.update({'topic': self.topic, 'refit_mode': self.refit_mode, 'refit_every': self.refit_every,
                  'path_fits_async': self.fitter.fits if self.fitter else None,
                  'wall_s': time.monotonic() - self._t_start})
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
