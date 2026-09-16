"""
Стабильный контракт между "алгоритмом обнаружения" и "остальной командой"
(ROS2-нода, Docker, демо). Ничего в этом файле не знает про rclpy/ROS2/Docker --
только numpy. Кто угодно может вызвать ObstacleDetector из живой ноды, из
offline-скрипта на bag-файлах, или из тестов -- интерфейс не меняется.

Как это использовать в ROS2-ноде (псевдокод, для человека, который её пишет):

    from detector_interface import ObstacleDetector, parse_pointcloud2

    det = ObstacleDetector()
    frame_i = 0

    def on_pointcloud(msg):  # msg: sensor_msgs.msg.PointCloud2
        global frame_i
        x, y, z = parse_pointcloud2(msg.data, msg.point_step)
        result = det.detect(x, y, z, refit_path=(frame_i % 10 == 0))
        # refit_path=True раз в 10 кадров -- геометрия пути пересчитывается
        # не каждый кадр (~90мс), проверка препятствия -- каждый кадр (<2мс)
        publish(result)
        frame_i += 1
"""
import numpy as np

from rail_pathfit import find_floor_bumps, cluster_lines, pick_rail_pair, fit_path

# --- сырой парсинг PointCloud2 -----------------------------------------
# Layout эмпирически установлен по реальным данным: x,y,z,intensity (float32),
# ring (uint16), timestamp (float64) -> point_step=26 байт.
# Работает как с сообщением из rosbags (offline), так и с настоящим
# sensor_msgs/msg/PointCloud2 из rclpy (msg.data, msg.point_step) -- оба дают
# байтовый буфер + point_step, больше эта функция ничего не требует.

def parse_pointcloud2(data: bytes, point_step: int):
    buf = np.frombuffer(data, dtype=np.uint8).reshape(-1, point_step)
    x = buf[:, 0:4].view(np.float32).ravel()
    y = buf[:, 4:8].view(np.float32).ravel()
    z = buf[:, 8:12].view(np.float32).ravel()
    valid = np.isfinite(x) & np.isfinite(y) & np.isfinite(z)
    return x[valid], y[valid], z[valid]


def estimate_floor_z(x, y, z, near=2.0, far=15.0, half_width=3.0):
    """Грубая оценка высоты пола из самого кадра (а не жёсткая константа) --
    пол/крепление лидара может отличаться между бегами и между поездами."""
    fwd = -y
    m = (fwd > near) & (fwd < far) & (np.abs(x) < half_width)
    if m.sum() < 50:
        return float(np.percentile(z, 10))
    return float(np.percentile(z[m], 10))


class ObstacleDetector:
    """Инкапсулирует состояние между кадрами: путь (рельсы) фитится не на
    каждом кадре, а по запросу (refit_path=True) -- быстрая проверка
    препятствия использует последний посчитанный путь."""

    def __init__(self, near_cutoff=2.0, half_width=1.0, air_gap_above_floor=0.3, air_height=1.7):
        self.near_cutoff = near_cutoff
        self.half_width = half_width
        self.air_gap_above_floor = air_gap_above_floor
        self.air_height = air_height
        self._fit_fwd = None
        self._fitted_cl = None
        self._gauge = None

    def update_path(self, x, y, z) -> bool:
        """Медленный путь (~90мс на реальных данных). Возвращает True, если
        путь успешно найден и обновлён; False -- держим предыдущий фит."""
        pair, fit_fwd, cl = fit_path(x, y, z)
        if pair is None:
            return False
        self._fit_fwd, self._fitted_cl, self._gauge = fit_fwd, cl, pair[3]
        return True

    def _center_of_fwd(self, fwd):
        if self._fitted_cl is None:
            return np.zeros_like(fwd)  # нет фита пути -- прямая линия как fallback
        return np.interp(fwd, self._fit_fwd, self._fitted_cl)

    def check_frame(self, x, y, z) -> dict:
        """Быстрый путь (<2мс на реальных данных). Не пересчитывает путь --
        использует последний известный фит (или прямую линию, если фита нет)."""
        floor_z = estimate_floor_z(x, y, z)
        z_min = floor_z + self.air_gap_above_floor
        z_max = floor_z + self.air_height

        fwd = -y
        center = self._center_of_fwd(fwd)
        mask = (fwd > self.near_cutoff) & (np.abs(x - center) < self.half_width) & (z > z_min) & (z < z_max)
        d = fwd[mask]

        detected = d.size > 0
        return {
            'obstacle': bool(detected),
            'distance_m': float(d.min()) if detected else None,
            'n_points': int(mask.sum()),
            'path_available': self._fitted_cl is not None,
            'gauge_m': self._gauge,
        }

    def detect(self, x, y, z, refit_path=False) -> dict:
        """Единая точка входа. refit_path=True -- пересчитать путь в этом же
        вызове (звать редко, не каждый кадр)."""
        if refit_path:
            self.update_path(x, y, z)
        return self.check_frame(x, y, z)


if __name__ == '__main__':
    # самопроверка контракта на синтетических данных -- без ROS2, без bag-файлов
    from lidar_sim import simulate_frame

    det = ObstacleDetector()
    x, y, z, *_ = simulate_frame(obstacle_forward=None)
    det.update_path(x, y, z)
    print('путь после update_path:', 'найден' if det._fitted_cl is not None else 'не найден')

    x, y, z, *_ = simulate_frame(obstacle_forward=50.0, obstacle_radius=0.35)
    result = det.detect(x, y, z, refit_path=False)
    print('результат на кадре с препятствием на 50м:', result)
