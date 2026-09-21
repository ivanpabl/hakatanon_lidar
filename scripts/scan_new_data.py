"""End-to-end run of our actual detector_interface.ObstacleDetector against the
new, much longer (20 min, 11271 frames) bag -- a real generalization test on data
none of our thresholds were tuned against."""
import time
import numpy as np
from rosbags.highlevel import AnyReader
from rosbags.typesys import Stores, get_typestore
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

from detector_interface import ObstacleDetector

ts = get_typestore(Stores.ROS2_HUMBLE)
BAG = Path('/home/pablo/Documents/hakaton/new_data')
OUT = Path(__file__).parent / 'out'
OUT.mkdir(exist_ok=True)

REFIT_EVERY = 50  # refresh path geometry every 50 frames (~5s at ~9.86Hz)

det = ObstacleDetector()

times, dists, obstacle_flags, path_ok_flags, n_points_list = [], [], [], [], []
t_start = time.perf_counter()

with AnyReader([BAG], default_typestore=ts) as reader:
    conn = reader.connections[0]
    t0 = None
    for i, (connection, timestamp, rawdata) in enumerate(reader.messages(connections=[conn])):
        msg = reader.deserialize(rawdata, connection.msgtype)
        buf = np.frombuffer(msg.data, dtype=np.uint8).reshape(-1, msg.point_step)
        x = buf[:, 0:4].view(np.float32).ravel()
        y = buf[:, 4:8].view(np.float32).ravel()
        z = buf[:, 8:12].view(np.float32).ravel()
        valid = np.isfinite(x) & np.isfinite(y) & np.isfinite(z)
        x, y, z = x[valid], y[valid], z[valid]

        if t0 is None:
            t0 = timestamp
        times.append((timestamp - t0) / 1e9)

        result = det.detect(x, y, z, refit_path=(i % REFIT_EVERY == 0))
        obstacle_flags.append(result['obstacle'])
        dists.append(result['distance_m'] if result['distance_m'] is not None else np.nan)
        path_ok_flags.append(result['path_available'])
        n_points_list.append(result['n_points'])

        del msg, rawdata, x, y, z, buf
        if i % 1000 == 0:
            print(f'  ...{i} кадров обработано, t={times[-1]:.0f}с')

wall_time = time.perf_counter() - t_start
n_frames = len(times)
times = np.array(times)
dists = np.array(dists)
obstacle_flags = np.array(obstacle_flags)
path_ok_flags = np.array(path_ok_flags)
n_points_arr = np.array(n_points_list)

print()
print(f'Всего кадров: {n_frames}, длительность записи: {times[-1]:.0f}с ({times[-1]/60:.1f} мин)')
print(f'Время обработки всей записи: {wall_time:.1f}с ({n_frames/wall_time:.1f} кадров/с в среднем)')
print(f'Путь найден в {path_ok_flags.sum()}/{n_frames} кадрах ({100*path_ok_flags.mean():.1f}%)')
print(f'Препятствие обнаружено в {obstacle_flags.sum()}/{n_frames} кадрах ({100*obstacle_flags.mean():.1f}%)')
if obstacle_flags.any():
    dd = dists[obstacle_flags]
    print(f'Дистанции при обнаружении: min={np.nanmin(dd):.1f}м, max={np.nanmax(dd):.1f}м, median={np.nanmedian(dd):.1f}м')

fig, axes = plt.subplots(3, 1, figsize=(14, 10), sharex=True)
axes[0].plot(times, dists, '.', ms=2)
axes[0].set_ylabel('дистанция, м')
axes[0].set_title(f'new_data: {n_frames} кадров, обнаружено в {100*obstacle_flags.mean():.1f}% кадров')
axes[0].grid(alpha=0.3)

axes[1].plot(times, obstacle_flags.astype(int), '.', ms=2, c='red')
axes[1].set_ylabel('obstacle (0/1)')
axes[1].grid(alpha=0.3)

axes[2].plot(times, path_ok_flags.astype(int), '.', ms=2, c='green')
axes[2].set_ylabel('path_available (0/1)')
axes[2].set_xlabel('время, с')
axes[2].grid(alpha=0.3)

fig.tight_layout()
fig.savefig(OUT / 'new_data_scan.png', dpi=130)
print('saved', OUT / 'new_data_scan.png')
