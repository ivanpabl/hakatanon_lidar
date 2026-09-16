"""Measure per-frame processing latency / FPS of our detection code on real bags.
No ROS2 / Docker needed for this -- it's a pure algorithmic timing benchmark,
directly relevant to ТЗ criterion 8.3 (speed / real-time performance)."""
import time
import numpy as np
from rosbags.highlevel import AnyReader
from rosbags.typesys import Stores, get_typestore
from pathlib import Path

from rail_pathfit import find_floor_bumps

ts = get_typestore(Stores.ROS2_HUMBLE)
ROOT = Path('/home/pablo/Documents/hakaton/Датасет/archive/for_hackathon')
BAGS = sorted([p for p in ROOT.iterdir() if p.is_dir()])
N_FRAMES = 15  # frames to time per bag (after 3 warmup frames)


def load_xyz_timed(rawdata, msgtype, reader):
    t0 = time.perf_counter()
    msg = reader.deserialize(rawdata, msgtype)
    buf = np.frombuffer(msg.data, dtype=np.uint8).reshape(-1, msg.point_step)
    x = buf[:, 0:4].view(np.float32).ravel()
    y = buf[:, 4:8].view(np.float32).ravel()
    z = buf[:, 8:12].view(np.float32).ravel()
    valid = np.isfinite(x) & np.isfinite(y) & np.isfinite(z)
    x, y, z = x[valid], y[valid], z[valid]
    t1 = time.perf_counter()
    return x, y, z, (t1 - t0)


def corridor_check_timed(x, y, z, half_width=1.0, near=1.0, z_min=-1.6, z_max=1.0):
    t0 = time.perf_counter()
    fwd = -y
    mask = (fwd > near) & (np.abs(x) < half_width) & (z > z_min) & (z < z_max)
    d = fwd[mask]
    _ = d.min() if d.size else np.nan
    t1 = time.perf_counter()
    return t1 - t0


print(f'{"бег":<38} {"точек/кадр":>10} {"parse мс":>9} {"corridor мс":>12} {"rail-fit мс":>12} {"обзор FPS*":>11}')
print('-' * 100)

overall_parse, overall_corridor, overall_railfit, overall_n = [], [], [], []

for bag in BAGS:
    parse_times, corridor_times, railfit_times, npts = [], [], [], []
    with AnyReader([bag], default_typestore=ts) as reader:
        conn = reader.connections[0]
        for i, (connection, timestamp, rawdata) in enumerate(reader.messages(connections=[conn])):
            if i >= 3 + N_FRAMES:
                break
            x, y, z, dt_parse = load_xyz_timed(rawdata, connection.msgtype, reader)
            dt_corridor = corridor_check_timed(x, y, z)
            t0 = time.perf_counter()
            find_floor_bumps(x, y, z)
            dt_railfit = time.perf_counter() - t0
            if i >= 3:  # skip warmup frames (first-call overhead, cache effects)
                parse_times.append(dt_parse)
                corridor_times.append(dt_corridor)
                railfit_times.append(dt_railfit)
                npts.append(len(x))
            del x, y, z, rawdata

    p_ms = np.mean(parse_times) * 1000
    c_ms = np.mean(corridor_times) * 1000
    r_ms = np.mean(railfit_times) * 1000
    fps = 1000 / (p_ms + c_ms)  # FPS if only parse+corridor ran every frame (rail-fit run less often)
    print(f'{bag.name:<38} {int(np.mean(npts)):>10} {p_ms:>9.2f} {c_ms:>12.3f} {r_ms:>12.1f} {fps:>11.0f}')
    overall_parse.append(p_ms); overall_corridor.append(c_ms); overall_railfit.append(r_ms); overall_n.append(np.mean(npts))

print('-' * 100)
print(f'{"СРЕДНЕЕ по всем бегам":<38} {int(np.mean(overall_n)):>10} {np.mean(overall_parse):>9.2f} '
      f'{np.mean(overall_corridor):>12.3f} {np.mean(overall_railfit):>12.1f} '
      f'{1000/(np.mean(overall_parse)+np.mean(overall_corridor)):>11.0f}')
print()
print('* FPS-оценка предполагает, что corridor-check (проверка препятствия) считается каждый кадр,')
print('  а rail-fit (геометрия пути) -- нет, он не обязан пересчитываться на каждом кадре')
print('  (путь меняется медленно относительно частоты кадров лидара).')
