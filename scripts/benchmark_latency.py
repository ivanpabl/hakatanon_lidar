"""Measure per-frame processing latency / FPS of our detection code on real bags.
No ROS2 / Docker needed for this -- it's a pure algorithmic timing benchmark,
directly relevant to ТЗ criterion 8.3 (speed / real-time performance)."""
import time
import warnings
import numpy as np
from rosbags.highlevel import AnyReader
from rosbags.typesys import Stores, get_typestore
from pathlib import Path

from detector_interface import ObstacleDetector, parse_pointcloud2

warnings.simplefilter('ignore', np.exceptions.RankWarning)
ts = get_typestore(Stores.ROS2_HUMBLE)
ROOT = Path(__file__).resolve().parent.parent / 'Датасет' / 'archive' / 'for_hackathon'
BAGS = sorted([p for p in ROOT.iterdir() if p.is_dir()])
N_FRAMES = 15  # frames to time per bag (after 3 warmup frames)


print(f'{"бег":<38} {"точек/кадр":>10} {"parse мс":>9} {"check мс":>9} {"refit мс":>9} {"detect FPS*":>12}')
print('-' * 95)

overall = []
for bag in BAGS:
    det = ObstacleDetector()
    t_parse, t_check, t_refit, npts = [], [], [], []
    with AnyReader([bag], default_typestore=ts) as reader:
        conn = reader.connections[0]
        for i, (connection, timestamp, rawdata) in enumerate(reader.messages(connections=[conn])):
            if i >= 3 + N_FRAMES:
                break
            msg = reader.deserialize(rawdata, connection.msgtype)
            t0 = time.perf_counter()
            x, y, z = parse_pointcloud2(msg.data, msg.point_step, msg.fields)
            t1 = time.perf_counter()
            det.update_path(x, y, z)
            t2 = time.perf_counter()
            det.check_frame(x, y, z)
            t3 = time.perf_counter()
            if i >= 3:  # skip warmup frames (first-call overhead, cache effects)
                t_parse.append(t1 - t0); t_refit.append(t2 - t1); t_check.append(t3 - t2)
                npts.append(len(x))

    p_ms, c_ms, r_ms = (np.mean(t) * 1000 for t in (t_parse, t_check, t_refit))
    fps = 1000 / (p_ms + c_ms + r_ms)  # путь пересчитывается на каждом кадре
    print(f'{bag.name:<38} {int(np.mean(npts)):>10} {p_ms:>9.1f} {c_ms:>9.1f} {r_ms:>9.1f} {fps:>12.0f}')
    overall.append((p_ms, c_ms, r_ms, np.mean(npts)))

p_ms, c_ms, r_ms, n = np.mean(overall, axis=0)
print('-' * 95)
print(f'{"СРЕДНЕЕ по всем бегам":<38} {int(n):>10} {p_ms:>9.1f} {c_ms:>9.1f} {r_ms:>9.1f} {1000/(p_ms+c_ms+r_ms):>12.0f}')
print()
print('* FPS при пересчёте пути (refit) на КАЖДОМ кадре; "точек/кадр" -- после очистки')
print('  от нулей и дублей dual return в parse_pointcloud2.')
