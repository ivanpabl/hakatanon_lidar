"""Время шагов конвейера без профилировщика (медиана по кадрам)."""
import sys, time, warnings
sys.path.insert(0, 'tools'); warnings.simplefilter('ignore')
import numpy as np
from tunnel_od import ObstacleDetector, parse_pointcloud2
from tunnel_od.geometry import rails, path, bed
from bags import open_cloud_bag

bag = sys.argv[1] if len(sys.argv) > 1 else 'roundT_doubleT'
frames = []
with open_cloud_bag(bag) as (r, c):
    for i, (cc, t, raw) in enumerate(r.messages(connections=[c])):
        if i >= 40: break
        m = r.deserialize(raw, cc.msgtype)
        frames.append((m.data, m.point_step, m.fields, t / 1e9))
T = {}
def tm(name, f, *a, **k):
    t0 = time.perf_counter(); out = f(*a, **k); T.setdefault(name, []).append(1000 * (time.perf_counter() - t0)); return out
det = ObstacleDetector()
for data, step, fields, t in frames:
    x, y, z = tm('parse', parse_pointcloud2, data, step, fields)
    tm('find_floor_bumps', rails.find_floor_bumps, x, y, z)
    tm('update_path', det.update_path, x, y, z)
    tm('check_frame', det.check_frame, x, y, z, stamp=t)
for k, v in T.items():
    print(f'{k:<18} p50 {np.median(v[5:]):6.1f} мс  p95 {np.percentile(v[5:], 95):6.1f} мс')
