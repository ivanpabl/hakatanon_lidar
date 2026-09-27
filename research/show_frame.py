"""Картинка кадра записи с решением детектора: python research/show_frame.py <bag> <кадр> '<json kwargs>' <png>"""
import json, sys, warnings
sys.path.insert(0, 'tools')
warnings.simplefilter('ignore')
from tunnel_od import ObstacleDetector, parse_pointcloud2
from bags import open_cloud_bag
from viz.render import render_frame

bag, frame, kw, out = sys.argv[1], int(sys.argv[2]), json.loads(sys.argv[3]), sys.argv[4]
det = ObstacleDetector(**kw)
with open_cloud_bag(bag) as (r, c):
    for i, (cc, t, raw) in enumerate(r.messages(connections=[c])):
        if i < frame - 15:
            continue
        m = r.deserialize(raw, cc.msgtype)
        xyz = parse_pointcloud2(m.data, m.point_step, m.fields)
        res = det.detect(*xyz, refit_path=True, stamp=t / 1e9)
        if i == frame:
            break
objs = [o for o in res['objects'] if o['confirmed']]
print({k: res[k] for k in ('obstacle', 'distance_m', 'path_range_m')})
for o in objs:
    print('  ', {k: (round(v, 2) if isinstance(v, float) else v) for k, v in o.items()})
render_frame(out, *xyz, det.track_path(), f'{bag} #{frame}: ' + ', '.join(f"{o['distance_m']:.0f}м/{o['lateral_m']:+.2f}" for o in objs))
