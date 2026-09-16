"""One-off diagnostic: why does rail detection fail on doubleT_obstacle (Hesai128)
but work on doubleT_platform (Livox)? Compares the intermediate detection stages
side by side instead of guessing."""
import numpy as np
from rosbags.highlevel import AnyReader
from rosbags.typesys import Stores, get_typestore
from pathlib import Path

from rail_pathfit import find_floor_bumps, cluster_lines, GAUGE_TARGET, GAUGE_TOL

ts = get_typestore(Stores.ROS2_HUMBLE)
ROOT = Path('/home/pablo/Documents/hakaton/Датасет/archive/for_hackathon')
FRAME_IDX = 10

def load_xyz(msg):
    buf = np.frombuffer(msg.data, dtype=np.uint8).reshape(-1, msg.point_step)
    x = buf[:, 0:4].view(np.float32).ravel()
    y = buf[:, 4:8].view(np.float32).ravel()
    z = buf[:, 8:12].view(np.float32).ravel()
    valid = np.isfinite(x) & np.isfinite(y) & np.isfinite(z)
    return x[valid], y[valid], z[valid]

def get_frame(bag, idx):
    with AnyReader([bag], default_typestore=ts) as reader:
        conn = reader.connections[0]
        print(f'  topic={conn.topic}, msgtype={conn.msgtype}')
        for i, (connection, timestamp, rawdata) in enumerate(reader.messages(connections=[conn])):
            if i == idx:
                msg = reader.deserialize(rawdata, connection.msgtype)
                return load_xyz(msg)
    return None

for name in ['doubleT_platform', 'doubleT_obstacle']:
    print(f'\n=== {name} ===')
    bag = ROOT / name
    x, y, z = get_frame(bag, FRAME_IDX)
    print(f'  total points={len(x)}, z median={np.median(z):.2f}, '
          f'z 10/30/50 pct={np.percentile(z, [10,30,50]).round(2)}')

    pf, pl, ph = find_floor_bumps(x, y, z)
    print(f'  candidate bumps found: {len(pf)}')

    clusters = cluster_lines(pf, pl)
    print(f'  clusters formed: {len(clusters)}')
    for c in clusters:
        print(f'    x_center={c["x_center"]:+.3f}  n_points={len(c["lat"])}  '
              f'fwd_range=[{c["fwd"].min():.0f},{c["fwd"].max():.0f}]')

    if len(clusters) >= 2:
        print('  adjacent-cluster gaps (looking for ~1.52m +/- 0.40m):')
        for i in range(len(clusters) - 1):
            d = clusters[i+1]['x_center'] - clusters[i]['x_center']
            ok = abs(d - GAUGE_TARGET) < GAUGE_TOL
            print(f'    {clusters[i]["x_center"]:+.2f} <-> {clusters[i+1]["x_center"]:+.2f} : '
                  f'gap={d:.2f}m {"  <-- MATCHES GAUGE" if ok else ""}')
