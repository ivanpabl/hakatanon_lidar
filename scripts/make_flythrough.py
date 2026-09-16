import sys
import numpy as np
from rosbags.highlevel import AnyReader
from rosbags.typesys import Stores, get_typestore
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.animation import PillowWriter

ts = get_typestore(Stores.ROS2_HUMBLE)
ROOT = Path('/home/pablo/Documents/hakaton/Датасет/archive/for_hackathon')
OUT = Path(__file__).parent / 'out'
OUT.mkdir(exist_ok=True)

BAG_NAME = sys.argv[1] if len(sys.argv) > 1 else 'doubleT_obstacle'
STEP = 2            # render every 2nd frame (keeps gif size/time reasonable)
MAX_POINTS = 30000  # subsample points per frame for speed
FPS = 8
FAR = 60

def load_xyz(msg):
    buf = np.frombuffer(msg.data, dtype=np.uint8).reshape(-1, msg.point_step)
    x = buf[:, 0:4].view(np.float32).ravel()
    y = buf[:, 4:8].view(np.float32).ravel()
    z = buf[:, 8:12].view(np.float32).ravel()
    valid = np.isfinite(x) & np.isfinite(y) & np.isfinite(z)
    return x[valid], y[valid], z[valid]

bag = ROOT / BAG_NAME
rng = np.random.default_rng(0)

fig, ax = plt.subplots(figsize=(6, 8))
sc = ax.scatter([], [], s=1.5, c=[], cmap='viridis', vmin=-2, vmax=2)
ax.set_xlim(-6, 6)
ax.set_ylim(0, FAR)
ax.invert_xaxis()
ax.set_xlabel('x, м (вбок)')
ax.set_ylabel('вперёд, м')
title = ax.set_title('')

writer = PillowWriter(fps=FPS)
out_path = OUT / f'{BAG_NAME}_flythrough.gif'

n_rendered = 0
with writer.saving(fig, out_path, dpi=100):
    with AnyReader([bag], default_typestore=ts) as reader:
        conn = reader.connections[0]
        for i, (connection, timestamp, rawdata) in enumerate(reader.messages(connections=[conn])):
            if i % STEP != 0:
                del rawdata
                continue
            msg = reader.deserialize(rawdata, connection.msgtype)
            x, y, z = load_xyz(msg)
            fwd = -y
            m = (fwd > 0) & (fwd < FAR) & (np.abs(x) < 6)
            xs, fs, zs = x[m], fwd[m], z[m]
            if len(xs) > MAX_POINTS:
                idx = rng.choice(len(xs), MAX_POINTS, replace=False)
                xs, fs, zs = xs[idx], fs[idx], zs[idx]
            sc.set_offsets(np.column_stack([xs, fs]))
            sc.set_array(zs)
            title.set_text(f'{BAG_NAME}  кадр {i}')
            writer.grab_frame()
            n_rendered += 1
            del msg, rawdata, x, y, z, xs, fs, zs

print(f'rendered {n_rendered} frames -> {out_path}')
