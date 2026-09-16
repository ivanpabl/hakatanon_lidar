"""The narrow-corridor approach keeps re-discovering tunnel-wall clutter, not the
real obstacle -- same failure mode as at the start of this whole investigation.
Instead, just look at several full, wide-view frames spaced through the sequence
and visually search for a compact cluster that doesn't belong to the smooth
wall/floor continuum."""
import numpy as np
from rosbags.highlevel import AnyReader
from rosbags.typesys import Stores, get_typestore
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

ts = get_typestore(Stores.ROS2_HUMBLE)
ROOT = Path('/home/pablo/Documents/hakaton/Датасет/archive/for_hackathon')
OUT = Path(__file__).parent / 'out'
OUT.mkdir(exist_ok=True)

BAG = ROOT / 'doubleT_obstacle'
FRAME_IDXS = [0, 40, 80, 120, 160, 200]

def load_xyz(msg):
    buf = np.frombuffer(msg.data, dtype=np.uint8).reshape(-1, msg.point_step)
    x = buf[:, 0:4].view(np.float32).ravel()
    y = buf[:, 4:8].view(np.float32).ravel()
    z = buf[:, 8:12].view(np.float32).ravel()
    valid = np.isfinite(x) & np.isfinite(y) & np.isfinite(z)
    return x[valid], y[valid], z[valid]

frames = {}
with AnyReader([BAG], default_typestore=ts) as reader:
    conn = reader.connections[0]
    wanted = set(FRAME_IDXS)
    for i, (connection, timestamp, rawdata) in enumerate(reader.messages(connections=[conn])):
        if i in wanted:
            msg = reader.deserialize(rawdata, connection.msgtype)
            frames[i] = load_xyz(msg)
            wanted.discard(i)
        if not wanted:
            break

fig, axes = plt.subplots(2, 3, figsize=(18, 11))
for ax, idx in zip(axes.ravel(), FRAME_IDXS):
    x, y, z = frames[idx]
    fwd = -y
    m = (fwd > 0) & (fwd < 100) & (np.abs(x) < 6)
    sc = ax.scatter(x[m], fwd[m], s=1, c=z[m], cmap='viridis', vmin=-2, vmax=2)
    ax.set_title(f'кадр {idx} (t≈{idx/9.86:.1f}с)')
    ax.set_xlabel('x, м'); ax.set_ylabel('вперёд, м')
    ax.invert_xaxis()

fig.tight_layout()
fig.savefig(OUT / 'doubleT_obstacle_wide_scan.png', dpi=130)
print('saved', OUT / 'doubleT_obstacle_wide_scan.png')
