"""Look for the actual physical obstacle in doubleT_obstacle (the one real bag that
has one) and estimate the real distance at which it becomes visible/distinguishable
from tunnel clutter -- something we have never actually measured, only assumed."""
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
NEAR_CUTOFF = 3.0
HALF_WIDTH = 1.2
Z_MIN, Z_MAX = -1.6, 2.0  # "air" band above floor/rail clutter, below ceiling

def load_xyz(msg):
    buf = np.frombuffer(msg.data, dtype=np.uint8).reshape(-1, msg.point_step)
    x = buf[:, 0:4].view(np.float32).ravel()
    y = buf[:, 4:8].view(np.float32).ravel()
    z = buf[:, 8:12].view(np.float32).ravel()
    valid = np.isfinite(x) & np.isfinite(y) & np.isfinite(z)
    return x[valid], y[valid], z[valid]

times, dists, lat_at_min, n_in_corridor = [], [], [], []
closest_frame = {'d': np.inf, 'i': None, 'xyz': None}

with AnyReader([BAG], default_typestore=ts) as reader:
    conn = reader.connections[0]
    t0 = None
    for i, (connection, timestamp, rawdata) in enumerate(reader.messages(connections=[conn])):
        msg = reader.deserialize(rawdata, connection.msgtype)
        x, y, z = load_xyz(msg)
        if t0 is None:
            t0 = timestamp
        times.append((timestamp - t0) / 1e9)
        fwd = -y
        m = (fwd > NEAR_CUTOFF) & (np.abs(x) < HALF_WIDTH) & (z > Z_MIN) & (z < Z_MAX)
        d = fwd[m]
        n_in_corridor.append(int(m.sum()))
        if d.size:
            dists.append(d.min())
            lat_at_min.append(x[m][np.argmin(d)])
            if d.min() < closest_frame['d']:
                closest_frame = {'d': d.min(), 'i': i, 'xyz': (x[m].copy(), y[m].copy(), z[m].copy())}
        else:
            dists.append(np.nan)
            lat_at_min.append(np.nan)
        del x, y, z, msg, rawdata

times = np.array(times)
dists = np.array(dists)
n_in_corridor = np.array(n_in_corridor)

finite = np.isfinite(dists)
print(f'Кадров всего: {len(times)}, из них с точками в коридоре (|x|<{HALF_WIDTH}м, near>{NEAR_CUTOFF}м): {finite.sum()}')
if finite.sum() > 5:
    # trend: linear fit distance vs time on the frames that have data
    tt, dd = times[finite], dists[finite]
    slope, intercept = np.polyfit(tt, dd, 1)
    resid = dd - (slope * tt + intercept)
    print(f'Тренд дистанции: {slope:.2f} м/с (наклон), разброс вокруг тренда (std)={resid.std():.2f}м')
    print(f'Минимальная дистанция за всю запись: {np.nanmin(dists):.1f}м на кадре {closest_frame["i"]} '
          f'(t={times[closest_frame["i"]]:.1f}с)')
    print(f'Дистанция в начале записи: {dd[0]:.1f}м, в конце: {dd[-1]:.1f}м')
else:
    print('Слишком мало кадров с точками в этом коридоре -- окно поиска не подходит для этого бега.')

fig, axes = plt.subplots(3, 1, figsize=(10, 10))
axes[0].plot(times, dists, '.-')
axes[0].set_ylabel('мин. дистанция в коридоре, м')
axes[0].set_title(f'doubleT_obstacle: |x|<{HALF_WIDTH}м, near>{NEAR_CUTOFF}м, z в [{Z_MIN},{Z_MAX}]')
axes[0].grid(alpha=0.3)

axes[1].plot(times, n_in_corridor, '.-', c='orange')
axes[1].set_ylabel('точек в коридоре')
axes[1].grid(alpha=0.3)

axes[2].plot(times, lat_at_min, '.-', c='green')
axes[2].set_ylabel('x ближайшей точки, м')
axes[2].set_xlabel('время, с')
axes[2].grid(alpha=0.3)

fig.tight_layout()
fig.savefig(OUT / 'doubleT_obstacle_search.png', dpi=130)
print('saved', OUT / 'doubleT_obstacle_search.png')

if closest_frame['i'] is not None:
    xo, yo, zo = closest_frame['xyz']
    fig2, ax2 = plt.subplots(figsize=(6, 6))
    sc = ax2.scatter(xo, -yo, c=zo, cmap='viridis', s=20)
    ax2.set_title(f'Ближайший кластер в коридоре, кадр {closest_frame["i"]}, d={closest_frame["d"]:.1f}м')
    ax2.set_xlabel('x, м'); ax2.set_ylabel('вперёд, м')
    plt.colorbar(sc, ax=ax2, label='z, высота')
    fig2.savefig(OUT / 'doubleT_obstacle_closest_cluster.png', dpi=130)
    print('saved', OUT / 'doubleT_obstacle_closest_cluster.png')
