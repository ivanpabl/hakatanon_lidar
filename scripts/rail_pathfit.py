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

BAGS = sorted([p for p in ROOT.iterdir() if p.is_dir()])
FRAME_IDX = 10
NEAR, FAR = 2.0, 40.0
HALF_WIDTH = 3.5
LAT_BIN = 0.05
FWD_BIN = 1.0
PROMINENCE = 0.05
GAUGE_TARGET = 1.52
GAUGE_TOL = 0.40  # accept 1.12 - 1.92 m
CLUSTER_GAP = 0.25  # meters, gap to split clusters when sorting lateral positions

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
        for i, (connection, timestamp, rawdata) in enumerate(reader.messages(connections=[conn])):
            if i == idx:
                msg = reader.deserialize(rawdata, connection.msgtype)
                return load_xyz(msg)
    return None

def find_floor_bumps(x, y, z):
    fwd = -y
    m = (fwd > NEAR) & (fwd < FAR) & (np.abs(x) < HALF_WIDTH)
    xf, zf, fwdf = x[m], z[m], fwd[m]
    if len(xf) < 100:
        return np.array([]), np.array([]), np.array([])
    peaks_fwd, peaks_lat, peaks_h = [], [], []
    for f0 in np.arange(NEAR, FAR, FWD_BIN):
        sel = (fwdf >= f0) & (fwdf < f0 + FWD_BIN)
        if sel.sum() < 50:
            continue
        xs, zs = xf[sel], zf[sel]
        zlo = np.percentile(zs, 30)
        floor_sel = zs < zlo + 0.9
        xs, zs = xs[floor_sel], zs[floor_sel]
        if len(xs) < 30:
            continue
        lat_edges = np.arange(-HALF_WIDTH, HALF_WIDTH + LAT_BIN, LAT_BIN)
        idx_bin = np.digitize(xs, lat_edges)
        n_bins = len(lat_edges) + 1
        profile = np.full(n_bins, np.nan)
        for b in np.unique(idx_bin):
            profile[b] = np.max(zs[idx_bin == b])
        centers = np.concatenate([[lat_edges[0] - LAT_BIN], lat_edges])
        valid = ~np.isnan(profile)
        if valid.sum() < 15:
            continue
        w = 10
        prof_filled = np.where(valid, profile, np.nan)
        baseline = np.full(n_bins, np.nan)
        for i in range(n_bins):
            lo, hi = max(0, i - w), min(n_bins, i + w + 1)
            window = prof_filled[lo:hi]
            if np.any(~np.isnan(window)):
                baseline[i] = np.nanmedian(window)
        prominence = profile - baseline
        is_peak = valid & (prominence > PROMINENCE)
        for i in np.where(is_peak)[0]:
            lo, hi = max(0, i - 3), min(n_bins, i + 4)
            if profile[i] >= np.nanmax(prof_filled[lo:hi]):
                peaks_fwd.append((f0 + f0 + FWD_BIN) / 2)
                peaks_lat.append(centers[i])
                peaks_h.append(prominence[i])
    return np.array(peaks_fwd), np.array(peaks_lat), np.array(peaks_h)

def cluster_lines(pf, pl):
    order = np.argsort(pl)
    pl_s, pf_s = pl[order], pf[order]
    clusters = []
    cur_idx = [0]
    for i in range(1, len(pl_s)):
        if pl_s[i] - pl_s[i - 1] > CLUSTER_GAP:
            clusters.append(cur_idx)
            cur_idx = []
        cur_idx.append(i)
    clusters.append(cur_idx)
    result = []
    for idxs in clusters:
        if len(idxs) < 5:
            continue
        result.append({'x_center': np.median(pl_s[idxs]), 'fwd': pf_s[idxs], 'lat': pl_s[idxs]})
    result.sort(key=lambda c: c['x_center'])
    return result

def pick_rail_pair(clusters):
    best = None
    for i in range(len(clusters) - 1):
        d = clusters[i + 1]['x_center'] - clusters[i]['x_center']
        if abs(d - GAUGE_TARGET) < GAUGE_TOL:
            mid = (clusters[i + 1]['x_center'] + clusters[i]['x_center']) / 2
            score = abs(mid)  # prefer pair straddling sensor x=0 (own track)
            if best is None or score < best[0]:
                best = (score, clusters[i], clusters[i + 1], d)
    return best

def fit_path(x, y, z):
    """Run the full rail-pair detection + quadratic centerline fit on one frame.
    Returns (pair_or_None, fit_fwd, centerline) -- centerline is None if no pair found."""
    pf, pl, ph = find_floor_bumps(x, y, z)
    clusters = cluster_lines(pf, pl)
    pair = pick_rail_pair(clusters)
    if pair is None:
        return None, None, None
    _, left, right, gauge = pair
    pl_left = np.polyfit(left['fwd'], left['lat'], 2)
    pl_right = np.polyfit(right['fwd'], right['lat'], 2)
    fit_fwd = np.linspace(max(left['fwd'].min(), right['fwd'].min()),
                           min(left['fwd'].max(), right['fwd'].max()), 100)
    cl = (np.polyval(pl_left, fit_fwd) + np.polyval(pl_right, fit_fwd)) / 2
    return pair, fit_fwd, cl


if __name__ == '__main__':
    fig, axes = plt.subplots(2, 3, figsize=(18, 11))
    report = []
    for ax, bag in zip(axes.ravel(), BAGS):
        res = get_frame(bag, FRAME_IDX)
        x, y, z = res
        fwd_all = -y
        m = (fwd_all > 0) & (fwd_all < FAR) & (np.abs(x) < HALF_WIDTH + 0.5)
        ax.scatter(x[m], fwd_all[m], s=0.2, c='lightgray', alpha=0.5)

        pf, pl, ph = find_floor_bumps(x, y, z)
        clusters = cluster_lines(pf, pl)
        for c in clusters:
            ax.scatter(c['lat'], c['fwd'], s=8, c='silver')

        pair, fit_fwd, cl = fit_path(x, y, z)
        if pair is None:
            report.append((bag.name, None))
            ax.set_title(f'{bag.name}: пара рельсов НЕ найдена')
        else:
            _, left, right, gauge = pair
            ax.scatter(left['lat'], left['fwd'], s=20, c='red', label='rail A')
            ax.scatter(right['lat'], right['fwd'], s=20, c='blue', label='rail B')
            ax.plot(cl, fit_fwd, 'k--', lw=2, label='центральная линия пути')
            drift = cl[-1] - cl[0]
            report.append((bag.name, gauge, drift, fit_fwd[-1] - fit_fwd[0]))
            ax.set_title(f'{bag.name}: колея={gauge:.2f}м, снос={drift:+.2f}м/{fit_fwd[-1]-fit_fwd[0]:.0f}м')
            ax.legend(fontsize=7)
        ax.set_xlabel('x, м'); ax.set_ylabel('вперёд (-y), м')
        ax.set_xlim(-HALF_WIDTH - 0.5, HALF_WIDTH + 0.5)
        ax.invert_xaxis()

    fig.tight_layout()
    fig.savefig(OUT / 'rail_path_final.png', dpi=130)
    print('saved', OUT / 'rail_path_final.png')
    print()
    for r in report:
        print(r)
