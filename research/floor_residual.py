"""Шум остатка до фона (развёртка вдоль оси) у полотна на пустых записях: можно ли
искать низкие предметы (300x300x100мм) ниже головки рельса."""
import sys, warnings
sys.path.insert(0, 'tools')
import numpy as np
from multiprocessing import Pool
from tunnel_od import ObstacleDetector, parse_pointcloud2
from tunnel_od.detection import background
from bags import open_cloud_bag
BAGS = ['doubleT_platform', 'roundT_doubleT', 'roundT_pressureGate_roundT',
        'roundT_squareT_pressureGate_squareT', 'squareT_platform_squareT_switch']
BANDS = [(3, 20), (20, 40), (40, 60), (60, 90)]


def run(bag):
    warnings.simplefilter('ignore')
    det = ObstacleDetector()
    out = {b: [] for b in BANDS}
    with open_cloud_bag(bag) as (r, c):
        for i, (cc, t, raw) in enumerate(r.messages(connections=[c])):
            m = r.deserialize(raw, cc.msgtype)
            x, y, z = parse_pointcloud2(m.data, m.point_step, m.fields)
            det.detect(x, y, z, refit_path=(i % 5 == 0))
            if i % 10 or det._rail_prof is None:
                continue
            fwd = -y
            lat = x - det._center_of_fwd(fwd)
            zr = z - det._tor_at(fwd, det._bed)
            q = (np.abs(lat) < 0.65) & (zr > -0.3) & (zr < 0.35) & (fwd > 3) & (fwd < 90)
            res = background.background_residual(fwd, lat, zr, q, 250.0)
            for b in BANDS:
                k = q & (fwd >= b[0]) & (fwd < b[1]) & np.isfinite(res)
                # максимум остатка по ячейкам 1м x 0.3м -- то, что увидел бы детектор
                if k.sum():
                    cell = np.floor(fwd[k]).astype(int) * 10 + np.floor((lat[k] + 0.65) / 0.3).astype(int)
                    mx = {}
                    for cc_, rr in zip(cell, res[k]):
                        mx[cc_] = max(mx.get(cc_, -9), rr)
                    out[b] += list(mx.values())
    return bag, out


if __name__ == '__main__':
    tot = {b: [] for b in BANDS}
    with Pool(2) as p:
        for bag, out in p.imap(run, BAGS):
            for b in BANDS:
                tot[b] += out[b]
    for b in BANDS:
        a = np.array(tot[b])
        print(f'{b}: ячеек {len(a)}, max-остаток p50/p99/p99.9 = {np.percentile(a, [50, 99, 99.9]).round(3)}, '
              f'доля >0.08 {np.mean(a > 0.08):.4f}, >0.12 {np.mean(a > 0.12):.4f}, >0.2 {np.mean(a > 0.2):.4f}')
