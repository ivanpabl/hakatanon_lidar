"""Дальность известной оси пути по кадрам: с дальним продлением и без (каждый 5-й кадр)."""
import sys, warnings
sys.path.insert(0, 'tools')
import numpy as np
from multiprocessing import Pool
from tunnel_od import ObstacleDetector, parse_pointcloud2
import tunnel_od.detection.detector as D
from bags import open_cloud_bag
BAGS = ['doubleT_platform', 'roundT_doubleT', 'roundT_pressureGate_roundT',
        'roundT_squareT_pressureGate_squareT', 'squareT_platform_squareT_switch']


def run(bag):
    warnings.simplefilter('ignore')
    orig = D.extend_path_by_walls
    out = {False: [], True: []}
    with open_cloud_bag(bag) as (r, c):
        for i, (cc, t, raw) in enumerate(r.messages(connections=[c])):
            if i % 5:
                continue
            m = r.deserialize(raw, cc.msgtype)
            xyz = parse_pointcloud2(m.data, m.point_step, m.fields)
            for fe in (False, True):
                D.extend_path_by_walls = lambda *a, fe=fe, **k: orig(*a, far_extend=fe, **k)
                det = ObstacleDetector()
                det.update_path(*xyz)
                out[fe].append(det.path_range or 0)
    D.extend_path_by_walls = orig
    return bag, out


if __name__ == '__main__':
    with Pool(3) as p:
        for bag, out in p.imap(run, BAGS):
            a, b = np.array(out[False]), np.array(out[True])
            print(f'{bag:<38} p10/50/90 без: {np.percentile(a, [10, 50, 90]).round()}  с дальним: {np.percentile(b, [10, 50, 90]).round()}', flush=True)
