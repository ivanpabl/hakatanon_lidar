"""Признаки правдоподобности объектов на выбранных окнах записи (для подбора тестов plausibility).

    python research/cand_dump.py --bag new_data --windows 0:300 2400:300 --tag nd --workers 4
    -> runs/cand_<tag>.jsonl: по строке на объект с треком (уровень, признаки, кадр, пробег)

Окно начинается с поиска по времени (кадр -> t_s из runs/alarms_<map>.csv), перед окном
WARMUP кадров прогрева детектора (не пишутся).
"""
import argparse
import csv
import json
import os
import sys
import warnings
from multiprocessing import Pool
from pathlib import Path

for _v in ('OMP_NUM_THREADS', 'OPENBLAS_NUM_THREADS', 'MKL_NUM_THREADS'):
    os.environ.setdefault(_v, '1')
ROOT = Path(__file__).resolve().parent.parent
sys.path[:0] = [str(ROOT / 'tools'), str(ROOT / 'core')]

from bags import RUNS, open_cloud_bag, safe_messages  # noqa: E402
from tunnel_od import ObstacleDetector, parse_pointcloud2  # noqa: E402
from tunnel_od.pointcloud import FrameRepeat  # noqa: E402

WARMUP = 30
SKIP = {'_idx'}


def run(job):
    bag, f0, n, t_start, det_kwargs = job
    warnings.simplefilter('ignore')
    out = []
    det, rep = ObstacleDetector(**det_kwargs), FrameRepeat()
    errors = []
    with open_cloud_bag(bag) as (r, c):
        t0 = r.start_time
        start = None if t_start is None else t0 + int(t_start * 1e9)
        first = f0 - WARMUP if start is not None else 0
        for j, (cc, t, raw) in enumerate(safe_messages(r.messages(connections=[c], start=start), errors)):
            i = first + j
            if i >= f0 + n:
                break
            m = r.deserialize(raw, cc.msgtype)
            if rep.check(m.data):
                continue
            res = det.detect(*parse_pointcloud2(m.data, m.point_step, m.fields), refit_path=True, stamp=t / 1e9)
            if i < f0:
                continue
            for o in res['objects']:
                if o.get('held') or 'track_id' not in o or o.get('edge_line'):
                    continue
                if not (o.get('level') or o.get('confirmed')):
                    continue
                rec = {k: (round(v, 3) if isinstance(v, float) else v) for k, v in o.items() if k not in SKIP}
                rec.update(bag=bag, frame=i, status=res['status'], travel_m=round(res['travel_m'], 2),
                           path_range_m=res['path_range_m'])
                out.append(rec)
    print(f'  {bag} {f0}:{n} объектов {len(out)}', flush=True)
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--bag', required=True)
    ap.add_argument('--windows', nargs='+', default=['0:0'], help='кадр:число кадров (0:0 -- вся запись)')
    ap.add_argument('--map', default='nd_final', help='runs/alarms_<map>.csv: кадр -> t_s для поиска')
    ap.add_argument('--tag', required=True)
    ap.add_argument('--det', default='{}')
    ap.add_argument('--workers', type=int, default=4)
    a = ap.parse_args()
    ts = {}
    mp = RUNS / f'alarms_{a.map}.csv'
    if mp.exists():
        for row in csv.DictReader(open(mp)):
            if row['bag'] == a.bag:
                ts[int(row['frame'])] = float(row['t_s'])
    jobs = []
    for w in a.windows:
        f0, n = map(int, w.split(':'))
        if f0 == 0:
            jobs.append((a.bag, 0, n or 10 ** 9, None, json.loads(a.det)))
        else:
            jobs.append((a.bag, f0, n, ts[f0 - WARMUP] - 0.05, json.loads(a.det)))
    with Pool(min(a.workers, len(jobs))) as p:
        rows = [x for part in p.imap_unordered(run, jobs) for x in part]
    path = RUNS / f'cand_{a.tag}.jsonl'
    with open(path, 'w') as f:
        for x in rows:
            f.write(json.dumps(x, ensure_ascii=False) + '\n')
    print(f'{len(rows)} -> {path}')


if __name__ == '__main__':
    main()
