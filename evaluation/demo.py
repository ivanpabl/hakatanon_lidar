"""Демонстрация: HTML-плеер (один файл, без сети) -- детектор по кадрам реальных записей в 3D.

    python -m evaluation demo                                        # -> output/report/demo.html
    python -m evaluation demo --clip doubleT_obstacle --clip roundT_doubleT:0:150
    python -m evaluation demo --bg-points 10000 --out /tmp/demo.html   # файл меньше

Отрывок -- запись[:первый кадр[:число кадров]]. Детектор идёт с первого кадра отрывка и пересчитывает
путь в каждом кадре, как офлайн-инструменты (evaluation/alarms.py). В плеере на каждом кадре: облако,
ось пути, коридор, объекты, решение и дистанция, время разбора и проверки кадра на этой машине.
Точки зоны и объектов сохраняются все, фона -- не больше --bg-points на кадр (размер файла).
"""
import argparse
import base64
import json
import os
for _v in ('OMP_NUM_THREADS', 'OPENBLAS_NUM_THREADS', 'MKL_NUM_THREADS'):
    os.environ.setdefault(_v, '1')
import time
import warnings
from datetime import date
from multiprocessing import Pool
from pathlib import Path

import numpy as np

from bags import ROOT, open_cloud_bag
from dashboard import frame_view, git_rev, host

TEMPLATE = Path(__file__).resolve().parent / 'templates' / 'demo.html'
OUT = ROOT / 'output' / 'report' / 'demo.html'
CLIPS = ['doubleT_obstacle', 'roundT_doubleT:0:150']


def parse_clip(spec):
    bag, *rest = spec.split(':')
    start = int(rest[0]) if rest and rest[0] else 0
    count = int(rest[1]) if len(rest) > 1 and rest[1] else None
    return bag, start, count


def run_clip(job):
    spec, bg_points, max_fwd = job
    from tunnel_od import ObstacleDetector, parse_pointcloud2
    warnings.simplefilter('ignore')
    bag, start, count = parse_clip(spec)
    det = ObstacleDetector()
    rng = np.random.default_rng(0)
    frames, pts, tags, offs = [], [], [], [0]
    with open_cloud_bag(bag) as (reader, conn):
        t0 = None
        for i, (c, t, raw) in enumerate(reader.messages(connections=[conn])):
            if i < start:
                continue
            if count is not None and i >= start + count:
                break
            m = reader.deserialize(raw, c.msgtype)
            a = time.perf_counter()
            x, y, z = parse_pointcloud2(m.data, m.point_step, m.fields)
            b = time.perf_counter()
            res = det.detect(x, y, z, refit_path=True, stamp=t / 1e9)
            e = time.perf_counter()
            v = frame_view(x, y, z, res, det.track_path(), max_fwd=max_fwd, bg_points=bg_points, rng=rng)
            t0 = t if t0 is None else t0
            pts.append(v['q'])
            tags.append(v['tag'])
            offs.append(offs[-1] + len(v['q']))
            frames.append({'i': i, 't': round((t - t0) / 1e9, 2), 'obstacle': bool(res['obstacle']),
                           'distance_m': res['distance_m'], 'path_range_m': v['path_range_m'],
                           'rails_to_m': v['rails_to_m'], 'axis': [[round(p, 2) for p in r] for r in v['axis']],
                           'objects': v['objects'], 'n_zone': int(res['n_points']),
                           'parse_ms': round((b - a) * 1e3, 1), 'check_ms': round((e - b) * 1e3, 1)})
            if len(frames) % 50 == 0:
                print(f'  {bag}: {len(frames)} кадров', flush=True)
    if not frames:
        raise SystemExit(f'{spec}: нет кадров')
    return {'bag': bag, 'start': start, 'frames': frames, 'offs': offs,
            'points': base64.b64encode(np.concatenate(pts).tobytes()).decode(),
            'tags': base64.b64encode(np.concatenate(tags).tobytes()).decode()}


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--clip', action='append', help=f'запись[:первый кадр[:число кадров]], по умолчанию {CLIPS}')
    ap.add_argument('--bg-points', type=int, default=14000, help='точек фона на кадр (остальные отбрасываются)')
    ap.add_argument('--max-fwd', type=float, default=200.0, help='показывать облако до этой дальности, м')
    ap.add_argument('--workers', type=int, default=2)
    ap.add_argument('--out', type=Path, default=OUT)
    args = ap.parse_args()
    clips = args.clip or CLIPS

    with Pool(max(1, min(len(clips), args.workers))) as pool:
        data = pool.map(run_clip, [(c, args.bg_points, args.max_fwd) for c in clips])
    for c in data:
        f = c['frames']
        alarm = sum(x['obstacle'] for x in f)
        ms = np.array([x['parse_ms'] + x['check_ms'] for x in f])
        print(f'{c["bag"]}: {len(f)} кадров, с тревогой {alarm}, разбор+проверка p50 {np.median(ms):.0f} мс')

    page = {'generated': date.today().isoformat(), 'commit': git_rev(), 'host': host(),
            'zone': {'half_width': 1.0, 'clearance': 0.15, 'height': 2.0}, 'clips': data}
    html = TEMPLATE.read_text(encoding='utf-8').replace(
        '/*__DATA__*/null', json.dumps(page, ensure_ascii=False, separators=(',', ':')))
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(html, encoding='utf-8')
    print(f'{args.out}  {args.out.stat().st_size / 1e6:.1f} МБ')


if __name__ == '__main__':
    main()
