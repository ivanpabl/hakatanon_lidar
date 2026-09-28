"""Демонстрация: HTML-плеер (один файл, без сети) -- детектор по кадрам реальных записей в 3D.

    python -m evaluation demo                                        # -> output/report/demo.html
    python -m evaluation demo --clip doubleT_obstacle --clip roundT_doubleT:0:150
    python -m evaluation demo --approach roundT_squareT_pressureGate_squareT:30:cube:130
    python -m evaluation demo --bg-points 10000 --out /tmp/demo.html   # файл меньше

Подъезд (--approach запись:первый кадр:фигура:дальность) -- как evaluation/eval_approach.py:
синтетический объект неподвижен в мире на заданной дальности, поезд к нему едет. Объект
вставляется лучами датчика (tunnel_od.sim.inject), дистанция в каждом кадре уменьшается на
путь поезда по сцене. Отрывок идёт, пока объект не ближе 8 м. В плеере видны истинная
дистанция, скорость поезда и момент первого подтверждения. Фигуры: person, lying, cube.

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
from dashboard import frame_view, git_rev, host, zone_params
from approach_trace import category
from eval_injection import front_of

TEMPLATE = Path(__file__).resolve().parent / 'templates' / 'demo.html'
OUT = ROOT / 'output' / 'report' / 'demo.html'
CLIPS = ['doubleT_obstacle', 'roundT_doubleT:0:150']
APPROACH = ['roundT_squareT_pressureGate_squareT:30:person:170']
SHAPES = {'person': ('человек стоит', 'cylinder', (0.25, 1.7)),
          'lying': ('человек лежит', 'box', (1.7, 0.5, 0.3)),
          'cube': ('куб 0,4 м', 'box', (0.4, 0.4, 0.4))}
WARMUP = 20
MIN_D = 8.0


def parse_clip(spec):
    bag, *rest = spec.split(':')
    start = int(rest[0]) if rest and rest[0] else 0
    count = int(rest[1]) if len(rest) > 1 and rest[1] else None
    return bag, start, count


def parse_approach(spec):
    bag, start, shape, d0 = spec.split(':')
    if shape not in SHAPES:
        raise SystemExit(f'{spec}: фигура {shape}, есть {", ".join(SHAPES)}')
    return bag, int(start), shape, float(d0)


def truth_box(kind, dims, d, lat, z0):
    """Габарит вставленной фигуры: [вбок от, до, высота от, до, вперёд от, до] -- как box объекта."""
    if kind == 'cylinder':
        r, h = dims
        return [lat - r, lat + r, z0, z0 + h, d - r, d + r]
    ln, w, h = dims
    return [lat - w / 2, lat + w / 2, z0, z0 + h, d - ln / 2, d + ln / 2]


def run_approach(job):
    spec, bg_points, max_fwd = job
    from tunnel_od import ObstacleDetector, parse_pointcloud2
    from tunnel_od.pointcloud import beam_directions, xyz_views
    from tunnel_od.sim.inject import inject, on_track
    from tunnel_od.sim.shapes import make_shape
    warnings.simplefilter('ignore')
    bag, start, shape, d0 = parse_approach(spec)
    label, kind, dims = SHAPES[shape]
    det, ref = ObstacleDetector(), ObstacleDetector()
    rng = np.random.default_rng(0)
    frames, pts, tags, offs = [], [], [], [0]
    travelled, first = 0.0, None
    with open_cloud_bag(bag) as (reader, conn):
        t0 = None
        for i, (c, t, raw) in enumerate(reader.messages(connections=[conn])):
            if i < start - WARMUP:
                continue
            m = reader.deserialize(raw, c.msgtype)
            clean = parse_pointcloud2(m.data, m.point_step, m.fields)
            r = ref.detect(*clean, refit_path=True, stamp=t / 1e9)
            if i < start:
                det.detect(*clean, refit_path=True, stamp=t / 1e9)
                continue
            if frames:
                travelled += r['displacement_m'] or 0.0
            d = d0 - travelled
            if d < MIN_D:
                break
            lat, z0 = on_track(ref.track_path(), d)
            v = xyz_views(np.frombuffer(m.data, np.uint8).reshape(-1, m.point_step), m.fields)
            data, info = inject(m.data, m.point_step, [make_shape(kind, dims, d, lat, z0)], m.fields,
                                dirs=beam_directions(v['x'], v['y'], v['z']), rng=rng)
            a = time.perf_counter()
            x, y, z = parse_pointcloud2(data, m.point_step, m.fields)
            b = time.perf_counter()
            res = det.detect(x, y, z, refit_path=True, stamp=t / 1e9)
            e = time.perf_counter()
            fv = frame_view(x, y, z, res, det.track_path(), max_fwd=max_fwd, bg_points=bg_points, rng=rng)
            t0 = t if t0 is None else t0
            front = front_of(kind, dims, d)
            if res['obstacle'] and first is None:
                first = len(frames)
            pts.append(fv['q'])
            tags.append(fv['tag'])
            offs.append(offs[-1] + len(fv['q']))
            frames.append({'i': i, 't': round((t - t0) / 1e9, 2), 'obstacle': bool(res['obstacle']),
                           'distance_m': res['distance_m'], 'status': res['status'], 'clear_to_m': res['clear_to_m'],
                           'caution_distance_m': res['caution_distance_m'], 'path_range_m': fv['path_range_m'],
                           'rails_to_m': fv['rails_to_m'], 'axis': [[round(p, 2) for p in q] for q in fv['axis']],
                           'objects': fv['objects'], 'n_zone': int(res['n_points']),
                           'parse_ms': round((b - a) * 1e3, 1), 'check_ms': round((e - b) * 1e3, 1),
                           'speed_mps': None if r['speed_mps'] is None else round(r['speed_mps'], 1),
                           'travelled_m': round(travelled, 1), 'truth_m': round(front, 2), 'truth_cat': category(res, front)[0],
                           'truth_points': int(info['points_on_object']),
                           'truth_box': [round(p, 2) for p in truth_box(kind, dims, d, lat, z0)]})
            if len(frames) % 50 == 0:
                print(f'  {bag} подъезд: {len(frames)} кадров, объект на {d:.0f} м', flush=True)
    if not frames:
        raise SystemExit(f'{spec}: нет кадров')
    return {'bag': bag, 'start': start, 'frames': frames, 'offs': offs,
            'approach': {'shape': label, 'start_m': d0, 'first_frame': first,
                         'first_m': frames[first]['truth_m'] if first is not None else None},
            'points': base64.b64encode(np.concatenate(pts).tobytes()).decode(),
            'tags': base64.b64encode(np.concatenate(tags).tobytes()).decode()}


def run_job(job):
    return run_approach(job[1:]) if job[0] == 'approach' else run_clip(job[1:])


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
                           'distance_m': res['distance_m'], 'status': res['status'], 'clear_to_m': res['clear_to_m'],
                           'caution_distance_m': res['caution_distance_m'], 'path_range_m': v['path_range_m'],
                           'rails_to_m': v['rails_to_m'], 'axis': [[round(p, 2) for p in r] for r in v['axis']],
                           'objects': v['objects'], 'n_zone': int(res['n_points']),
                           'parse_ms': round((b - a) * 1e3, 1), 'check_ms': round((e - b) * 1e3, 1),
                           'speed_mps': None if res['speed_mps'] is None else round(res['speed_mps'], 1)})
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
    ap.add_argument('--approach', action='append',
                    help=f'подъезд: запись:первый кадр:фигура ({"|".join(SHAPES)}):дальность, м; по умолчанию {APPROACH}')
    ap.add_argument('--no-approach', action='store_true', help='без отрывка с подъездом')
    ap.add_argument('--bg-points', type=int, default=14000, help='точек фона на кадр (остальные отбрасываются)')
    ap.add_argument('--max-fwd', type=float, default=200.0, help='показывать облако до этой дальности, м')
    ap.add_argument('--workers', type=int, default=2)
    ap.add_argument('--out', type=Path, default=OUT)
    args = ap.parse_args()
    clips = args.clip or CLIPS
    approach = [] if args.no_approach else (args.approach or APPROACH)
    jobs = [('approach', a, args.bg_points, args.max_fwd) for a in approach] + \
           [('clip', c, args.bg_points, args.max_fwd) for c in clips]

    with Pool(max(1, min(len(jobs), args.workers))) as pool:
        data = pool.map(run_job, jobs)
    for c in data:
        f = c['frames']
        alarm = sum(x['obstacle'] for x in f)
        ms = np.array([x['parse_ms'] + x['check_ms'] for x in f])
        extra = ''
        if 'approach' in c:
            a = c['approach']
            extra = (f', {a["shape"]} со {a["start_m"]:.0f} м: первое подтверждение '
                     + (f'на {a["first_m"]:.0f} м' if a['first_m'] is not None else 'не было'))
        print(f'{c["bag"]}: {len(f)} кадров, с тревогой {alarm}, разбор+проверка p50 {np.median(ms):.0f} мс{extra}')

    page = {'generated': date.today().isoformat(), 'commit': git_rev(), 'host': host(),
            'zone': zone_params(), 'clips': data}
    html = TEMPLATE.read_text(encoding='utf-8').replace(
        '/*__DATA__*/null', json.dumps(page, ensure_ascii=False, separators=(',', ':')))
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(html, encoding='utf-8')
    print(f'{args.out}  {args.out.stat().st_size / 1e6:.1f} МБ')


if __name__ == '__main__':
    main()
