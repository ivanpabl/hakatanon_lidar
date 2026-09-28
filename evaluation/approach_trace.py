"""Решение детектора по каждому кадру подъезда: где СТОП, где объект виден, но решение слабее, и почему.

Те же окна движения и та же вставка, что evaluation/eval_approach.py (медиана первого подтверждения
в дашборде -- оттуда), но подъезд идёт дальше, до 8 м или до max_frames, и сохраняется каждый кадр:

    stop     СТОП на вставленном объекте
    beyond   объект найден и подтверждён, но дальше конца оси пути в этом кадре -> ВНИМАНИЕ
             (path_margin = 0: за концом оси СТОП не поднимается)
    caution  объект найден, ВНИМАНИЕ по другой причине (pending -- свидетельство ещё копится,
             ego_carried -- ЭГО-тест)
    none     на месте объекта ничего нет: точек в зоне нет, объект меньше минимума или
             помечен продольной конструкцией

    python -m evaluation approach-trace --tag final        # -> output/runs/approach_trace_final.json
"""
import argparse
import os
for _v in ('OMP_NUM_THREADS', 'OPENBLAS_NUM_THREADS', 'MKL_NUM_THREADS'):
    os.environ.setdefault(_v, '1')
import copy
import json
import warnings
import zlib
from multiprocessing import Pool

import numpy as np

import eval_approach as ea
from bags import RUNS as OUT, cloud_parser
from eval_injection import front_of
from tunnel_od import ObstacleDetector
from tunnel_od.pointcloud import beam_directions, xyz_views
from tunnel_od.sim.inject import inject, on_track
from tunnel_od.sim.shapes import make_shape

TRIALS = [('человек стоит', 170), ('куб 0.4', 170)]
MIN_D = 8.0
FEAT_KEYS = ('distance_m', 'lateral_m', 'lat_min_m', 'lat_max_m', 'low_m', 'height_m', 'n_points', 'level', 'reason',
             'confirmed', 'hits', 'evidence', 'rings', 'ext_beams', 'behind_n', 'behind_low_n', 'front_maxh', 'side_n',
             'shell_n', 'wall_left_n', 'wall_right_n', 'implausible')


def features(res, front):
    """Признаки (detection/plausibility.py) самого крупного объекта детектора на месте вставленного."""
    near = [o for o in ea.match(res['objects'], front) if 'rings' in o]
    if not near:
        return None
    o = max(near, key=lambda o: o['n_points'])
    return {k: (round(o[k], 3) if isinstance(o[k], float) else o[k]) for k in FEAT_KEYS if k in o}


def category(res, front):
    near = ea.match(res['objects'], front)
    if not near:
        return 'none', None
    if any(o.get('level') == 'stop' for o in near):
        return 'stop', None
    o = max(near, key=lambda o: o['n_points'])
    if o.get('level') == 'caution' and o.get('reason') == 'beyond_path':
        return 'beyond', o.get('reason')
    if o.get('level') == 'caution':
        return 'caution', o.get('reason')
    return 'none', 'edge_line' if o.get('edge_line') else 'too_small' if o.get('too_small') else o.get('reason')


def run_bag(job):
    bag, det_kwargs, seed, max_frames, trials = job
    warnings.simplefilter('ignore')
    parse = cloud_parser('python')
    out = []
    for w, start in enumerate(ea.pick_windows(bag, ea.N_WINDOWS, det_kwargs)):
        frames = ea.read_frames(bag, start, ea.WARMUP + max_frames)
        ego = ObstacleDetector(ego_motion=True)
        ref = ObstacleDetector(**det_kwargs)
        for data, step, fields, t in frames[:ea.WARMUP]:
            xyz = parse(data, step, fields)
            ego.detect(*xyz, refit_path=True, stamp=t)
            ref.detect(*xyz, refit_path=True, stamp=t)
        start_state = copy.deepcopy(ref)
        prep, travelled = [], 0.0
        for k, (data, step, fields, t) in enumerate(frames[ea.WARMUP:]):
            xyz = parse(data, step, fields)
            e = ego.detect(*xyz, refit_path=True, stamp=t)
            if k:
                travelled += e['displacement_m'] or 0.0
            if max(d0 for _, d0 in trials) - travelled < MIN_D:
                break
            ref.detect(*xyz, refit_path=True, stamp=t)
            v = xyz_views(np.frombuffer(data, np.uint8).reshape(-1, step), fields)
            prep.append((data, step, fields, t, beam_directions(v['x'], v['y'], v['z']), ref.track_path(),
                         travelled, e['speed_mps']))
        for name, d0 in trials:
            kind, dims = ea.SHAPES[name]
            rng = np.random.default_rng(zlib.crc32(repr((seed, bag, w, name, d0)).encode()))
            test = copy.deepcopy(start_state)
            rows = []
            for data, step, fields, t, dirs, path, trav, speed in prep:
                d = d0 - trav
                if d < MIN_D:
                    break
                lat, z0 = on_track(path, d)
                new, info = inject(data, step, [make_shape(kind, dims, d, lat, z0)], fields, dirs=dirs, rng=rng)
                res = test.detect(*parse(new, step, fields), refit_path=True, stamp=t)
                front = front_of(kind, dims, d)
                cat, why = category(res, front)
                rows.append([round(front, 1), cat, why, None if res['path_range_m'] is None else round(res['path_range_m'], 1),
                             int(info['points_on_object']), None if speed is None else round(speed, 1),
                             features(res, front)])
            out.append({'bag': bag, 'window': w, 'start_frame': start, 'shape': name, 'start_m': d0,
                        'cols': ['truth_m', 'cat', 'reason', 'path_range_m', 'points_on_object', 'speed_mps', 'feat'],
                        'frames': rows})
        print(f'  {bag}: окно {w + 1} (кадр {start}), {len(prep)} кадров подъезда', flush=True)
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--tag', default='final')
    ap.add_argument('--bags', nargs='+', default=ea.BAGS)
    ap.add_argument('--det', default='{}', help='kwargs ObstacleDetector в JSON')
    ap.add_argument('--workers', type=int, default=3)
    ap.add_argument('--seed', type=int, default=0)
    ap.add_argument('--max-frames', type=int, default=200, help='кадров подъезда, не больше')
    ap.add_argument('--shapes', nargs='+', default=None,
                    help='фигура:старт, например "человек лежит:170" (по умолчанию -- TRIALS)')
    args = ap.parse_args()
    det_kwargs = json.loads(args.det)
    OUT.mkdir(exist_ok=True)
    with Pool(min(len(args.bags), args.workers)) as pool:
        tr = TRIALS if not args.shapes else [(s.rsplit(':', 1)[0], int(s.rsplit(':', 1)[1])) for s in args.shapes]
        jobs = [(b, det_kwargs, args.seed, args.max_frames, tr) for b in args.bags]
        trials = [r for part in pool.imap_unordered(run_bag, jobs) for r in part]
    trials.sort(key=lambda r: (r['shape'], r['bag'], r['window']))
    path = OUT / f'approach_trace_{args.tag}.json'
    path.write_text(json.dumps({'det': det_kwargs, 'trials': trials}, ensure_ascii=False), encoding='utf-8')
    print(f'{len(trials)} подъездов -> {path}')
    for tr in trials:
        cats = [r[1] for r in tr['frames']]
        first = next((i for i, c in enumerate(cats) if c == 'stop'), None)
        tail = cats[first:] if first is not None else []
        print(f'{tr["shape"]:<14} {tr["bag"]:<36} w{tr["window"]}: кадров {len(cats)}, первый СТОП '
              f'{tr["frames"][first][0] if first is not None else "--"} м; после него СТОП {tail.count("stop")}, '
              f'за осью {tail.count("beyond")}, ВНИМ. {tail.count("caution")}, нет {tail.count("none")}')


if __name__ == '__main__':
    main()
