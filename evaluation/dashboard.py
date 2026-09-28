"""Дашборд метрик: один HTML-файл без внешних зависимостей (открывается без сети).

    python -m evaluation dashboard                   # -> output/report/metrics.html
    python -m evaluation dashboard --no-scene        # без 3D-сцены (не нужны записи в data/)
    python -m evaluation dashboard --scene-frame 60 --out /tmp/d.html

Цифры берутся из результатов инструментов в output/runs, а не переписываются руками:
  alarms_A_base.csv / alarms_P_ev_f20.csv      тревоги исходной и текущей версии на 6 записях
  alarms_nd_A_base.csv / alarms_nd_P_ev_f20.csv  отрезки new_data (8 x 400 кадров)
  alarms_final_nd_full.csv                     вся new_data, текущая версия
  approach_A_base.csv / approach_P_ev_f20.csv  дальность при подъезде (eval_approach.py)
  check_preproc.json                           побитное совпадение и время разбора (check_preproc.py)
  input_report.json                            проверка входного потока (input_report.py)
  smoke/doubleT_platform_*                     e2e в Docker с C++-приёмом (docker compose up play)
Файла нет в output/runs -- берётся из evaluation/baseline.json (эталон, в git): исходная версия
A_base (её код -- ae38df6, текущим не воспроизводится), e2e в Docker, проверка входа и замер
разбора C++. Откуда взят каждый файл, видно на странице (раздел «Методика»).
Чего нет ни там, ни там (замер десериализации, число тестов), лежит в DOCUMENTED с источником.
3D-сцена -- кадр doubleT_obstacle после прогона детектора с первого кадра: облако, ось пути,
коридор, объект на 56 м.
"""
import argparse
import base64
import csv
import json
import os
import platform
import re
import subprocess
import sys
import warnings
from datetime import date
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from bags import BAGS, RUNS, ROOT, open_cloud_bag   # noqa: E402

HERE = Path(__file__).resolve().parent
TEMPLATE = HERE / 'templates' / 'dashboard.html'
BASELINE = json.loads((HERE / 'baseline.json').read_text(encoding='utf-8'))
OUT = ROOT / 'output' / 'report' / 'metrics.html'
BEFORE, AFTER = 'A_base', 'P_ev_f20'           # исходная версия (ae38df6) и текущая по умолчанию
EMPTY = [b for b in BAGS if b not in ('doubleT_obstacle', 'new_data')]
FRAME_MB = {'doubleT_obstacle': 23}            # остальные записи -- 8 МБ
GAP = 2                                        # как alarms.py: разрыв до 2 кадров -- тот же эпизод

# Замеры, которых нет в runs/: значение и откуда оно.
DOCUMENTED = {
    'deser_ms': {'p50': 5.5, 'p95': 10.6,
                 'src': 'ветка research, research/ingest_timing.py: десериализация 23 МБ в Python, которой больше нет в узле'},
    'pytest': {'n': 35, 'src': 'последний прогон тестов: docker compose run test и pytest tests'},
    'bench': BASELINE['bench'],
}


SOURCES = {}                                   # файл -> 'runs' | 'baseline': откуда взят


def _found(name, kind):
    SOURCES[name] = kind
    return True


def read_csv(name):
    """Строки csv из output/runs, иначе из baseline.json (значения -- строки, как в csv)."""
    p = RUNS / name
    if p.exists() and _found(name, 'runs'):
        with open(p, encoding='utf-8', newline='') as f:
            return list(csv.DictReader(f))
    b = BASELINE['files'].get(name)
    if b is None:
        return None
    _found(name, 'baseline')
    return [dict(zip(b['columns'], row)) for row in b['rows']]


def read_json(name):
    p = RUNS / name
    if p.exists() and _found(name, 'runs'):
        return json.loads(p.read_text(encoding='utf-8'))
    b = BASELINE['files'].get(name)
    return b if b is None or _found(name, 'baseline') else None


def read_jsonl(name):
    p = RUNS / name
    if p.exists() and _found(name, 'runs'):
        return [json.loads(line) for line in p.read_text(encoding='utf-8').splitlines() if line.strip()]
    b = BASELINE['files'].get(name)
    return b if b is None or _found(name, 'baseline') else None


def host():
    """Эта машина: для подписи прогонов качества и замера разбора, если он сделан здесь."""
    cpu = platform.processor() or platform.machine()
    if sys.platform == 'darwin':
        try:
            cpu = subprocess.run(['sysctl', '-n', 'machdep.cpu.brand_string'], capture_output=True,
                                 text=True).stdout.strip() or cpu
        except OSError:
            pass
    elif sys.platform.startswith('linux'):
        try:
            cpu = next(line.split(':', 1)[1].strip() for line in open('/proc/cpuinfo', encoding='utf-8')
                       if line.startswith('model name'))
        except (OSError, StopIteration):
            pass
    return f'{cpu}, {os.cpu_count()} потоков, {platform.system()} {platform.release()}, Python {platform.python_version()}'


def episodes(alarm):
    idx = np.where(alarm)[0]
    if not len(idx):
        return 0
    return len(np.split(idx, np.where(np.diff(idx) > GAP + 1)[0] + 1))


def alarm_share(rows):
    a = np.array([int(r['alarm']) for r in rows])
    hours = len(a) / 10.0 / 3600
    return {'frames': len(a), 'alarm_frames': int(a.sum()), 'pct': 100 * a.mean(),
            'episodes': episodes(a), 'per_hour': episodes(a) / hours if hours else 0}


def quality():
    q = {}
    before, after = read_csv(f'alarms_{BEFORE}.csv'), read_csv(f'alarms_{AFTER}.csv')
    if before and after:
        ob = lambda rows: [r for r in rows if r['bag'] == 'doubleT_obstacle']
        dist = [float(r['distance_m']) for r in ob(after) if r['alarm'] == '1']
        q['obstacle'] = {'before': [int(r['alarm']) for r in ob(before)],
                         'after': [int(r['alarm']) for r in ob(after)],
                         'distance_m': float(np.median(dist)) if dist else None}
        per_bag = []
        empty = [b for b in EMPTY if any(r['bag'] == b for r in after)]     # прогон мог быть не по всем
        for b in empty:
            sb, sa = alarm_share([r for r in before if r['bag'] == b]), alarm_share([r for r in after if r['bag'] == b])
            per_bag.append({'bag': b, 'frames': sa['frames'], 'before': sb['pct'], 'after': sa['pct']})
        eb, ea = [r for r in before if r['bag'] in empty], [r for r in after if r['bag'] in empty]
        q['empty'] = {'bags': per_bag, 'before': alarm_share(eb)['pct'], 'after': alarm_share(ea)['pct'],
                      'frames': len(ea), 'minutes': len(ea) / 600}
    nb, na = read_csv(f'alarms_nd_{BEFORE}.csv'), read_csv(f'alarms_nd_{AFTER}.csv')
    if nb and na:
        # эпизоды считаются внутри отрезка: детектор перезапускается в начале каждого
        def seg(rows):
            out = {'frames': 0, 'alarm_frames': 0, 'episodes': 0}
            for k in range(8):
                s = alarm_share(rows[k * 400:(k + 1) * 400])
                for key in out:
                    out[key] += s[key]
            out['pct'] = 100 * out['alarm_frames'] / out['frames']
            out['per_hour'] = out['episodes'] / (out['frames'] / 36000)
            return out
        q['nd_segments'] = {'before': seg(nb), 'after': seg(na)}
    full = read_csv('alarms_final_nd_full.csv')
    if full:
        s = alarm_share(full)
        a = np.array([int(r['alarm']) for r in full])
        d = [float(r['distance_m']) for r in full if r['alarm'] == '1']
        per_min = [int(a[i:i + 600].sum()) for i in range(0, len(a), 600)]
        per_100 = [int(a[i:i + 100].sum()) for i in range(0, len(a), 100)]
        q['nd_full'] = {**s, 'minutes': len(a) / 600, 'per_min': per_min, 'per_100': per_100,
                        'distance_median': float(np.median(d)) if d else None}
    ab, aa = read_csv(f'approach_{BEFORE}.csv'), read_csv(f'approach_{AFTER}.csv')
    if ab and aa:
        shapes = ['человек стоит', 'куб 0.4', 'человек лежит', 'куб 0.7', 'куб 0.2']
        starts = sorted({int(r['start_m']) for r in aa})

        def cell(rows, shape, start):
            rs = [r for r in rows if r['shape'] == shape and int(r['start_m']) == start]
            hit = [float(r['first_m']) for r in rs if r['detected'] == '1' and r['first_m']]
            return {'n': len(rs), 'found': len(hit), 'median': float(np.median(hit)) if hit else None}
        q['approach'] = {'shapes': shapes, 'starts': starts,
                         'cells': [{'shape': s, 'start': st, 'before': cell(ab, s, st), 'after': cell(aa, s, st)}
                                   for s in shapes for st in starts],
                         'windows': len({(r['bag'], r['window']) for r in aa})}
        err = sorted(abs(float(r['dist_err_m'])) for r in aa if r['dist_err_m'])
        q['dist_err'] = {'values': err, 'median': float(np.median(err)), 'p95': float(np.percentile(err, 95))}
    return q


def speed():
    s = {'deser': DOCUMENTED['deser_ms']}
    d = read_json('check_preproc.json')
    if d:
        s['parse'] = [{'bag': b, 'mb': FRAME_MB.get(b, 8), 'python': v['parse_ms_python'], 'cpp': v['parse_ms_cpp'],
                       'kept_after_crop': v['kept_after_crop']} for b, v in d.items()]
    stats = read_json('smoke/doubleT_platform_stats.json')
    if stats:
        rows = read_jsonl('smoke/doubleT_platform_result.jsonl') or []
        pre = stats.get('preproc', {})
        s['e2e'] = {k: stats.get(k) for k in ('received', 'processed', 'dropped_stale', 'latency_ms_p50', 'latency_ms_p95',
                                              'latency_ms_max', 'e2e_ms_p50', 'e2e_ms_p95', 'e2e_ms_max', 'alarm_frames')}
        s['e2e'].update({'bag': 'doubleT_platform', 'mb': 8, 'rate_hz': 10, 'preproc_published': pre.get('published'),
                         'latency_ms': [round(r['latency_ms'], 2) for r in rows if r.get('latency_ms') is not None],
                         'detect_ms': [round(r['detect_ms'], 2) for r in rows if r.get('detect_ms') is not None],
                         'queue_ms': [round(r['queue_ms'], 2) for r in rows if r.get('queue_ms') is not None]})
        pj = read_json('smoke/doubleT_platform_stats_preproc.json')
        if pj:
            s['e2e']['preproc_ms_p50'], s['e2e']['preproc_ms_p95'] = pj.get('preproc_ms_p50'), pj.get('preproc_ms_p95')
            s['e2e']['transport_ms_p50'], s['e2e']['transport_ms_p95'] = pj.get('transport_ms_p50'), pj.get('transport_ms_p95')
    return s


def count_tests():
    """gtest считается по исходникам; pytest -- из DOCUMENTED: тесты узлов собираются только в образе."""
    gtest = sum(len(re.findall(r'^TEST(?:_F|_P)?\(', f.read_text(encoding='utf-8'), re.M))
                for f in (ROOT / 'ros2_ws' / 'src').glob('*/test/*.cpp'))
    return {'gtest': gtest, 'pytest': DOCUMENTED['pytest']['n'], 'pytest_src': DOCUMENTED['pytest']['src']}


def robustness():
    r = {'tests': count_tests()}
    d = read_json('check_preproc.json')
    if d:
        r['bitwise'] = [{'bag': b, 'frames': v['frames'], 'parse_bitwise': v['parse_bitwise'],
                         'result_identical': v['result_identical']} for b, v in d.items()]
    d = read_json('input_report.json')
    if d:
        rows = []
        for bag, v in d.items():
            c = v['checks']
            rows.append({'bag': bag, 'frames': v['frames'], 'violations': v['violations'],
                         'levels': {k: c[k]['level'] for k in ('format', 'M1', 'M2', 'M3', 'M4', 'M5', 'M6')},
                         'messages': {k: c[k]['message'] for k in ('format', 'M1', 'M2', 'M3', 'M4', 'M5', 'M6')},
                         'zero_share': c['M1'].get('zero_share'), 'stamp_year': c['M4'].get('stamp_year'),
                         'gaps': c['M4'].get('gaps'), 'gap_total_s': c['M4'].get('gap_total_s'),
                         'sector_deg': c['M5'].get('sector_deg'), 'pair_share': c['M2'].get('pair_coincident_share'),
                         'format': c['format'].get('message')})
        r['input'] = rows
    return r


def frame_view(x, y, z, res, snap, max_fwd=230.0, bg_points=None, rng=None):
    """Кадр для 3D-вида: точки (см, int16: вбок, вверх, вперёд), метки точек, ось, объекты.
    bg_points -- оставить столько точек фона (случайно), точки зоны и объектов -- все."""
    fwd, lat = -y, x
    keep = (fwd > -15) & (fwd < max_fwd) & (np.abs(lat) < 12) & (z > -6) & (z < 8)
    fwd, lat, z = fwd[keep], lat[keep], z[keep]

    # оси детектора: вперёд = -y, вбок = x; в сцене: X вбок, Y вверх, Z назад (правая тройка WebGL)
    cf = np.clip(fwd, 2, None)
    top = snap.rail_top_at(cf)
    rel = z - top                                    # высота над головкой рельса
    cen = snap.center_at(cf)
    in_zone = (fwd > 2) & (np.abs(lat - cen) <= 1.0) & (rel >= 0.15) & (rel <= 2.0)
    if snap.path_range:
        in_zone &= fwd <= snap.path_range
    tag = np.zeros(len(fwd), np.uint8)               # 0 фон, 1 зона, 2 объект (подтверждён), 3 объект (нет)
    tag[in_zone] = 1
    objects = []
    for o in res['objects']:
        if o['too_small'] and not o['confirmed']:
            continue
        near, far = o['distance_m'], o.get('far_m', o['distance_m'])
        sel = in_zone & (fwd >= near - 0.05) & (fwd <= far + 0.05) & (np.abs(lat - cen - o['lateral_m']) < 0.8)
        alarm = bool(o['confirmed'] and not o['beyond_path'])
        tag[sel] = 2 if alarm else 3
        if sel.any():
            objects.append({'distance_m': near, 'far_m': far, 'lateral_m': o['lateral_m'], 'height_m': o['height_m'],
                            'n_points': int(o['n_points']), 'confirmed': bool(o['confirmed']), 'alarm': alarm,
                            'edge_line': bool(o.get('edge_line')),
                            'box': [float(lat[sel].min()), float(lat[sel].max()), float(z[sel].min()),
                                    float(z[sel].max()), float(fwd[sel].min()), float(fwd[sel].max())]})
    objects.sort(key=lambda o: (not o['alarm'], o['distance_m']))

    if bg_points is not None and (tag == 0).sum() > bg_points:
        # фон: сначала по точке на воксель (вблизи точек в сотни раз больше, чем вдали -- дальние
        # стены так не пропадают), остаток -- случайно
        bg = np.flatnonzero(tag == 0)
        for vox in (0.15, 0.25, 0.4):
            key = np.floor(np.stack([lat[bg], z[bg], fwd[bg]], 1) / vox).astype(np.int32)
            _, first = np.unique(key, axis=0, return_index=True)
            if len(first) <= bg_points * 1.5:
                break
        keep_bg = bg[np.sort(first)]
        if len(keep_bg) > bg_points:
            keep_bg = np.sort((rng or np.random.default_rng(0)).choice(keep_bg, bg_points, replace=False))
        mask = tag > 0
        mask[keep_bg] = True
        lat, z, fwd, tag = lat[mask], z[mask], fwd[mask], tag[mask]
    q = np.stack([np.round(lat * 100), np.round(z * 100), np.round(fwd * 100)], 1).astype(np.int16)
    step = 1.0
    axis_f = np.arange(2.0, (snap.path_range or 40.0) + step / 2, step)
    axis = np.stack([snap.center_at(axis_f), snap.rail_top_at(axis_f), axis_f], 1)
    rails = None
    if snap.rail_prof is not None:
        rails = float(snap.rail_prof[0][-1])
    return {'q': q, 'tag': tag, 'axis': np.round(axis, 3).tolist(), 'path_range_m': snap.path_range,
            'rails_to_m': rails, 'objects': objects[:12]}


def scene(frame, bag='doubleT_obstacle'):
    """Кадр записи после прогона детектора с начала записи: облако, ось, коридор, объекты."""
    from tunnel_od import ObstacleDetector, parse_pointcloud2
    warnings.simplefilter('ignore')
    det = ObstacleDetector()
    with open_cloud_bag(bag) as (reader, conn):
        for i, (c, t, raw) in enumerate(reader.messages(connections=[conn])):
            m = reader.deserialize(raw, c.msgtype)
            x, y, z = parse_pointcloud2(m.data, m.point_step, m.fields)
            res = det.detect(x, y, z, refit_path=True, stamp=t / 1e9)
            if i == frame:
                break
    v = frame_view(x, y, z, res, det.track_path())
    return {'bag': bag, 'frame': frame, 'n_points': int(len(v['q'])),
            'points': base64.b64encode(v['q'].tobytes()).decode(), 'tags': base64.b64encode(v['tag'].tobytes()).decode(),
            'axis': v['axis'], 'path_range_m': v['path_range_m'], 'rails_to_m': v['rails_to_m'],
            'obstacle': bool(res['obstacle']), 'distance_m': res['distance_m'], 'objects': v['objects'],
            'zone': {'half_width': 1.0, 'clearance': 0.15, 'height': 2.0}}


def git_rev():
    try:
        return subprocess.run(['git', 'rev-parse', '--short', 'HEAD'], cwd=ROOT, capture_output=True,
                              text=True).stdout.strip()
    except OSError:
        return ''


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--out', type=Path, default=OUT)
    ap.add_argument('--no-scene', action='store_true', help='без 3D-сцены (записи в data/ не нужны)')
    ap.add_argument('--scene-frame', type=int, default=101, help='кадр doubleT_obstacle для 3D-сцены')
    args = ap.parse_args()

    data = {'generated': date.today().isoformat(), 'commit': git_rev(), 'bench': DOCUMENTED['bench'],
            'host': host(), 'quality': quality(), 'speed': speed(), 'robust': robustness()}
    data['sources'] = dict(sorted(SOURCES.items()))
    if not args.no_scene:
        try:
            data['scene'] = scene(args.scene_frame)
        except Exception as e:                                          # noqa: BLE001
            print(f'3D-сцена пропущена: {e}', file=sys.stderr)
    html = TEMPLATE.read_text(encoding='utf-8').replace('/*__DATA__*/null', json.dumps(data, ensure_ascii=False, separators=(',', ':')))
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(html, encoding='utf-8')
    ref = [k for k, v in SOURCES.items() if v == 'baseline']
    print(f'{args.out}  {args.out.stat().st_size / 1e6:.1f} МБ')
    if ref:
        print(f'  из baseline.json: {", ".join(sorted(ref))}')


if __name__ == '__main__':
    main()
