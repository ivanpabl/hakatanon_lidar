"""Дашборд метрик: один HTML-файл без внешних зависимостей (открывается без сети).

    python tools/dashboard.py                        # -> gui/dashboard.html
    python tools/dashboard.py --no-scene             # без 3D-сцены (не нужны записи в data/)
    python tools/dashboard.py --scene-frame 60 --out /tmp/d.html

Цифры берутся из результатов инструментов в runs/, а не переписываются руками:
  alarms_A_base.csv / alarms_P_ev_f20.csv      тревоги исходной и текущей версии на 6 записях
  alarms_nd_A_base.csv / alarms_nd_P_ev_f20.csv  отрезки new_data (8 x 400 кадров)
  alarms_final_nd_full.csv                     вся new_data, текущая версия
  approach_A_base.csv / approach_P_ev_f20.csv  дальность при подъезде (eval_approach.py)
  check_preproc.json                           побитное совпадение и время разбора (check_preproc.py)
  input_report.json                            проверка входного потока (input_report.py)
  smoke/doubleT_platform_*                     e2e в Docker с C++-приёмом (./run.sh play)
Чего нет в runs/ (замер десериализации, число тестов), лежит в DOCUMENTED с источником.
3D-сцена -- кадр doubleT_obstacle после прогона детектора с первого кадра: облако, ось пути,
коридор, объект на 56 м.
"""
import argparse
import base64
import csv
import json
import re
import subprocess
import sys
import warnings
from datetime import date
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from bags import BAGS, RUNS, ROOT, open_cloud_bag   # noqa: E402

TEMPLATE = ROOT / 'gui' / 'template.html'
OUT = ROOT / 'gui' / 'dashboard.html'
BEFORE, AFTER = 'A_base', 'P_ev_f20'           # исходная версия (ae38df6) и текущая по умолчанию
EMPTY = [b for b in BAGS if b not in ('doubleT_obstacle', 'new_data')]
FRAME_MB = {'doubleT_obstacle': 23}            # остальные записи -- 8 МБ
GAP = 2                                        # как tools/alarms.py: разрыв до 2 кадров -- тот же эпизод

# Замеры, которых нет в runs/: значение и откуда оно.
DOCUMENTED = {
    'deser_ms': {'p50': 5.5, 'p95': 10.6,
                 'src': 'research/ingest_timing.py: десериализация 23 МБ в Python, которой больше нет в узле'},
    'pytest': {'n': 35, 'src': 'последний прогон тестов: ./run.sh test и pytest tests'},
    'bench': 'Intel i5-1038NG7, 4 ядра / 8 потоков, Docker в VM OrbStack',
}


def read_csv(name):
    p = RUNS / name
    return list(csv.DictReader(open(p))) if p.exists() else None


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
        for b in EMPTY:
            sb, sa = alarm_share([r for r in before if r['bag'] == b]), alarm_share([r for r in after if r['bag'] == b])
            per_bag.append({'bag': b, 'frames': sa['frames'], 'before': sb['pct'], 'after': sa['pct']})
        eb, ea = [r for r in before if r['bag'] in EMPTY], [r for r in after if r['bag'] in EMPTY]
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
    p = RUNS / 'check_preproc.json'
    if p.exists():
        d = json.loads(p.read_text())
        s['parse'] = [{'bag': b, 'mb': FRAME_MB.get(b, 8), 'python': v['parse_ms_python'], 'cpp': v['parse_ms_cpp'],
                       'kept_after_crop': v['kept_after_crop']} for b, v in d.items()]
    st, res = RUNS / 'smoke' / 'doubleT_platform_stats.json', RUNS / 'smoke' / 'doubleT_platform_result.jsonl'
    if st.exists():
        stats = json.loads(st.read_text())
        rows = [json.loads(line) for line in open(res) if line.strip()] if res.exists() else []
        pre = stats.get('preproc', {})
        s['e2e'] = {k: stats.get(k) for k in ('received', 'processed', 'dropped_stale', 'latency_ms_p50', 'latency_ms_p95',
                                              'latency_ms_max', 'e2e_ms_p50', 'e2e_ms_p95', 'e2e_ms_max', 'alarm_frames')}
        s['e2e'].update({'bag': 'doubleT_platform', 'mb': 8, 'rate_hz': 10, 'preproc_published': pre.get('published'),
                         'latency_ms': [round(r['latency_ms'], 2) for r in rows if r.get('latency_ms') is not None],
                         'detect_ms': [round(r['detect_ms'], 2) for r in rows if r.get('detect_ms') is not None],
                         'queue_ms': [round(r['queue_ms'], 2) for r in rows if r.get('queue_ms') is not None]})
        pp = RUNS / 'smoke' / 'doubleT_platform_stats_preproc.json'
        if pp.exists():
            pj = json.loads(pp.read_text())
            s['e2e']['preproc_ms_p50'], s['e2e']['preproc_ms_p95'] = pj.get('preproc_ms_p50'), pj.get('preproc_ms_p95')
            s['e2e']['transport_ms_p50'], s['e2e']['transport_ms_p95'] = pj.get('transport_ms_p50'), pj.get('transport_ms_p95')
    return s


def count_tests():
    """gtest считается по исходникам; pytest -- из DOCUMENTED: тесты узлов собираются только в образе."""
    gtest = sum(len(re.findall(r'^TEST(?:_F|_P)?\(', f.read_text(), re.M))
                for f in (ROOT / 'ros2_ws' / 'src').glob('*/test/*.cpp'))
    return {'gtest': gtest, 'pytest': DOCUMENTED['pytest']['n'], 'pytest_src': DOCUMENTED['pytest']['src']}


def robustness():
    r = {'tests': count_tests()}
    p = RUNS / 'check_preproc.json'
    if p.exists():
        d = json.loads(p.read_text())
        r['bitwise'] = [{'bag': b, 'frames': v['frames'], 'parse_bitwise': v['parse_bitwise'],
                         'result_identical': v['result_identical']} for b, v in d.items()]
    p = RUNS / 'input_report.json'
    if p.exists():
        d = json.loads(p.read_text())
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
    snap = det.track_path()
    fwd, lat = -y, x
    keep = (fwd > -15) & (fwd < 230) & (np.abs(lat) < 12) & (z > -6) & (z < 8)
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

    q = np.stack([np.round(lat * 100), np.round(z * 100), np.round(fwd * 100)], 1).astype(np.int16)
    step = 1.0
    axis_f = np.arange(2.0, (snap.path_range or 40.0) + step / 2, step)
    axis = np.stack([snap.center_at(axis_f), snap.rail_top_at(axis_f), axis_f], 1)
    rails = None
    if snap.rail_prof is not None:
        rails = float(snap.rail_prof[0][-1])
    return {'bag': bag, 'frame': frame, 'n_points': int(len(fwd)),
            'points': base64.b64encode(q.tobytes()).decode(), 'tags': base64.b64encode(tag.tobytes()).decode(),
            'axis': np.round(axis, 3).tolist(), 'path_range_m': snap.path_range, 'rails_to_m': rails,
            'obstacle': bool(res['obstacle']), 'distance_m': res['distance_m'], 'objects': objects[:12],
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
            'quality': quality(), 'speed': speed(), 'robust': robustness()}
    if not args.no_scene:
        try:
            data['scene'] = scene(args.scene_frame)
        except Exception as e:                                          # noqa: BLE001
            print(f'3D-сцена пропущена: {e}', file=sys.stderr)
    html = TEMPLATE.read_text().replace('/*__DATA__*/null', json.dumps(data, ensure_ascii=False, separators=(',', ':')))
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(html)
    print(f'{args.out}  {args.out.stat().st_size / 1e6:.1f} МБ')


if __name__ == '__main__':
    main()
