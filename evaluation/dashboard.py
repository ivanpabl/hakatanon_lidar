"""Дашборд метрик: один HTML-файл без внешних зависимостей (открывается без сети).

    python -m evaluation dashboard                   # -> output/report/metrics.html
    python -m evaluation dashboard --no-scene        # без 3D-сцены (не нужны записи в data/)
    python -m evaluation dashboard --scene-frame 60 --out /tmp/d.html

Цифры берутся из результатов инструментов в output/runs, а не переписываются руками. Итоговая версия
называется final или P_ev_f20 (python -m evaluation all) -- берётся более свежий файл:
  alarms_A_base.csv / alarms_<итог>.csv        тревоги исходной и итоговой версии на 6 записях
  alarms_final_nd_full.csv                     вся new_data, итоговая версия
  approach_A_base.csv / approach_<итог>.csv    дальность при подъезде (eval_approach.py)
  fake_obj_fo_{base,final}.csv, alarms_fo_final.csv, fake_obj_truth.csv
                                               бэг организаторов с 10 объектами (eval_fake_obj.py)
  selflabel_nd_final.csv                       эпизоды СТОП на new_data, проверенные проездом (selflabel.py)
  compare_table.md                             варианты параметров (python -m evaluation compare)
  check_preproc.json                           побитное совпадение и время разбора (check_preproc.py)
  input_report.json                            проверка входного потока (input_report.py)
  smoke/<запись>_*, docker/<запись>_*         e2e в Docker с C++-приёмом (docker compose up play)
Файла нет в output/runs -- берётся из evaluation/baseline.json (эталон, в git): исходная версия
A_base (её код -- ae38df6, текущим не воспроизводится), e2e в Docker, проверка входа, замер
разбора C++, разметка объектов организаторов по кадрам, таблица вариантов. Откуда взят каждый файл, видно на странице (раздел «Методика»).
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

from bags import RUNS, ROOT, open_cloud_bag
from bags import EMPTY_BAGS as EMPTY


def zone_params():
    """Прямоугольная зона детектора по умолчанию (half_width, clearance, height) -- для рисования."""
    import inspect
    from tunnel_od import ObstacleDetector
    p = inspect.signature(ObstacleDetector.__init__).parameters
    return {k: p[k].default for k in ('half_width', 'clearance', 'height')}

HERE = Path(__file__).resolve().parent
TEMPLATE = HERE / 'templates' / 'dashboard.html'
BASELINE = json.loads((HERE / 'baseline.json').read_text(encoding='utf-8'))
OUT = ROOT / 'output' / 'report' / 'metrics.html'
BEFORE = 'A_base'
AFTER_TAGS = ('final', 'P_ev_f20')
FRAME_MB = {'doubleT_obstacle': 23}
GAP = 2

DOCUMENTED = {
    'deser_ms': {'p50': 5.5, 'p95': 10.6,
                 'src': 'ветка research, research/ingest_timing.py: десериализация 23 МБ в Python, которой больше нет в узле'},
    'bench': BASELINE['bench'],
}


SOURCES = {}


def _found(name, kind):
    SOURCES[name] = kind
    return True


def _baseline(name):
    b = BASELINE['files'].get(name)
    return b if b is None or _found(name, 'baseline') else None


def has(name):
    return (RUNS / name).exists() or name in BASELINE['files']


def after_tag(prefix, ext='.csv'):
    """Тег итоговой версии для файлов prefix<тег><ext>: самый свежий из AFTER_TAGS в output/runs, иначе из baseline.json."""
    found = [((RUNS / f'{prefix}{t}{ext}').stat().st_mtime, t) for t in AFTER_TAGS if (RUNS / f'{prefix}{t}{ext}').exists()]
    if found:
        return max(found)[1]
    return next((t for t in AFTER_TAGS if f'{prefix}{t}{ext}' in BASELINE['files']), AFTER_TAGS[0])


def read_csv(name):
    """Строки csv из output/runs, иначе из baseline.json (значения -- строки, как в csv)."""
    p = RUNS / name
    if p.exists() and _found(name, 'runs'):
        with open(p, encoding='utf-8', newline='') as f:
            return list(csv.DictReader(f))
    b = _baseline(name)
    return None if b is None else [dict(zip(b['columns'], row)) for row in b['rows']]


def read_json(name):
    p = RUNS / name
    if p.exists() and _found(name, 'runs'):
        return json.loads(p.read_text(encoding='utf-8'))
    return _baseline(name)


def read_jsonl(name):
    p = RUNS / name
    if p.exists() and _found(name, 'runs'):
        return [json.loads(line) for line in p.read_text(encoding='utf-8').splitlines() if line.strip()]
    return _baseline(name)


def read_text(name):
    p = RUNS / name
    if p.exists() and _found(name, 'runs'):
        return p.read_text(encoding='utf-8')
    b = _baseline(name)
    return None if b is None else b['text']


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


def status_mix(bag, rows):
    st = [r.get('status') or ('stop' if r['alarm'] == '1' else 'clear') for r in rows]
    n = len(st)
    return {'bag': bag, 'frames': n, **{k: 100 * st.count(k) / n if n else 0 for k in ('stop', 'caution', 'unknown', 'clear')}}


FAKE_SHORT = {1: 'куб 2×2 м по оси', 2: 'предмет 0,3 м на высоте лидара, в кривой', 3: 'предмет 0,3 м на правом рельсе',
              4: 'предмет 0,3 м справа у края', 5: 'предмет 0,3 м слева за краем', 6: 'короб 2×2×2 м справа, уходит в стену',
              7: 'короб 2×2×2 м слева, уходит в стену', 8: 'короб над путём у свода', 9: 'плита 2,0×0,5×0,25 м поперёк пути',
              10: 'стержень 5 см со свода'}
FAKE_WHY = {4: 'край объекта на 0,96–1,0 м от нашей оси: зона ±1,0 м даёт ВНИМАНИЕ. Шире нельзя — СТОП получит объект 5 за габаритом',
            10: 'низ на 2,7 м над головкой рельса, выше зоны 2,5 м. Зона 3,0 м его видит, но даёт СТОП на объекте 8 у свода',
            6: 'СТОП вспышкой на 186 м, дальше ВНИМАНИЕ: край короба идёт по краю зоны',
            2: 'в кривой дальше 50 м генератор уводит объект с нашей оси — в зону он входит ближе 45 м',
            3: 'в кривой на 50–80 м генератор поднимает объект на 2,4 м — в зону он входит ближе 50 м'}


def fake_obj():
    """Бэг организаторов: итог по 10 объектам и дистанция каждого объекта по кадрам с решением детектора."""
    fin, base = read_csv('fake_obj_fo_final.csv'), read_csv('fake_obj_fo_base.csv')
    if not fin:
        return None
    lvl0 = {int(r['id']): r['level'] for r in base or []}
    f = lambda v: float(v) if v not in ('', None) else None
    objs = [{'id': int(r['id']), 'name': FAKE_SHORT.get(int(r['id']), r['desc'][:40]), 'expect': r['expect'],
             'level': r['level'] or 'none', 'base_level': lvl0.get(int(r['id'])) or None, 'first_stop_m': f(r['first_stop_m']),
             'sight_m': f(r['sight_m']), 'stop_share': f(r['stop_share']), 'ok': r['ok'] == 'True',
             'why': FAKE_WHY.get(int(r['id']))} for r in fin]
    out = {'objects': objs}
    frames, truth = read_csv('alarms_fo_final.csv'), read_csv('fake_obj_truth.csv')
    if frames and truth:
        code = {'clear': 0, 'caution': 1, 'stop': 2, 'unknown': 3}
        out['frames'] = [[code.get(r['status'], 0), f(r['distance_m']), f(r['caution_distance_m']), f(r['sight_m'])] for r in frames]
        tracks = {}
        for r in truth:
            i, o, d = int(r['frame']), int(r['obj']), float(r['dist_m'])
            if not 0 < d <= 300 or i >= len(frames):
                continue
            st, dist, cdist, _ = out['frames'][i]
            s_ = 2 if st == 2 and dist is not None and abs(dist - d) < 5 else \
                 1 if cdist is not None and abs(cdist - d) < 5 else 0
            tracks.setdefault(o, []).append([i, round(d, 1), s_])
        out['tracks'] = tracks
        out['stop_outside'] = sum(1 for i, fr in enumerate(out['frames']) if fr[0] == 2 and not any(
            p[0] == i and p[2] == 2 for t in tracks.values() for p in t))
    return out


def experiments():
    """Строки compare_table.md: тег, параметры, итог на бэге организаторов и прочих наборах."""
    text = read_text('compare_table.md')
    if not text:
        return None
    rows, seen = [], {}
    for line in text.splitlines():
        c = [x.strip() for x in line.strip().strip('|').split('|')]
        if len(c) < 8 or c[0] in ('тег', '') or set(c[0]) <= set('-'):
            continue
        m = re.search(r'СТОП (\d+)/(\d+); СТОП вне габарита: ([^;]+); ложных СТОП-кадров (\d+)', c[2])
        row = {'tag': c[0], 'params': c[1].strip('`'), 'fo_in': int(m.group(1)) if m else None,
               'fo_n': int(m.group(2)) if m else None, 'fo_out': m.group(3).strip() if m else None,
               'fo_false': int(m.group(4)) if m else None, 'empty': c[3], 'nd': c[4], 'person': c[5], 'cube': c[6]}
        seen[(row['tag'], row['params'])] = len(rows)
        rows.append(row)
    return [rows[i] for i in sorted(set(seen.values()))]


def work():
    """Объём проделанной работы: коммиты, тесты, записи и кадры, на которых проверялся детектор."""
    try:
        commits = int(subprocess.run(['git', 'rev-list', '--count', 'HEAD'], cwd=ROOT, capture_output=True,
                                     text=True).stdout.strip() or 0)
    except (OSError, ValueError):
        commits = None
    pytest = sum(len(re.findall(r'^\s*def test_', f.read_text(encoding='utf-8'), re.M)) for f in (ROOT / 'tests').glob('test_*.py'))
    return {'commits': commits, 'pytest': pytest}


def quality():
    q = {}
    before, after = read_csv(f'alarms_{BEFORE}.csv'), read_csv(f'alarms_{after_tag("alarms_")}.csv')
    if before and after:
        ob = lambda rows: [r for r in rows if r['bag'] == 'doubleT_obstacle']
        dist = [float(r['distance_m']) for r in ob(after) if r['alarm'] == '1']
        q['obstacle'] = {'before': [int(r['alarm']) for r in ob(before)],
                         'after': [int(r['alarm']) for r in ob(after)],
                         'distance_m': float(np.median(dist)) if dist else None}
        per_bag = []
        empty = [b for b in EMPTY if any(r['bag'] == b for r in after)]
        for b in empty:
            sb, sa = alarm_share([r for r in before if r['bag'] == b]), alarm_share([r for r in after if r['bag'] == b])
            per_bag.append({'bag': b, 'frames': sa['frames'], 'before': sb['pct'], 'after': sa['pct']})
        eb, ea = [r for r in before if r['bag'] in empty], [r for r in after if r['bag'] in empty]
        q['empty'] = {'bags': per_bag, 'before': alarm_share(eb)['pct'], 'after': alarm_share(ea)['pct'],
                      'frames': len(ea), 'minutes': len(ea) / 600}
    if after:
        q['status'] = [status_mix(b, [r for r in after if r['bag'] == b]) for b in dict.fromkeys(r['bag'] for r in after)]
    nd_name = nd_full_name()
    full = read_csv(nd_name)
    if full:
        s = alarm_share(full)
        a = np.array([int(r['alarm']) for r in full])
        d = [float(r['distance_m']) for r in full if r['alarm'] == '1']
        per_min = [int(a[i:i + 600].sum()) for i in range(0, len(a), 600)]
        per_100 = [int(a[i:i + 100].sum()) for i in range(0, len(a), 100)]
        q['nd_full'] = {**s, 'minutes': len(a) / 600, 'per_min': per_min, 'per_100': per_100,
                        'distance_median': float(np.median(d)) if d else None, 'file': nd_name,
                        'dist_bins': [sum(lo <= v < hi for v in d) for lo, hi in ((0, 40), (40, 80), (80, 130), (130, 1e9))]}
        q.setdefault('status', []).append(status_mix('new_data', full))
    fo = read_csv('alarms_fo_final.csv')
    if fo:
        q.setdefault('status', []).append(status_mix('cloud_with_fake_obj', fo))
    sl = read_csv('selflabel_nd_final.csv')
    if sl:
        labels = [r['label'] for r in sl]
        q['selflabel'] = {'episodes': len(labels), 'false_proven': labels.count('false_proven'),
                          'unresolved': labels.count('unresolved')}
        head = read_text('selflabel_nd_final.txt')
        if head:
            km = re.search(r'пробег по одометрии ([\d.]+) км', head)
            q['selflabel'].update({'unreliable': 'НЕНАДЁЖНО' in head, 'odometry_km': float(km.group(1)) if km else None})
    ab, aa = read_csv(f'approach_{BEFORE}.csv'), read_csv(f'approach_{after_tag("approach_")}.csv')
    if ab and aa:
        shapes = ['человек стоит', 'куб 0.4', 'человек лежит', 'куб 0.7', 'куб 0.2']
        starts = sorted({int(r['start_m']) for r in aa})

        def cell(rows, shape, start):
            rs = [r for r in rows if r['shape'] == shape and int(r['start_m']) == start]
            hit = [float(r['first_m']) for r in rs if r['detected'] == '1' and r['first_m']]
            return {'n': len(rs), 'found': len(hit), 'median': float(np.median(hit)) if hit else None,
                    'values': sorted(hit)}
        q['approach'] = {'shapes': shapes, 'starts': starts,
                         'cells': [{'shape': s, 'start': st, 'before': cell(ab, s, st), 'after': cell(aa, s, st)}
                                   for s in shapes for st in starts],
                         'windows': len({(r['bag'], r['window']) for r in aa})}
        err = sorted(abs(float(r['dist_err_m'])) for r in aa if r['dist_err_m'])
        q['dist_err'] = {'values': err, 'median': float(np.median(err)), 'p95': float(np.percentile(err, 95))}
    tr = read_json(f'approach_trace_{after_tag("approach_trace_", ".json")}.json')
    if tr:
        code = {'stop': 'S', 'beyond': 'B', 'caution': 'C', 'none': 'N'}
        q['trace'] = [{'bag': t['bag'], 'window': t['window'], 'shape': t['shape'], 'start_m': t['start_m'],
                       'truth_m': [r[0] for r in t['frames']], 'cat': ''.join(code[r[1]] for r in t['frames']),
                       'reason': [r[2] for r in t['frames']], 'path_m': [r[3] for r in t['frames']],
                       'speed_mps': float(np.median([r[5] for r in t['frames'] if r[5] is not None] or [0]))}
                      for t in tr['trials']]
    return q


def nd_full_name():
    """Вся new_data итоговой версии: alarms_nd_final.csv или alarms_final_nd_full.csv -- самый свежий файл
    со всей записью (не отрезки) в output/runs."""
    found = []
    for name in ('alarms_nd_final.csv', 'alarms_final_nd_full.csv'):
        p = RUNS / name
        if p.exists():
            with open(p, encoding='utf-8') as f:
                n = sum(1 for _ in f) - 1
            if n >= 10000:
                found.append((p.stat().st_mtime, name))
    return max(found)[1] if found else 'alarms_final_nd_full.csv'


def speed():
    s = {'deser': DOCUMENTED['deser_ms']}
    d = read_json('check_preproc.json')
    if d:
        s['parse'] = [{'bag': b, 'mb': FRAME_MB.get(b, 8), 'python': v['parse_ms_python'], 'cpp': v['parse_ms_cpp'],
                       'kept_after_crop': v['kept_after_crop']} for b, v in d.items()]
    run = 'docker/doubleT_platform' if has('docker/doubleT_platform_stats.json') else 'smoke/doubleT_platform'
    stats = read_json(f'{run}_stats.json')
    if stats:
        rows = read_jsonl(f'{run}_result.jsonl') or []
        pre = stats.get('preproc', {})
        s['e2e'] = {k: stats.get(k) for k in ('received', 'processed', 'dropped_stale', 'latency_ms_p50', 'latency_ms_p95',
                                              'latency_ms_max', 'e2e_ms_p50', 'e2e_ms_p95', 'e2e_ms_max', 'alarm_frames')}
        s['e2e'].update({'bag': 'doubleT_platform', 'mb': 8, 'rate_hz': 10, 'preproc_published': pre.get('published'),
                         'latency_ms': [round(r['latency_ms'], 2) for r in rows if r.get('latency_ms') is not None],
                         'detect_ms': [round(r['detect_ms'], 2) for r in rows if r.get('detect_ms') is not None],
                         'queue_ms': [round(r['queue_ms'], 2) for r in rows if r.get('queue_ms') is not None]})
        e2e = read_json(f'{run}_e2e.json')
        if e2e and e2e.get('e2e_ms'):
            s['e2e']['e2e_ms'] = [round(v, 2) for v in e2e['e2e_ms']]
        ob = read_json('docker/doubleT_obstacle_stats.json')
        if ob:
            s['e2e_big'] = {k: ob.get(k) for k in ('received', 'processed', 'latency_ms_p50', 'latency_ms_p95', 'e2e_ms_p50', 'e2e_ms_p95')}
        fo = read_json('docker/cloud_with_fake_obj_stats.json')
        if fo:
            s['e2e_fo'] = {k: fo.get(k) for k in ('received', 'processed', 'latency_ms_p50', 'latency_ms_p95', 'e2e_ms_p50', 'e2e_ms_p95')}
        pj = read_json(f'{run}_stats_preproc.json')
        if pj:
            s['e2e']['preproc_ms_p50'], s['e2e']['preproc_ms_p95'] = pj.get('preproc_ms_p50'), pj.get('preproc_ms_p95')
            s['e2e']['transport_ms_p50'], s['e2e']['transport_ms_p95'] = pj.get('transport_ms_p50'), pj.get('transport_ms_p95')
    return s


def count_tests():
    """Тесты по исходникам: gtest -- макросы TEST в ros2_ws, pytest -- функции test_ в tests/."""
    gtest = sum(len(re.findall(r'^TEST(?:_F|_P)?\(', f.read_text(encoding='utf-8'), re.M))
                for f in (ROOT / 'ros2_ws' / 'src').glob('*/test/*.cpp'))
    return {'gtest': gtest, 'pytest': work()['pytest']}


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

    cf = np.clip(fwd, 2, None)
    top = snap.rail_top_at(cf)
    rel = z - top
    cen = snap.center_at(cf)
    zp = zone_params()
    in_zone = (fwd > 2) & (np.abs(lat - cen) <= zp['half_width']) & (rel >= zp['clearance']) & (rel <= zp['height'])
    in_zone_all = in_zone.copy()
    if snap.path_range:
        in_zone &= fwd <= snap.path_range
    tag = np.zeros(len(fwd), np.uint8)
    tag[in_zone] = 1
    objects = []
    for o in res['objects']:
        if o.get('level') is None and o['too_small']:
            continue
        near, far = o['distance_m'], o.get('far_m', o['distance_m'])
        sel = (in_zone_all if o.get('reason') == 'beyond_path' else in_zone) & (fwd >= near - 0.05) & (fwd <= far + 0.05) & (np.abs(lat - cen - o['lateral_m']) < 0.8)
        alarm = o.get('level') == 'stop'
        tag[sel] = 2 if alarm else 3
        box = None
        if sel.any():
            box = [float(lat[sel].min()), float(lat[sel].max()), float(z[sel].min()),
                   float(z[sel].max()), float(fwd[sel].min()), float(fwd[sel].max())]
        elif o.get('held'):
            c = float(snap.center_at(np.array([near]))[0]) + o['lateral_m']
            r = float(snap.rail_top_at(np.array([near]))[0])
            box = [c - 0.2, c + 0.2, r + o.get('low_m', 0.0), r + max(o['height_m'], 0.3), near, near + 0.3]
        if box is not None:
            objects.append({'distance_m': near, 'far_m': far, 'lateral_m': o['lateral_m'], 'height_m': o['height_m'],
                            'n_points': int(o['n_points']), 'confirmed': bool(o['confirmed']), 'alarm': alarm,
                            'level': o.get('level'), 'reason': o.get('reason'),
                            'edge_line': bool(o.get('edge_line')), 'held': bool(o.get('held')), 'box': box})
    objects.sort(key=lambda o: (not o['alarm'], o['distance_m']))

    if bg_points is not None and (tag == 0).sum() > bg_points:
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
            'status': res['status'], 'clear_to_m': res['clear_to_m'], 'caution_distance_m': res['caution_distance_m'],
            'zone': zone_params()}


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
            'host': host(), 'quality': quality(), 'speed': speed(), 'robust': robustness(),
            'fake_obj': fake_obj(), 'experiments': experiments(), 'work': work()}
    data['sources'] = dict(sorted(SOURCES.items()))
    if not args.no_scene:
        try:
            data['scene'] = scene(args.scene_frame)
        except Exception as e:
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
