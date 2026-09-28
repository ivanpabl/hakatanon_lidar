"""Оценка на бэге организаторов cloud_with_fake_obj (#437, #460): 10 синтетических объектов
через ~100 м. Объект привязан к миру координатой X = travel_m + distance_m (пробег поезда по
оценке скорости + дистанция в кадре), окно эталона X +- 5 м.

    python tools/alarms.py --tag fo_base0 --bags cloud_with_fake_obj --workers 1 --all-objects
    python tools/eval_fake_obj.py draft --tag fo_base0     # группы по X -> черновик эталона
    python tools/eval_fake_obj.py eval --tag fo_base       # метрики варианта по эталону

Эталон -- data/cloud_with_fake_obj/objects.yaml (копия в git: reference/cloud_with_fake_obj/objects.yaml):
    objects:
      - {id: 1, desc: '...', expect: stop, x_m: 312.4, lateral_m: 0.02, low_m: 1.40, rail_z_m: -1.07}
expect: stop -- объекты 1, 2, 3, 4, 6, 9, 10 (в габарите); not_stop -- 5, 7, 8.

Метрики по объекту: первый СТОП (дистанция, кадр, sight_m в этом кадре), доля кадров СТОП от
первого до последнего кадра, где объект виден (пока объект впереди), итоговый уровень (высший
за проезд) и причина. Ложный СТОП-кадр -- кадр со status=stop, в котором есть объект stop вне
всех окон. Пробег зависит от оценки скорости: если меняется EgoMotion (задача 13), эталон
строится заново.
"""
import argparse
import csv
import json
import sys
from collections import Counter
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from bags import DATA, FAKE_OBJ, ROOT, RUNS          # noqa: E402

HALF_WINDOW = 5.0
RANK = {None: 0, 'caution': 1, 'stop': 2}
REF_PATHS = [DATA / FAKE_OBJ / 'objects.yaml', ROOT / 'reference' / FAKE_OBJ / 'objects.yaml']


def group_x(points, gap=15.0):
    pts = sorted(points, key=lambda p: p[0])
    groups = []
    for p in pts:
        if groups and p[0] - groups[-1][-1][0] <= gap:
            groups[-1].append(p)
        else:
            groups.append([p])
    return groups


def evaluate(ref, frames, objs, half=HALF_WINDOW):
    per = {r['id']: {'first_frame': None, 'first_stop_m': None, 'sight_m': None,
                     'post': [], 'last_seen_frame': None, 'levels': []} for r in ref}
    false_frames = []
    for fr in frames:
        f, travel = fr['frame'], fr['travel_m']
        seen, outside = {}, False
        for o in objs.get(f, []):
            x = travel + o['distance_m']
            hit = [r for r in ref if abs(x - r['x_m']) <= half]
            if o.get('level') == 'stop' and not hit:
                outside = True
            for r in hit:
                if RANK[o.get('level')] > RANK[seen.get(r['id'], (None,))[0]]:
                    seen[r['id']] = (o.get('level'), o.get('reason'), o)
        if fr['status'] == 'stop' and outside:
            false_frames.append(f)
        for r in ref:
            p = per[r['id']]
            level, reason, o = seen.get(r['id'], (None, None, None))
            if level:
                p['levels'].append((level, reason))
                p['last_seen_frame'] = f
            if level == 'stop' and p['first_frame'] is None:
                p['first_frame'], p['first_stop_m'], p['sight_m'] = f, o['distance_m'], fr['sight_m']
            if p['first_frame'] is not None and travel < r['x_m'] - half:
                p['post'].append((f, level))
    rows = []
    for r in ref:
        p = per[r['id']]
        top = max((lv for lv, _ in p['levels']), key=RANK.get, default=None)
        reasons = Counter(s for lv, s in p['levels'] if lv == top)
        # доля кадров СТОП: от первого СТОП до последнего кадра, где объект виден (иначе
        # в долю попадают кадры без объекта в кадре вовсе -- ср. test_evaluate_first_stop_share).
        post = [t for t in p['post'] if p['last_seen_frame'] is not None and t[0] <= p['last_seen_frame']]
        stop_share = round(sum(lv == 'stop' for _, lv in post) / len(post), 2) if post else None
        rows.append({'id': r['id'], 'desc': r.get('desc', ''), 'expect': r['expect'], 'level': top or 'none',
                     'reason': reasons.most_common(1)[0][0] if reasons else '',
                     'first_stop_m': p['first_stop_m'], 'first_frame': p['first_frame'], 'sight_m': p['sight_m'],
                     'stop_share': stop_share,
                     'ok': (top == 'stop') == (r['expect'] == 'stop')})
    return rows, false_frames


def summary_line(tag, rows, false_frames, n_frames):
    need = [r for r in rows if r['expect'] == 'stop']
    got = sum(r['level'] == 'stop' for r in need)
    bad = [str(r['id']) for r in rows if r['expect'] != 'stop' and r['level'] == 'stop']
    firsts = ' '.join(f"{r['id']}:{r['first_stop_m']:.0f}м" for r in need if r['first_stop_m'] is not None)
    return (f'fake_obj {tag}: в габарите СТОП {got}/{len(need)}; СТОП вне габарита: {", ".join(bad) or "нет"}; '
            f'ложных СТОП-кадров {len(false_frames)} из {n_frames}; первые СТОП {firsts}')


def _num(v):
    return None if v in ('', None) else float(v)


def load_run(tag):
    frames = [{'frame': int(r['frame']), 'travel_m': float(r['travel_m']), 'status': r['status'],
               'sight_m': _num(r['sight_m'])}
              for r in csv.DictReader(open(RUNS / f'alarms_{tag}.csv', encoding='utf-8')) if r['bag'] == FAKE_OBJ]
    objs = {}
    with open(RUNS / f'alarms_{tag}_objects.jsonl', encoding='utf-8') as f:
        for line in f:
            o = json.loads(line)
            if o['bag'] == FAKE_OBJ:
                objs[o['frame']] = o['objects']
    return frames, objs


def load_ref(path=None):
    from ruamel.yaml import YAML
    path = Path(path) if path else next(p for p in REF_PATHS if p.exists())
    return YAML(typ='safe').load(path.read_text(encoding='utf-8'))['objects']


def draft(tag):
    frames, objs = load_run(tag)
    travel = {f['frame']: f['travel_m'] for f in frames}
    pts = [(travel[k] + o['distance_m'], k, o) for k, os in objs.items() for o in os]
    groups = [g for g in group_x(pts) if len({p[1] for p in g}) >= 3]
    print(f'кадров {len(frames)}, пробег {frames[-1]["travel_m"]:.0f} м, групп (>= 3 кадров) {len(groups)}')
    print(f'{"#":>3}{"X, м":>9}{"шаг":>7}{"кадры":>12}{"lat":>7}{"low":>7}{"height":>8}{"rail_z":>8}{"уровень":>9}')
    out, prev = [], None
    for k, g in enumerate(groups, 1):
        med = lambda key: float(np.median([p[2][key] for p in g if key in p[2]])) if any(key in p[2] for p in g) else None
        x = float(np.median([p[0] for p in g]))
        top = max((p[2].get('level') for p in g), key=RANK.get)
        fr = sorted({p[1] for p in g})
        print(f'{k:>3}{x:>9.1f}{"" if prev is None else f"{x - prev:.0f}":>7}{f"{fr[0]}-{fr[-1]}":>12}'
              f'{med("lateral_m"):>7.2f}{med("low_m"):>7.2f}{med("height_m"):>8.2f}{med("rail_z_m"):>8.2f}{str(top):>9}')
        out.append({'id': k, 'desc': '', 'expect': 'stop', 'x_m': round(x, 1), 'lateral_m': round(med('lateral_m'), 2),
                    'low_m': round(med('low_m'), 2), 'rail_z_m': round(med('rail_z_m'), 2),
                    'frames': f'{fr[0]}-{fr[-1]}'})
        prev = x
    from ruamel.yaml import YAML
    path = RUNS / f'fake_obj_draft_{tag}.yaml'
    with open(path, 'w', encoding='utf-8') as f:
        YAML().dump({'objects': out}, f)
    print(f'черновик: {path} -- сверить с #437/#460 и show_frame.py, заполнить desc/expect, сохранить как '
          f'{REF_PATHS[0]} и {REF_PATHS[1]}')


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('cmd', choices=['draft', 'eval'])
    ap.add_argument('--tag', required=True, help='тег прогона alarms.py по cloud_with_fake_obj')
    ap.add_argument('--ref', default=None, help='эталон objects.yaml (по умолчанию data/, затем reference/)')
    args = ap.parse_args()
    if args.cmd == 'draft':
        return draft(args.tag)
    frames, objs = load_run(args.tag)
    rows, false_frames = evaluate(load_ref(args.ref), frames, objs)
    with open(RUNS / f'fake_obj_{args.tag}.csv', 'w', newline='', encoding='utf-8') as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)
    for r in rows:
        first = '' if r['first_stop_m'] is None else f"{r['first_stop_m']:.0f} м (видимость {r['sight_m']:.0f} м)"
        print(f"{r['id']:>3} {r['expect']:<9}{r['level']:<9}{r['reason']:<13}{first}")
    line = summary_line(args.tag, rows, false_frames, len(frames))
    (RUNS / f'fake_obj_{args.tag}.txt').write_text(line + '\n', encoding='utf-8')
    print(line)


if __name__ == '__main__':
    main()
