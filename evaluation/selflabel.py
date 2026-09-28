"""Саморазметка эпизодов СТОП на new_data проездом: если поезд потом проехал место, где стоял
«объект», и не остановился, тревога доказанно ложная. В подбор параметров не входит -- только
утверждение для README: «N из M эпизодов доказанно ложные, остальные не разрешены».

    python -m evaluation alarms --tag nd_P --bags new_data --workers 1
    python -m evaluation selflabel --tag nd_P

Эпизод -- кадры СТОП одного track_id, разрыв до GAP кадров не рвёт. X = travel_m + distance_m на
первом кадре. Сегмент одометрии рвётся, если displacement_m был None дольше 1 с, пробег убыл или
кадры идут не подряд (отрезки alarms.py --segments). Метка false_proven -- в том же сегменте
позже travel_m >= X + 3 + 0,02 * distance_m; иначе unresolved.
"""
import argparse
import csv
import json
import sys
from collections import Counter
from pathlib import Path

import numpy as np

from bags import RUNS, read_problems

GAP = 5
NONE_SEG_S = 1.0
MARGIN_M, MARGIN_REL = 3.0, 0.02
ND_KM = (10.0, 16.0)


def segments(rows):
    seg, out, none_t0, prev = 0, [], None, None
    for r in rows:
        if prev is not None and (r['bag'] != prev['bag'] or r['frame'] != prev['frame'] + 1
                                 or r['travel_m'] < prev['travel_m'] - 1e-6):
            seg, none_t0 = seg + 1, None
        if r['displacement_m'] is None:
            none_t0 = r['t_s'] if none_t0 is None else none_t0
        else:
            if none_t0 is not None and r['t_s'] - none_t0 > NONE_SEG_S:
                seg += 1
            none_t0 = None
        out.append(seg)
        prev = r
    return out


def episodes(stops, gap=GAP):
    by_track = {}
    for s in sorted(stops, key=lambda s: (s[2], s[1])):
        by_track.setdefault(s[2], []).append(s)
    eps = []
    for tid, ss in by_track.items():
        cur = [ss[0]]
        for s in ss[1:]:
            if s[1] - cur[-1][1] - 1 <= gap:
                cur.append(s)
            else:
                eps.append(cur)
                cur = [s]
        eps.append(cur)
    return [{'track_id': e[0][2], 'first_row': e[0][0], 'first_frame': e[0][1], 'last_frame': e[-1][1],
             'frames': len(e), 'distance_m': e[0][3], 'reason': Counter(s[4] for s in e).most_common(1)[0][0]}
            for e in sorted(eps, key=lambda e: e[0][0])]


def label(ep, rows, seg):
    i = ep['first_row']
    x = rows[i]['travel_m'] + ep['distance_m']
    need = x + MARGIN_M + MARGIN_REL * ep['distance_m']
    for j in range(i + 1, len(rows)):
        if seg[j] != seg[i]:
            break
        if rows[j]['travel_m'] >= need:
            return 'false_proven'
    return 'unresolved'


def sanity(n_rows, km, problems):
    """Почему сводке нельзя верить (None -- можно): запись прочитана не вся или пробег вне ND_KM."""
    if problems:
        return 'запись прочитана не вся -- ' + '; '.join(problems)
    if n_rows >= 11000 and not ND_KM[0] <= km <= ND_KM[1]:
        return f'пробег {km:.1f} км вне {ND_KM[0]:.0f}-{ND_KM[1]:.0f} км'
    return None


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--tag', required=True)
    args = ap.parse_args()
    num = lambda v: None if v == '' else float(v)
    rows = [{'bag': r['bag'], 'frame': int(r['frame']), 't_s': float(r['t_s']), 'travel_m': float(r['travel_m']),
             'displacement_m': num(r['displacement_m'])}
            for r in csv.DictReader(open(RUNS / f'alarms_{args.tag}.csv', encoding='utf-8'))]
    index = {(r['bag'], r['frame']): k for k, r in enumerate(rows)}
    stops = []
    with open(RUNS / f'alarms_{args.tag}_objects.jsonl', encoding='utf-8') as f:
        for line in f:
            o = json.loads(line)
            k = index[(o['bag'], o['frame'])]
            stops += [(k, o['frame'], ob['track_id'], ob['distance_m'], ob.get('reason'))
                      for ob in o['objects'] if ob.get('level') == 'stop' and 'track_id' in ob]
    seg = segments(rows)
    eps = episodes(stops)
    for e in eps:
        e['label'] = label(e, rows, seg)
        e['segment'] = seg[e['first_row']]
        e['x_m'] = round(rows[e['first_row']]['travel_m'] + e['distance_m'], 1)
    with open(RUNS / f'selflabel_{args.tag}.csv', 'w', newline='', encoding='utf-8') as f:
        w = csv.DictWriter(f, fieldnames=['track_id', 'first_frame', 'last_frame', 'frames', 'distance_m', 'x_m',
                                          'reason', 'segment', 'label', 'first_row'])
        w.writeheader()
        w.writerows(eps)
    km = sum(max(rows[k]['travel_m'] for k in ks) - min(rows[k]['travel_m'] for k in ks)
             for s in set(seg) for ks in [[k for k, v in enumerate(seg) if v == s]]) / 1000
    why = sanity(len(rows), km, read_problems(args.tag))
    c = Counter(e['label'] for e in eps)
    d = np.array([e['distance_m'] for e in eps]) if eps else np.array([])
    bins = [(0, 40), (40, 80), (80, 130), (130, 1e9)]
    lines = [f'{args.tag}: эпизодов СТОП {len(eps)}, доказанно ложных {c["false_proven"]}, не разрешено {c["unresolved"]}'
             + ('' if why is None else f' -- НЕНАДЁЖНО: {why}'),
             f'пробег по одометрии {km:.1f} км, сегментов {len(set(seg))}',
             'по дистанции: ' + ', '.join(f'{lo:.0f}-{"" if hi > 1e8 else f"{hi:.0f}"} м: {int(((d >= lo) & (d < hi)).sum())}'
                                          for lo, hi in bins),
             'по причине: ' + ', '.join(f'{k}: {v}' for k, v in Counter(e['reason'] for e in eps).items())]
    (RUNS / f'selflabel_{args.tag}.txt').write_text('\n'.join(lines) + '\n', encoding='utf-8')
    print('\n'.join(lines))


if __name__ == '__main__':
    main()
