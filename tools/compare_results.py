"""Сравнение результатов детектора из двух прогонов (JSONL из result_file узла).

Кадры сопоставляются по полю stamp (header.stamp). Для записи в контракте v1, где штампы
сдвинуты к настоящему времени, сдвиг находится сам (--shift auto) или задаётся в секундах.
Поля obstacle, distance_m, objects сравниваются точно (JSON хранит float64 без потерь);
по остальным полям детектора -- сколько кадров отличается.

    python tools/compare_results.py runs/before/doubleT_obstacle_result.jsonl \\
                                    runs/after/doubleT_obstacle_result.jsonl
Код возврата 0 -- обязательные поля совпали на всех общих кадрах и кадры не потеряны.
"""
import argparse
import json
import sys

import numpy as np

REQUIRED = ('obstacle', 'distance_m', 'objects')
INFO = ('n_points', 'path_available', 'gauge_m', 'path_age_frames', 'path_range_m', 'speed_mps',
        'displacement_m', 'refit_path')


def load(path):
    rows = [json.loads(line) for line in open(path, encoding='utf-8') if line.strip()]
    return {r['stamp']: r for r in rows}


def find_shift(a, b):
    sa, sb = np.array(sorted(a)), np.array(sorted(b))
    if abs(np.median(sa) - np.median(sb)) < 1.0:
        return 0.0
    best = (0, 0.0)
    for cand in {round(x - y, 3) for x in sb[:20] for y in sa[:20]}:
        idx = np.clip(np.searchsorted(sb, sa + cand), 1, len(sb) - 1)
        near = np.minimum(np.abs(sb[idx] - sa - cand), np.abs(sb[idx - 1] - sa - cand))
        hits = int((near < 1e-3).sum())
        if hits > best[0]:
            best = (hits, cand)
    return best[1]


def compare(a, b, shift=0.0, tol=1e-3):
    sb = np.array(sorted(b))
    pairs, only_a = [], []
    for s in sorted(a):
        j = int(np.clip(np.searchsorted(sb, s + shift), 1, len(sb) - 1)) if len(sb) > 1 else 0
        k = min((j - 1, j), key=lambda i: abs(sb[i] - s - shift)) if len(sb) > 1 else 0
        if len(sb) and abs(sb[k] - s - shift) < tol:
            pairs.append((a[s], b[sb[k]]))
        else:
            only_a.append(s)
    matched_b = {id(rb) for _, rb in pairs}
    only_b = [s for s, r in b.items() if id(r) not in matched_b]
    diff = {k: 0 for k in REQUIRED + INFO}
    first = {}
    for ra, rb in pairs:
        for k in REQUIRED + INFO:
            if ra.get(k) != rb.get(k):
                diff[k] += 1
                first.setdefault(k, (ra.get('frame'), ra.get(k), rb.get(k)))
    return {'frames_a': len(a), 'frames_b': len(b), 'matched': len(pairs), 'only_a': len(only_a),
            'only_b': len(only_b), 'diff': diff, 'first_diff': first, 'shift_s': shift,
            'alarm_frames_a': sum(1 for r, _ in pairs if r.get('obstacle')),
            'alarm_frames_b': sum(1 for _, r in pairs if r.get('obstacle'))}


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('a')
    ap.add_argument('b')
    ap.add_argument('--shift', default='auto', help='сдвиг штампов b - a, с, или auto')
    ap.add_argument('--allow-missing', action='store_true', help='не считать ошибкой кадры только в одном файле')
    args = ap.parse_args()
    a, b = load(args.a), load(args.b)
    shift = find_shift(a, b) if args.shift == 'auto' else float(args.shift)
    r = compare(a, b, shift)
    print(f'кадров: {r["frames_a"]} / {r["frames_b"]}, общих {r["matched"]}, только в первом {r["only_a"]}, '
          f'только во втором {r["only_b"]} (сдвиг штампов {r["shift_s"]:.3f} с)')
    print(f'кадров с тревогой (общие): {r["alarm_frames_a"]} / {r["alarm_frames_b"]}')
    for k, n in r['diff'].items():
        mark = 'ОБЯЗАТЕЛЬНОЕ' if k in REQUIRED else ''
        print(f'  {k:<16} отличается в {n:>4} кадрах  {mark if n else ""}')
    for k, (fr, va, vb) in r['first_diff'].items():
        print(f'  первое отличие {k} (кадр {fr}): {str(va)[:120]} | {str(vb)[:120]}')
    bad = any(r['diff'][k] for k in REQUIRED) or r['matched'] == 0 or (
        not args.allow_missing and (r['only_a'] or r['only_b']))
    print('ИТОГ:', 'совпадает' if not bad else 'РАЗЛИЧАЕТСЯ')
    sys.exit(1 if bad else 0)


if __name__ == '__main__':
    main()
