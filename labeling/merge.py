"""Сводит CSV разметки от участников в один файл и проверяет согласие.

    python labeling/merge.py data/labeling/results/*.csv

Пишет data/labeling/labels_merged.csv и печатает:
- согласие на калибровочных кадрах (их размечали все) и на перекрытии (двое);
- список спорных кадров -- их стоит разобрать вместе и поправить вручную.

Вердикты: empty (пусто), obstacle (препятствие), unclear (неясно).
"""
import csv
import sys
from collections import Counter, defaultdict
from itertools import combinations
from pathlib import Path

OUT = Path(__file__).resolve().parent.parent / 'data' / 'labeling' / 'labels_merged.csv'
SAME_OBJECT_FWD = 3.0
SAME_OBJECT_LAT = 0.6


def parse_objects(s):
    out = []
    for part in filter(None, s.split('|')):
        fwd, lat, cls = part.split(';')
        out.append((float(fwd), float(lat), cls))
    return out


def objects_match(a, b):
    """Каждый объект одного разметчика нашёлся у другого (и наоборот)."""
    def covered(xs, ys):
        return all(any(abs(x[0] - y[0]) < SAME_OBJECT_FWD and abs(x[1] - y[1]) < SAME_OBJECT_LAT for y in ys) for x in xs)
    return covered(a, b) and covered(b, a)


def main(paths):
    by_id = defaultdict(list)
    meta = {}
    for p in paths:
        with open(p, encoding='utf-8-sig') as f:
            for row in csv.DictReader(f):
                if not row['verdict']:
                    continue
                by_id[row['id']].append(row)
                meta[row['id']] = (row['bag'], int(row['frame']), row['t'])

    stats = {'calib': [0, 0], 'overlap': [0, 0]}
    disputed = []
    merged = []
    for id_, rows in by_id.items():
        verdicts = [r['verdict'] for r in rows]
        agree = len(set(verdicts)) == 1
        if agree and verdicts[0] == 'obstacle':
            objs = [parse_objects(r['objects']) for r in rows]
            agree = all(objects_match(a, b) for a, b in combinations(objs, 2))
        kind = rows[0]['set'] if len(rows) > 1 else 'single'
        if kind in stats:
            stats[kind][0] += 1
            stats[kind][1] += agree
        if agree:
            verdict = verdicts[0]
        else:
            top, n = Counter(verdicts).most_common(1)[0]
            verdict = top if n > len(verdicts) / 2 and top != 'obstacle' else 'disputed'
            if verdict == 'disputed':
                disputed.append((id_, verdicts))
        bag, frame, t = meta[id_]
        merged.append({'id': id_, 'bag': bag, 'frame': frame, 't': t, 'verdict': verdict,
                       'axis_wrong': int(any(r['axis_wrong'] == '1' for r in rows)),
                       'objects': rows[0]['objects'], 'n_labelers': len(rows),
                       'labelers': ' '.join(r['participant'] for r in rows),
                       'comments': ' / '.join(r['comment'] for r in rows if r['comment'])})

    merged.sort(key=lambda r: (r['bag'], r['frame']))
    OUT.parent.mkdir(parents=True, exist_ok=True)
    with open(OUT, 'w', newline='', encoding='utf-8') as f:
        w = csv.DictWriter(f, fieldnames=list(merged[0].keys()))
        w.writeheader()
        w.writerows(merged)

    c = Counter(r['verdict'] for r in merged)
    print(f'Кадров с разметкой: {len(merged)}  ({dict(c)})')
    for kind, (n, ok) in stats.items():
        if n:
            print(f'Согласие ({kind}): {ok}/{n} = {100 * ok / n:.0f}%')
    print(f'Спорных кадров: {len(disputed)}')
    for id_, v in disputed[:30]:
        print(f'  {id_}: {v}')
    print(f'Записано: {OUT}')


if __name__ == '__main__':
    if len(sys.argv) < 2:
        sys.exit(__doc__)
    main(sys.argv[1:])
