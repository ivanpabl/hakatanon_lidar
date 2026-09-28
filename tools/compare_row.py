"""Строка сводной таблицы вариантов детектора по результатам прогона:
    python tools/compare_row.py <tag> '<json kwargs>'  -> печать + строка в runs/compare_table.md
Колонки: объекты организаторов (N из 7, СТОП на 5/7/8, ложные кадры), % кадров СТОП/ВНИМ./unknown
на 5 пустых записях и на new_data, человек со 170 м и куб 0,4 м (медиана первого СТОП),
первый СТОП на объекте 56 м в doubleT_obstacle (кадр, дистанция)."""
import csv
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from bags import EMPTY_BAGS, RUNS, read_problems            # noqa: E402

HEADER = ('| тег | параметры | объекты организаторов | пустые СТОП/ВНИМ./unk, % | new_data СТОП/ВНИМ./unk, % '
          '| человек 170 м | куб 0,4 м | 56 м: кадр/дист. |\n|---|---|---|---|---|---|---|---|\n')


def _read(path):
    return list(csv.DictReader(open(path, encoding='utf-8'))) if path.exists() else []


def shares(rows):
    n = max(len(rows), 1)
    return tuple(100.0 * sum(r['status'] == s for r in rows) / n for s in ('stop', 'caution', 'unknown'))


def first_stop_56(rows, lo=45.0, hi=65.0):
    for r in rows:
        if r['bag'] == 'doubleT_obstacle' and r['status'] == 'stop' and r['distance_m'] and lo <= float(r['distance_m']) <= hi:
            return int(r['frame']), float(r['distance_m'])
    return None


def approach_median(rows, shape, start=None):
    d = [float(r['first_m']) for r in rows
         if r['shape'] == shape and r['detected'] == '1' and (start is None or int(r['start_m']) == start)]
    return float(np.median(d)) if d else None


def row(tag, det):
    alarms = _read(RUNS / f'alarms_{tag}.csv')
    nd = _read(RUNS / f'alarms_nd_{tag}.csv')
    ap = _read(RUNS / f'approach_{tag}.csv')
    fo = RUNS / f'fake_obj_fo_{tag}.txt'
    fo = fo.read_text(encoding='utf-8').strip().split(': ', 1)[-1] if fo.exists() else '-'
    pct = lambda t: '/'.join(f'{v:.1f}' for v in t)
    f56 = first_stop_56(alarms)
    m = lambda v: '-' if v is None else f'{v:.0f} м'
    empty = [r for r in alarms if r['bag'] in EMPTY_BAGS]
    probs = [m for t in (tag, f'nd_{tag}', f'fo_{tag}') if (RUNS / f'alarms_{t}.csv').exists() or (RUNS / f'alarms_{t}_read.json').exists()
             for m in read_problems(t, RUNS)]
    name = tag + (f' НЕДОЧИТАНО: {"; ".join(probs)}' if probs else '')
    return (f'| {name} | `{det}` | {fo} | {pct(shares(empty)) if empty else "-"} '
            f'| {pct(shares(nd)) if nd else "-"} | {m(approach_median(ap, "человек стоит", 170))} '
            f'| {m(approach_median(ap, "куб 0.4"))} | {"-" if f56 is None else f"{f56[0]}/{f56[1]:.1f}"} |')


def main():
    tag, det = sys.argv[1], (sys.argv[2] if len(sys.argv) > 2 else '{}')
    line = row(tag, det)
    table = RUNS / 'compare_table.md'
    if not table.exists():
        table.write_text(HEADER, encoding='utf-8')
    with open(table, 'a', encoding='utf-8') as f:
        f.write(line + '\n')
    print(line)


if __name__ == '__main__':
    main()
