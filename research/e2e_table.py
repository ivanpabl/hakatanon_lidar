"""Таблица e2e-замеров по каталогам прогонов play.sh / e2e_sidecar.sh.

    python research/e2e_table.py runs/e2e_before runs/e2e_after -- doubleT_obstacle doubleT_platform
Для каждого каталога и записи: кадров в записи / принято / обработано / отброшено,
e2e по latency_probe (публикация кадра -> публикация результата), задержка узла
(приём -> публикация), разбор, detect, очередь -- p50 / p95 / max."""
import json
import sys
from pathlib import Path

import numpy as np


def pct(v):
    v = [x for x in v if x is not None]
    return '—' if not v else f'{np.percentile(v, 50):.1f} / {np.percentile(v, 95):.1f} / {max(v):.1f}'


def main():
    args = sys.argv[1:]
    k = args.index('--')
    dirs, bags = args[:k], args[k + 1:]
    head = ('прогон', 'запись', 'принято', 'обработано', 'отброшено (детектор/приём)', 'e2e, мс', 'узел, мс',
            'разбор, мс', 'detect, мс', 'очередь, мс')
    print('| ' + ' | '.join(head) + ' |')
    print('|' + '---|' * len(head))
    for d in dirs:
        for b in bags:
            p = Path(d)
            try:
                st = json.loads((p / f'{b}_stats.json').read_text())
            except FileNotFoundError:
                continue
            rows = [json.loads(l) for l in open(p / f'{b}_result.jsonl') if l.strip()]
            e2e = json.loads((p / f'{b}_e2e.json').read_text()) if (p / f'{b}_e2e.json').exists() else {}
            pre = (st.get('preproc') or {}).get('dropped', '—')
            e = pct(e2e.get('e2e_ms', [])) if e2e else '—'
            print(f'| {p.name} | {b} | {st["received"]} | {st["processed"]} | {st["dropped_stale"]} / {pre} | {e} | '
                  f'{pct([r["latency_ms"] for r in rows])} | {pct([r["parse_ms"] for r in rows])} | '
                  f'{pct([r["detect_ms"] for r in rows])} | {pct([r["queue_ms"] for r in rows])} |')


if __name__ == '__main__':
    main()
