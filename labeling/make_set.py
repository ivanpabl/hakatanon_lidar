"""Набор кадров для ручной разметки препятствий, разбитый на участников.

Два шага:

  render -- прогоняет детектор по записи, берёт каждый STEP-й кадр (плюс по
            одному кадру от коротких тревог, не попавших на шаг) и рисует для
            него картинку: "выпрямленный" коридор вдоль оси пути (ближняя и
            дальняя часть), профиль высоты и вид сверху. Пишет
            data/labeling/frames/<bag>/f_<кадр>.png и мета-данные part_<start>.json.

  split  -- раскладывает все отрисованные кадры на N участников: связные куски
            по времени, общий калибровочный набор для всех и перекрытие с
            соседним участником. Каждая папка data/labeling/participant_<k>/
            самодостаточна (index.html + manifest.js + img/) -- её можно
            заархивировать и отдать человеку.

Примеры:
    python labeling/make_set.py render --bag new_data --range 0:2300
    python labeling/make_set.py render --bag doubleT_obstacle
    python labeling/make_set.py split --participants 4

Разметка -- сцены, а не тревог: человек отмечает объекты, которые реально
есть на пути, с их положением. Поэтому картинки не показывают решений
детектора -- только ось пути, по которой выпрямлен коридор.
"""
import argparse
import json
import random
import shutil
import sys
import warnings
from collections import deque
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / 'tools'))
from bags import DATA, open_cloud_bag
from viz.render import render_frame

OUT = DATA / 'labeling'
STEP = 10
WARMUP = 20
CALIB_N = 40
OVERLAP = 0.12


def cmd_render(args):
    import sys
    from tunnel_od import ObstacleDetector, parse_pointcloud2
    warnings.simplefilter('ignore')

    start, end = 0, None
    if args.range:
        a, b = args.range.split(':')
        start, end = int(a), (int(b) if b else None)
    out_dir = OUT / 'frames' / args.bag
    out_dir.mkdir(parents=True, exist_ok=True)

    det = ObstacleDetector()
    buf = deque(maxlen=STEP + 2)
    event, frames = None, []
    with open_cloud_bag(args.bag) as (reader, conn):
        t0 = None
        for i, (c, t, raw) in enumerate(reader.messages(connections=[conn])):
            if t0 is None:
                t0 = t
            if i < start - WARMUP:
                continue
            if end is not None and i >= end:
                break
            msg = reader.deserialize(raw, c.msgtype)
            x, y, z = parse_pointcloud2(msg.data, msg.point_step, msg.fields)
            res = det.detect(x, y, z, refit_path=True, stamp=t / 1e9)
            if i < start:
                continue
            rec = {'frame': i, 't': (t - t0) / 1e9, 'alarm': res['obstacle']}
            buf.append((rec, x, y, z, det.track_path()))

            todo = []
            if i % STEP == 0:
                todo.append((buf[-1], 'step'))
            if res['obstacle']:
                if event is None:
                    event = {'first': i, 'covered': False}
                event['covered'] |= i % STEP == 0
            elif event is not None:
                if not event['covered']:
                    first = next((b for b in buf if b[0]['frame'] == event['first']), None)
                    if first is not None:
                        todo.append((first, 'short_alarm'))
                event = None

            for (r, fx, fy, fz, snap), why in todo:
                name = f"f_{r['frame']:06d}.png"
                title = f"{args.bag}   кадр {r['frame']}   t={r['t']:.1f} с"
                geom = render_frame(out_dir / name, fx, fy, fz, snap, title)
                frames.append({**r, 'why': why, 'img': name, **geom})
            if i % 500 == 0:
                print(f'{args.bag} [{start}:{end}] кадр {i}, отрисовано {len(frames)}', flush=True)

    (out_dir / f'part_{start:06d}.json').write_text(json.dumps(frames))
    print(f'{args.bag} [{start}:{end}]: готово, {len(frames)} кадров')


def cmd_split(args):
    items = []
    for bag_dir in sorted((OUT / 'frames').iterdir()):
        for part in sorted(bag_dir.glob('part_*.json')):
            for f in json.loads(part.read_text()):
                items.append({**f, 'bag': bag_dir.name})
    order = {'new_data': 0}
    items.sort(key=lambda f: (order.get(f['bag'], 1), f['bag'], f['frame']))
    seen = set()
    items = [f for f in items if not ((f['bag'], f['frame']) in seen or seen.add((f['bag'], f['frame'])))]
    for k, f in enumerate(items):
        f['id'] = f"{f['bag']}:{f['frame']}"

    rng = random.Random(0)
    alarm = [f for f in items if f['alarm']]
    quiet = [f for f in items if not f['alarm']]
    calib = rng.sample(alarm, min(CALIB_N // 2, len(alarm)))
    calib += rng.sample(quiet, min(CALIB_N - len(calib), len(quiet)))
    calib_ids = {f['id'] for f in calib}
    rng.shuffle(calib)

    main = [f for f in items if f['id'] not in calib_ids]
    n = args.participants
    bounds = np.linspace(0, len(main), n + 1).astype(int)
    chunks = [main[bounds[k]:bounds[k + 1]] for k in range(n)]
    tool = Path(__file__).parent / 'tool.html'
    summary = []
    for k in range(n):
        nxt = chunks[(k + 1) % n]
        overlap = nxt[:int(len(nxt) * OVERLAP)]
        sets = [('calib', calib), ('main', chunks[k]), ('overlap', overlap)]
        pdir = OUT / f'participant_{k + 1}'
        if pdir.exists():
            shutil.rmtree(pdir)
        (pdir / 'img').mkdir(parents=True)
        manifest = []
        for set_name, fs in sets:
            for f in fs:
                img = f"{f['bag']}_{f['img']}"
                shutil.copy(OUT / 'frames' / f['bag'] / f['img'], pdir / 'img' / img)
                manifest.append({'id': f['id'], 'bag': f['bag'], 'frame': f['frame'], 't': round(f['t'], 1),
                                 'set': set_name, 'img': 'img/' + img, 'w': f['w'], 'h': f['h'],
                                 'panels': f['panels']})
        (pdir / 'manifest.js').write_text(
            'window.MANIFEST = ' + json.dumps({'participant': k + 1, 'items': manifest}, ensure_ascii=False) + ';\n')
        shutil.copy(tool, pdir / 'index.html')
        summary.append((k + 1, len(calib), len(chunks[k]), len(overlap)))

    print(f'Всего кадров: {len(items)} (калибровка {len(calib)}, основных {len(main)})')
    for p, c, m, o in summary:
        print(f'  участник {p}: калибровка {c} + свой кусок {m} + перекрытие {o} = {c + m + o} кадров')


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest='cmd', required=True)
    r = sub.add_parser('render')
    r.add_argument('--bag', required=True, help="new_data или имя записи из data/Датасет/archive/for_hackathon")
    r.add_argument('--range', help='диапазон кадров start:end (для параллельного запуска)')
    s = sub.add_parser('split')
    s.add_argument('--participants', type=int, default=4)
    args = ap.parse_args()
    {'render': cmd_render, 'split': cmd_split}[args.cmd](args)


if __name__ == '__main__':
    main()
