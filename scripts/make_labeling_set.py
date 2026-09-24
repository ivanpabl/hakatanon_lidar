"""Набор кадров для ручной разметки препятствий, разбитый на участников.

Два шага:

  render -- прогоняет детектор по записи, берёт каждый STEP-й кадр (плюс по
            одному кадру от коротких тревог, не попавших на шаг) и рисует для
            него картинку: "выпрямленный" коридор вдоль оси пути (ближняя и
            дальняя часть), профиль высоты и вид сверху. Пишет
            labeling/frames/<bag>/f_<кадр>.png и мета-данные part_<start>.json.

  split  -- раскладывает все отрисованные кадры на N участников: связные куски
            по времени, общий калибровочный набор для всех и перекрытие с
            соседним участником. Каждая папка labeling/participant_<k>/
            самодостаточна (index.html + manifest.js + img/) -- её можно
            заархивировать и отдать человеку.

Примеры:
    python scripts/make_labeling_set.py render --bag new_data --range 0:2300
    python scripts/make_labeling_set.py render --bag doubleT_obstacle
    python scripts/make_labeling_set.py split --participants 4

Разметка -- сцены, а не тревог: человек отмечает объекты, которые реально
есть на пути, с их положением. Поэтому картинки не показывают решений
детектора -- только ось пути, по которой выпрямлен коридор.
"""
import argparse
import json
import random
import shutil
import warnings
from collections import deque
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent.parent
DATASET = ROOT / 'Датасет' / 'archive' / 'for_hackathon'
OUT = ROOT / 'labeling'
STEP = 10
WARMUP = 20            # кадров прогрева детектора перед диапазоном --range
LAT_VIEW = 3.0         # полуширина выпрямленного коридора на картинке, м
NEAR_VIEW = (2.0, 50.0)
FAR_VIEW = (50.0, 150.0)
H_VIEW = (-0.3, 2.5)   # цветовая шкала: высота над головкой рельса
CORRIDOR_HALF = 1.0    # линии на картинке -- только ориентир, не граница разметки
CALIB_N = 40
OVERLAP = 0.12


def bag_path(name):
    return ROOT / name if name == 'new_data' else DATASET / name


# --- render -------------------------------------------------------------------

def _snapshot(det):
    """То, что нужно для отрисовки кадра: ось пути и высота головки рельса."""
    return {
        'fit_fwd': None if det._fit_fwd is None else det._fit_fwd.copy(),
        'cl': None if det._fitted_cl is None else det._fitted_cl.copy(),
        'bed': None if det._bed is None else (det._bed[0].copy(), det._bed[1].copy()),
        'rail_prof': None if det._rail_prof is None else (det._rail_prof[0].copy(), det._rail_prof[1].copy()),
        'rail_offset': det.rail_offset,
        'path_range': det._path_range,
    }


def _center(snap, fwd):
    if snap['cl'] is None:
        return np.zeros_like(fwd)
    return np.interp(fwd, snap['fit_fwd'], snap['cl'])


def _tor(snap, fwd):
    import detector_interface as di
    tor = di.bed_at(fwd, snap['bed']) + snap['rail_offset']
    if snap['rail_prof'] is not None:
        rf, rz = snap['rail_prof']
        inside = fwd <= rf[-1]
        tor[inside] = np.interp(fwd[inside], rf, rz)
    return tor


def render_frame(fig_path, x, y, z, snap, title):
    """Рисует кадр. Возвращает положение кликабельных панелей в пикселях картинки."""
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    fwd = -y
    keep = (fwd > 0) & (fwd < FAR_VIEW[1])
    x, fwd, z = x[keep], fwd[keep], z[keep]
    lat = x - _center(snap, fwd)
    h = z - _tor(snap, fwd)

    fig = plt.figure(figsize=(18, 11), dpi=80)
    gs = fig.add_gridspec(2, 4, width_ratios=[1, 1, 1.25, 1.25], height_ratios=[1, 1],
                          left=0.04, right=0.985, top=0.93, bottom=0.06, wspace=0.28, hspace=0.25)
    fig.suptitle(title, fontsize=13)
    cmap = 'turbo'
    panels = []
    path_range = snap['path_range'] or 0.0

    for col, (f0, f1), label, size in ((0, NEAR_VIEW, 'ближняя зона', 2.0), (1, FAR_VIEW, 'дальняя зона', 4.0)):
        ax = fig.add_subplot(gs[:, col])
        s = (fwd >= f0) & (fwd <= f1) & (np.abs(lat) < LAT_VIEW) & (h > H_VIEW[0] - 0.5) & (h < 4.0)
        o = np.argsort(h[s])  # высокие точки поверх низких
        ax.scatter(lat[s][o], fwd[s][o], c=np.clip(h[s][o], *H_VIEW), s=size, cmap=cmap,
                   vmin=H_VIEW[0], vmax=H_VIEW[1], linewidths=0, rasterized=True)
        for v in (-CORRIDOR_HALF, CORRIDOR_HALF):
            ax.axvline(v, color='k', lw=0.8, ls='--')
        for v in (-0.76, 0.76):
            ax.axvline(v, color='gray', lw=0.5, ls=':')
        if path_range < f1:
            ax.axhspan(max(path_range, f0), f1, color='gray', alpha=0.15)
            ax.text(0, max(path_range, f0) + 1, 'ось пути здесь -- догадка', ha='center', fontsize=8, color='dimgray')
        ax.set_xlim(-LAT_VIEW, LAT_VIEW)
        ax.set_ylim(f0, f1)
        ax.set_xlabel('от оси пути вбок, м')
        ax.set_ylabel('вперёд, м')
        ax.set_title(f'Коридор, {label} (кликать сюда)', fontsize=10)
        ax.grid(alpha=0.25)
        panels.append((ax, f0, f1))

    ax = fig.add_subplot(gs[0, 2:])
    s = (fwd > NEAR_VIEW[0]) & (np.abs(lat) < 1.5) & (h > -1.0) & (h < 4.0)
    sc = ax.scatter(fwd[s], h[s], c=np.clip(h[s], *H_VIEW), s=1.5, cmap=cmap, vmin=H_VIEW[0], vmax=H_VIEW[1],
                    linewidths=0, rasterized=True)
    ax.axhline(0, color='k', lw=0.6)
    ax.set_xlim(0, FAR_VIEW[1]); ax.set_ylim(-1.0, 3.5)
    ax.set_xlabel('вперёд, м'); ax.set_ylabel('высота над головкой рельса, м')
    ax.set_title('Профиль: точки не дальше 1,5 м от оси', fontsize=10)
    ax.grid(alpha=0.25)
    fig.colorbar(sc, ax=ax, label='высота над рельсом, м', pad=0.01)

    ax = fig.add_subplot(gs[1, 2:])
    s = (np.abs(x) < 10) & (h > -1.0) & (h < 4.0)
    ax.scatter(fwd[s], x[s], c=np.clip(h[s], *H_VIEW), s=0.8, cmap=cmap, vmin=H_VIEW[0], vmax=H_VIEW[1],
               linewidths=0, rasterized=True)
    ff = np.linspace(2, FAR_VIEW[1], 200)
    cc = _center(snap, ff)
    ax.plot(ff, cc, 'k-', lw=1)
    ax.plot(ff, cc - CORRIDOR_HALF, 'k--', lw=0.6)
    ax.plot(ff, cc + CORRIDOR_HALF, 'k--', lw=0.6)
    ax.set_xlim(0, FAR_VIEW[1]); ax.set_ylim(-10, 10)
    ax.set_xlabel('вперёд, м'); ax.set_ylabel('вбок, м')
    ax.set_title('Вид сверху как есть: проверить, что ось пути (линия) лежит на пути', fontsize=10)
    ax.grid(alpha=0.25)

    fig.canvas.draw()
    W, H = fig.canvas.get_width_height()
    meta = []
    for ax, f0, f1 in panels:
        bb = ax.get_window_extent()
        meta.append({'x0': bb.x0, 'x1': bb.x1, 'y_top': H - bb.y1, 'y_bottom': H - bb.y0,
                     'lat': [-LAT_VIEW, LAT_VIEW], 'fwd': [f0, f1]})
    fig.savefig(fig_path, dpi=80)
    plt.close(fig)
    return {'w': W, 'h': H, 'panels': meta}


def cmd_render(args):
    import sys
    sys.path.insert(0, str(Path(__file__).parent))
    from rosbags.highlevel import AnyReader
    from rosbags.typesys import Stores, get_typestore
    import detector_interface as di
    warnings.simplefilter('ignore')

    start, end = 0, None
    if args.range:
        a, b = args.range.split(':')
        start, end = int(a), (int(b) if b else None)
    out_dir = OUT / 'frames' / args.bag
    out_dir.mkdir(parents=True, exist_ok=True)

    det = di.ObstacleDetector()
    buf = deque(maxlen=STEP + 2)   # последние кадры -- чтобы отрисовать короткую тревогу задним числом
    event, frames = None, []
    ts = get_typestore(Stores.ROS2_HUMBLE)
    with AnyReader([bag_path(args.bag)], default_typestore=ts) as reader:
        conn = [c for c in reader.connections if c.msgtype == 'sensor_msgs/msg/PointCloud2'][0]
        t0 = None
        for i, (c, t, raw) in enumerate(reader.messages(connections=[conn])):
            if t0 is None:
                t0 = t
            if i < start - WARMUP:
                continue
            if end is not None and i >= end:
                break
            msg = reader.deserialize(raw, c.msgtype)
            x, y, z = di.parse_pointcloud2(msg.data, msg.point_step, msg.fields)
            res = det.detect(x, y, z, refit_path=True, stamp=t / 1e9)
            if i < start:
                continue
            rec = {'frame': i, 't': (t - t0) / 1e9, 'alarm': res['obstacle']}
            buf.append((rec, x, y, z, _snapshot(det)))

            todo = []
            if i % STEP == 0:
                todo.append((buf[-1], 'step'))
            # тревога, целиком проскочившая между кадрами шага, -- берём её первый кадр
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


# --- split --------------------------------------------------------------------

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

    # калибровка: одинаковые кадры у всех -- половина с тревогой детектора, половина без
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
    tool = Path(__file__).parent / 'labeling_tool.html'
    summary = []
    for k in range(n):
        nxt = chunks[(k + 1) % n]
        overlap = nxt[:int(len(nxt) * OVERLAP)]  # начало куска соседа -- размечают двое
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
    r.add_argument('--bag', required=True, help="new_data или имя записи из Датасет/archive/for_hackathon")
    r.add_argument('--range', help='диапазон кадров start:end (для параллельного запуска)')
    s = sub.add_parser('split')
    s.add_argument('--participants', type=int, default=4)
    args = ap.parse_args()
    {'render': cmd_render, 'split': cmd_split}[args.cmd](args)


if __name__ == '__main__':
    main()
