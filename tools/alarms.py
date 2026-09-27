"""Прогон детектора по реальным записям без вставок: сколько тревог он поднимает.

Сохраняет результат по каждому кадру, чтобы после изменений детектора сравнить
версии на тех же кадрах (какие тревоги появились / исчезли).

    python tools/alarms.py --tag v2                       # все записи -> runs/alarms_v2.csv
    python tools/alarms.py --tag v2g --corridor gauge     # габарит вместо прямоугольника +-1м
    python tools/alarms.py --tag v2n --minpts off         # без порога числа точек
    python tools/alarms.py --tag test --bags roundT_doubleT

Эпизод тревоги -- подряд идущие кадры с тревогой (разрывы до GAP кадров склеиваются).
Какие из тревог ложные, скрипт не знает: заведомо чистых записей нет
(в doubleT_obstacle есть объект на ~56м).
"""
import argparse
import os
for _v in ('OMP_NUM_THREADS', 'OPENBLAS_NUM_THREADS', 'MKL_NUM_THREADS'):
    os.environ.setdefault(_v, '1')
import csv
import json
import warnings
from multiprocessing import Pool

import numpy as np

from tunnel_od import GAUGE_METRO, ObstacleDetector
from tunnel_od.pointcloud import FrameRepeat

from bags import BAGS, EMPTY_BAGS, RUNS as OUT, cloud_parser, open_cloud_bag, safe_messages
GAP = 2


CORRIDORS = {'rect': None, 'gauge': GAUGE_METRO}
MINPTS = {'on': {}, 'off': {'min_points_k': 0.0, 'min_points_floor': 1}}

OBJ_KEYS = ('track_id', 'distance_m', 'lateral_m', 'height_m', 'low_m', 'n_points', 'level', 'reason',
            'ego_slope', 'evidence', 'held', 'too_small', 'rail_z_m')


def obj_record(o):
    return {k: (round(o[k], 3) if isinstance(o[k], float) else o[k]) for k in OBJ_KEYS if k in o}


def _fmt(v, nd):
    return '' if v is None else round(v, nd)


def _stream(reader, conn, segments, seglen, errors):
    """(номер кадра, t, сообщение): вся запись или segments отрезков по seglen кадров,
    равномерно по времени (детектор перезапускается в начале каждого отрезка)."""
    if not segments:
        for i, (c, t, raw) in enumerate(safe_messages(reader.messages(connections=[conn]), errors)):
            yield i, t, c, raw, i == 0
        return
    t0, dur = reader.start_time, reader.duration
    for k in range(segments):
        start = t0 + int(dur * (k + 0.5) / segments)
        for j, (c, t, raw) in enumerate(safe_messages(reader.messages(connections=[conn], start=start), errors)):
            if j == seglen:
                break
            yield k * seglen + j, t, c, raw, j == 0


def run_bag(job):
    bag, corridor, minpts, method, det_kwargs, segments, seglen, parser, all_objects = job
    warnings.simplefilter('ignore')
    parse_pointcloud2 = cloud_parser(parser)
    make = lambda: ObstacleDetector(zone=CORRIDORS[corridor], method=method, **{**MINPTS[minpts], **det_kwargs})
    rows, objs, errors = [], [], []
    with open_cloud_bag(bag) as (reader, conn):
        t0 = reader.start_time
        for i, t, c, raw, fresh in _stream(reader, conn, segments, seglen, errors):
            if fresh:
                det, repeat, prev = make(), FrameRepeat(), None
            m = reader.deserialize(raw, c.msgtype)
            t_s = round((t - t0) / 1e9, 2)
            if repeat.check(m.data) and prev is not None:
                # побитовый повтор кадра: детектор не вызывается, результат -- прошлый
                rows.append(dict(prev[0], frame=i, t_s=t_s, repeated=1))
                objs.append(dict(prev[1], frame=i, repeated=1))
                continue
            res = det.detect(*parse_pointcloud2(m.data, m.point_step, m.fields), refit_path=True, stamp=t / 1e9)
            stop = [o for o in res['objects'] if o['level'] == 'stop']
            near = min(stop, key=lambda o: o['distance_m']) if stop else None
            row = {'bag': bag, 'frame': i, 't_s': t_s, 'alarm': int(res['obstacle']), 'status': res['status'],
                   'caution': int(res['status'] == 'caution'),
                   'distance_m': _fmt(res['distance_m'], 1), 'caution_distance_m': _fmt(res['caution_distance_m'], 1),
                   'lateral_m': round(near['lateral_m'], 2) if near else '',
                   'height_m': round(near['height_m'], 2) if near else '',
                   'n_points': near['n_points'] if near else '',
                   'n_confirmed': len(stop),
                   'path_range_m': round(res['path_range_m'], 0) if res['path_range_m'] else '',
                   'sight_m': round(res['sight_m'], 1), 'clear_to_m': round(res['clear_to_m'], 1),
                   'travel_m': round(res['travel_m'], 2), 'displacement_m': _fmt(res['displacement_m'], 3),
                   'repeated': 0}
            keep = [o for o in res['objects'] if o['level'] or (all_objects and not o.get('edge_line'))]
            ol = {'bag': bag, 'frame': i, 'repeated': 0, 'objects': [obj_record(o) for o in keep]}
            rows.append(row)
            objs.append(ol)
            prev = (row, ol)
            if i % 2000 == 0 and i:
                print(f'  {bag}: {i} кадров', flush=True)
    if errors:
        print(f'  {bag}: чтение оборвалось после {len(rows)} кадров: {errors[0]}', flush=True)
    return rows, objs, errors


def episodes(alarm):
    idx = np.where(alarm)[0]
    if not len(idx):
        return []
    return np.split(idx, np.where(np.diff(idx) > GAP + 1)[0] + 1)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--tag', default='v0', help='метка версии детектора для имени файла')
    ap.add_argument('--bags', nargs='+', default=BAGS)
    ap.add_argument('--corridor', default='rect', choices=list(CORRIDORS))
    ap.add_argument('--minpts', default='on', choices=list(MINPTS), help='порог числа точек по дальности')
    ap.add_argument('--method', default='zone', choices=['zone', 'background'], help='zone -- точки в зоне; background -- остаток до фона тоннеля')
    ap.add_argument('--workers', type=int, default=2, help='параллельных процессов (каждый -- ядро и до ~0.5ГБ)')
    ap.add_argument('--det', default='{}', help='доп. kwargs ObstacleDetector в JSON')
    ap.add_argument('--segments', type=int, default=0, help='не вся запись, а столько отрезков (для длинной new_data)')
    ap.add_argument('--seglen', type=int, default=400, help='кадров в отрезке')
    ap.add_argument('--parser', default='python', choices=['python', 'cpp'],
                    help='разбор облака: python -- parse_pointcloud2, cpp -- как C++-узел приёма (tunnel_od_preproc)')
    ap.add_argument('--all-objects', action='store_true', help='в objects.jsonl -- все объекты, не только stop/caution (для эталона cloud_with_fake_obj)')
    args = ap.parse_args()
    OUT.mkdir(exist_ok=True)
    for suffix in ('.csv', '_objects.jsonl', '_read.json'):      # не оставлять выход прошлого прогона с тем же тегом
        (OUT / f'alarms_{args.tag}{suffix}').unlink(missing_ok=True)
    with Pool(min(len(args.bags), args.workers)) as pool:
        parts = pool.map(run_bag, [(b, args.corridor, args.minpts, args.method, json.loads(args.det),
                                    args.segments, args.seglen, args.parser, args.all_objects) for b in args.bags])
    rows = [r for p in parts for r in p[0]]
    obj_lines = [o for p in parts for o in p[1]]
    (OUT / f'alarms_{args.tag}_read.json').write_text(json.dumps(
        {'segments': args.segments,
         'bags': {b: {'frames': len(p[0]), 'error': p[2][0] if p[2] else None} for b, p in zip(args.bags, parts)}},
        ensure_ascii=False, indent=1), encoding='utf-8')
    if not rows:                       # запись не читается с первого кадра
        print('ни одного кадра:', '; '.join(e for p in parts for e in p[2]))
        return
    path = OUT / f'alarms_{args.tag}.csv'
    with open(path, 'w', newline='', encoding='utf-8') as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)
    with open(OUT / f'alarms_{args.tag}_objects.jsonl', 'w', encoding='utf-8') as f:
        for o in obj_lines:
            f.write(json.dumps(o, ensure_ascii=False) + '\n')

    head = (f'\n{"запись":<37}{"кадров":>7}{"мин":>6}{"СТОП":>8}{"ВНИМ.":>8}{"unknown":>9}{"повт.":>7}'
            f'{"эпизодов":>10}{"эпизодов/ч":>11}{"дист. медиана":>14}')
    print(head)

    def line(name, part):
        st = np.array([r['status'] for r in part])
        a = st == 'stop'
        dur = len(part) / 10.0 / 3600   # 10 Гц; по t_s нельзя: отрезки
        d = [r['distance_m'] for r in part if r['alarm']]
        print(f'{name:<37}{len(part):>7}{60 * dur:>6.1f}{100 * a.mean():>7.1f}%{100 * (st == "caution").mean():>7.1f}%'
              f'{100 * (st == "unknown").mean():>8.1f}%{sum(r["repeated"] for r in part):>7}'
              f'{len(episodes(a)):>10}{len(episodes(a)) / max(dur, 1e-9):>11.0f}'
              f'{(f"{np.median(d):.0f}м" if d else "-"):>14}')

    by_bag = {b: [r for r in rows if r['bag'] == b] for b in args.bags}
    for bag in args.bags:
        if by_bag[bag]:
            line(bag, by_bag[bag])
    empty = [r for b in args.bags if b in EMPTY_BAGS for r in by_bag[b]]
    if empty:
        line('ПУСТЫЕ', empty)
    line('ВСЕГО', rows)
    for bag, p in zip(args.bags, parts):
        if p[2]:
            print(f'{bag}: ошибка чтения -- {p[2][0]}')
    print(f'\nМетод: {args.method}, коридор: {args.corridor}, порог точек: {args.minpts}. Записано: {path}')


if __name__ == '__main__':
    main()
