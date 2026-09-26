"""Офлайн-сверка C++-приёма облака с прежним разбором -- по кадрам записей, без ROS 2.

Для каждой записи и каждого кадра:
  python   -- parse_pointcloud2 (как было) -> ObstacleDetector.detect;
  cpp      -- тот же кадр через библиотеку tunnel_od_preproc (разбор + обрезка по
              дальности, как в узле), облако упаковано как /tunnel_od/cloud (x, y, z
              подряд) и взято представлениями без копии, как в узле -> detect;
  contract -- кадр переведён в контракт v1 (tools/to_contract_v1.py) -> tunnel_od_preproc
              -> detect.
Сравниваются: облако до обрезки (побитно, python vs cpp vs contract) и результат detect
целиком (JSON). Путь пересчитывается в каждом кадре (как refit_mode: sync), поэтому
результат детерминирован.

    python tools/check_preproc.py                          # 7 записей по 30 кадров
    python tools/check_preproc.py --bags doubleT_obstacle --frames 0      # все кадры
"""
import argparse
import json
import sys
import time
import warnings
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / 'ros2_ws' / 'src' / 'tunnel_od_preproc'))
sys.path.insert(0, str(Path(__file__).resolve().parent))
warnings.simplefilter('ignore')

from tunnel_od import ObstacleDetector, parse_pointcloud2          # noqa: E402

from bags import BAGS, open_cloud_bag                              # noqa: E402
from to_contract_v1 import legacy_to_contract                      # noqa: E402

CROP = (2.0, 250.0)


class _F:
    def __init__(self, name, offset, datatype):
        self.name, self.offset, self.datatype, self.count = name, offset, datatype, 1


def as_published(x, y, z):
    """Как облако приходит в узел детектора: x, y, z подряд в одном буфере, представления без копии."""
    buf = np.empty((len(x), 3), np.float32)
    buf[:, 0], buf[:, 1], buf[:, 2] = x, y, z
    a = np.frombuffer(buf.tobytes(), np.float32).reshape(-1, 3)
    return a[:, 0], a[:, 1], a[:, 2]


def same_bits(a, b):
    return len(a[0]) == len(b[0]) and all(np.array_equal(p.view(np.uint32), q.view(np.uint32)) for p, q in zip(a, b))


def dump(res):
    return json.dumps(res, sort_keys=True, default=lambda o: o.tolist() if hasattr(o, 'tolist') else str(o))


def check_bag(bag, frames, det_kwargs):
    from tunnel_od_preproc import native
    dets = {k: ObstacleDetector(**det_kwargs) for k in ('python', 'cpp', 'contract')}
    n = parse_ok = res_ok = 0
    t_py = t_cpp = t_con = 0.0
    kept = []
    first_bad = None
    with open_cloud_bag(bag) as (reader, conn):
        for i, (c, t, raw) in enumerate(reader.messages(connections=[conn])):
            if frames and i >= frames:
                break
            m = reader.deserialize(raw, c.msgtype)
            stamp_ns = int(m.header.stamp.sec) * 10 ** 9 + int(m.header.stamp.nanosec)
            stamp = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9

            t0 = time.perf_counter()
            py = parse_pointcloud2(m.data, m.point_step, m.fields)
            t1 = time.perf_counter()
            full = native.parse_msg(m)
            cut = native.parse_msg(m, crop=CROP)
            t2 = time.perf_counter()
            d, h, w, ps, fl = legacy_to_contract(m.data, m.point_step, m.fields, stamp_ns)
            flds = [_F(*f) for f in fl]
            t3 = time.perf_counter()
            con_full = native.parse_fast(d, ps, flds, h, w, ps * w, stamp_ns=stamp_ns)
            t4 = time.perf_counter()
            con_cut = native.parse_fast(d, ps, flds, h, w, ps * w, stamp_ns=stamp_ns, crop=CROP)
            t_py += t1 - t0
            t_cpp += (t2 - t1) / 2
            t_con += t4 - t3
            ok_parse = same_bits(py, full) and same_bits(py, con_full) and same_bits(cut, con_cut)
            parse_ok += ok_parse
            kept.append(len(cut[0]) / max(len(py[0]), 1))

            r_py = dump(dets['python'].detect(*py, refit_path=True, stamp=stamp))
            r_cpp = dump(dets['cpp'].detect(*as_published(*cut), refit_path=True, stamp=stamp))
            r_con = dump(dets['contract'].detect(*as_published(*con_cut), refit_path=True, stamp=stamp))
            ok_res = r_py == r_cpp == r_con
            res_ok += ok_res
            if (not ok_parse or not ok_res) and first_bad is None:
                first_bad = i
            n += 1
    return {'frames': n, 'parse_bitwise': parse_ok, 'result_identical': res_ok, 'first_bad_frame': first_bad,
            'kept_after_crop': float(np.mean(kept)) if kept else None,
            'parse_ms_python': 1e3 * t_py / max(n, 1), 'parse_ms_cpp': 1e3 * t_cpp / max(n, 1),
            'parse_ms_contract': 1e3 * t_con / max(n, 1)}


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--bags', nargs='+', default=BAGS)
    ap.add_argument('--frames', type=int, default=30, help='кадров на запись с начала (0 -- все)')
    ap.add_argument('--det', default='{}', help='kwargs ObstacleDetector в JSON')
    ap.add_argument('--out', default=str(ROOT / 'runs' / 'check_preproc.json'))
    args = ap.parse_args()
    from tunnel_od_preproc.build_host import ensure
    if ensure(quiet=False) is None:
        sys.exit('не собрать libtunnel_od_canonical')

    rows = {}
    print(f'{"запись":<37}{"кадров":>7}{"облако=":>9}{"detect=":>9}{"после обрезки":>15}'
          f'{"разбор py/cpp/contract, мс":>30}')
    for bag in args.bags:
        r = check_bag(bag, args.frames, json.loads(args.det))
        rows[bag] = r
        print(f'{bag:<37}{r["frames"]:>7}{r["parse_bitwise"]:>9}{r["result_identical"]:>9}'
              f'{100 * r["kept_after_crop"]:>14.0f}%{r["parse_ms_python"]:>12.1f}{r["parse_ms_cpp"]:>9.1f}'
              f'{r["parse_ms_contract"]:>9.1f}', flush=True)
    Path(args.out).parent.mkdir(parents=True, exist_ok=True)
    Path(args.out).write_text(json.dumps(rows, ensure_ascii=False, indent=2))
    bad = [b for b, r in rows.items() if r['parse_bitwise'] != r['frames'] or r['result_identical'] != r['frames']]
    print('ИТОГ:', 'всё совпадает' if not bad else f'РАСХОЖДЕНИЯ: {bad}')
    sys.exit(1 if bad else 0)


if __name__ == '__main__':
    main()
