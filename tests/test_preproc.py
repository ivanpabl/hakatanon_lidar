"""C++-приём облака (ros2_ws/src/tunnel_od_preproc) против parse_pointcloud2 и контракта v1.

Библиотека собирается на месте (tunnel_od_preproc/build_host.py); нет компилятора -- тесты
пропускаются. Тесты на записях пропускаются, если записей нет (data/ не в git).

    pytest tests/test_preproc.py
"""
import math
import sys
import warnings
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / 'ros2_ws' / 'src' / 'tunnel_od_preproc'))
sys.path.insert(0, str(ROOT / 'tools'))

from tunnel_od import ObstacleDetector, parse_pointcloud2          # noqa: E402
from tunnel_od.pointcloud import COLUMN_HEIGHT                    # noqa: E402
from tunnel_od_preproc.build_host import ensure                   # noqa: E402

warnings.filterwarnings('ignore', category=RuntimeWarning)

if ensure() is None:
    pytest.skip('не собрать libtunnel_od_canonical (нет компилятора C++)', allow_module_level=True)
from tunnel_od_preproc import native                              # noqa: E402

if not native.available():
    pytest.skip(f'libtunnel_od_canonical: {native.load_error}', allow_module_level=True)

from to_contract_v1 import legacy_to_contract                     # noqa: E402


class _F:
    def __init__(self, name, offset, datatype=7):
        self.name, self.offset, self.datatype, self.count = name, offset, datatype, 1


LEGACY_DT = np.dtype([('x', '<f4'), ('y', '<f4'), ('z', '<f4'), ('intensity', '<f4'), ('ring', '<u2'),
                      ('timestamp', '<f8')])
LEGACY_FIELDS = [_F(n, LEGACY_DT.fields[n][1], {'<f4': 7, '<u2': 4, '<f8': 8}[LEGACY_DT.fields[n][0].str])
                 for n in LEGACY_DT.names]


def legacy_cloud(x, y, z, stamp=946687297.9):
    a = np.zeros(len(x), LEGACY_DT)
    a['x'], a['y'], a['z'] = x, y, z
    a['ring'] = np.arange(len(x)) % COLUMN_HEIGHT
    a['timestamp'] = stamp + 1e-6 * np.arange(len(x))
    assert a.dtype.itemsize == 26
    return a.tobytes()


def bits_equal(a, b):
    return len(a) == len(b) and all(np.array_equal(p.view(np.uint32), q.view(np.uint32)) for p, q in zip(a, b))


def native_legacy(data, **kw):
    return native.parse_fast(data, 26, LEGACY_FIELDS, 1, len(data) // 26, stamp_ns=946687297_900000000, **kw)


def native_contract(data, h, w, ps, flds, stamp_ns=946687297_900000000):
    return native.parse_fast(data, ps, [_F(*f) for f in flds], h, w, ps * w, stamp_ns=stamp_ns)


def synthetic(n, seed):
    """Облако с пустыми лучами, NaN/inf, -0 и дублями около порога 1 см."""
    rng = np.random.default_rng(seed)
    x = rng.normal(0, 3, n).astype(np.float32)
    y = -rng.uniform(-5, 200, n).astype(np.float32)
    z = rng.normal(-1, 1, n).astype(np.float32)
    k = n - n % (2 * COLUMN_HEIGHT)
    if k:
        cols = lambda a: a[:k].reshape(-1, 2, COLUMN_HEIGHT)
        for a in (x, y, z):
            same = rng.random(cols(a)[:, 1].shape) < 0.7
            eps = rng.choice(np.float32([0.0, 0.0099, 0.01, 0.0101, -0.0099, 0.5]), size=same.shape)
            cols(a)[:, 1] = np.where(same, cols(a)[:, 0] + eps, cols(a)[:, 1])
    empty = rng.random(n) < 0.4
    x[empty] = y[empty] = z[empty] = 0.0
    x[rng.random(n) < 0.01] = np.nan
    y[rng.random(n) < 0.005] = np.inf
    neg = rng.random(n) < 0.01
    x[neg], y[neg], z[neg] = -0.0, 0.0, -0.0
    return x, y, z


@pytest.mark.parametrize('n,seed', [(8 * COLUMN_HEIGHT, 0), (40 * COLUMN_HEIGHT, 1), (2 * COLUMN_HEIGHT + 64, 2),
                                    (3 * COLUMN_HEIGHT, 3), (100, 4)])
def test_parse_bitwise_equal_to_python(n, seed):
    x, y, z = synthetic(n, seed)
    data = legacy_cloud(x, y, z)
    py = parse_pointcloud2(data, 26, LEGACY_FIELDS)
    fmt = 'legacy_hesai' if n % 256 == 0 else 'auto'
    assert bits_equal(py, native_legacy(data, fmt=fmt))


def test_parse_like_test_core():
    """Случай из tests/test_core.py: второй столбец пары = первый, 10 пустых лучей в обоих."""
    n_cols = 4
    x = np.arange(2 * n_cols * COLUMN_HEIGHT, dtype=np.float32) * 0.01 + 1.0
    y = np.full_like(x, -5.0)
    z = np.zeros_like(x)
    x.reshape(-1, 2, COLUMN_HEIGHT)[:, 1] = x.reshape(-1, 2, COLUMN_HEIGHT)[:, 0]
    for a in (x, y, z):
        a[:10] = a[COLUMN_HEIGHT:COLUMN_HEIGHT + 10] = 0.0
    data = legacy_cloud(x, y, z)
    out = native_legacy(data, with_stats=True)
    assert bits_equal(parse_pointcloud2(data, 26, LEGACY_FIELDS), out[:3])
    assert len(out[0]) == n_cols * COLUMN_HEIGHT - 10
    assert out[3]['format'] == 'legacy_hesai' and out[3]['dedupe_applied']


@pytest.mark.parametrize('seed', [0, 1])
def test_contract_v1_gives_same_canonical_cloud(seed):
    x, y, z = synthetic(16 * COLUMN_HEIGHT, seed)
    data = legacy_cloud(x, y, z)
    stamp = 946687297_900000000
    conv = legacy_to_contract(data, 26, LEGACY_FIELDS, stamp)
    assert conv[1] == COLUMN_HEIGHT and conv[3] == 24
    assert bits_equal(parse_pointcloud2(data, 26, LEGACY_FIELDS), native_contract(*conv, stamp_ns=stamp))


def test_crop_only_removes_points_outside_range():
    x, y, z = synthetic(64 * COLUMN_HEIGHT, 5)
    y[: 20 * COLUMN_HEIGHT] = -np.abs(y[: 20 * COLUMN_HEIGHT]) % 12 - 2.5     # пол в окне 2..15 м
    x[: 20 * COLUMN_HEIGHT] = np.clip(x[: 20 * COLUMN_HEIGHT], -2, 2)
    data = legacy_cloud(x, y, z)
    full = native_legacy(data)
    cut = native_legacy(data, crop=(2.0, 250.0), with_stats=True)
    keep = (-full[1] >= 2.0) & (-full[1] <= 250.0)
    assert bits_equal(tuple(a[keep] for a in full), cut[:3])
    assert cut[3]['crop_applied'] and cut[3]['n_cropped'] == int((~keep).sum())


def test_required_range_covers_core_and_config():
    """Обрезка по умолчанию (config/detector.yaml) не задевает то, что использует ядро."""
    sys.path.insert(0, str(ROOT / 'ros2_ws' / 'src' / 'tunnel_od_detector'))
    from tunnel_od_detector.util import required_fwd_range
    lo, hi = required_fwd_range({})
    assert (lo, hi) == (2.0, 250.0)
    assert required_fwd_range({'near_cutoff': 1.0, 'max_range': 300.0}) == (1.0, 300.0)
    import inspect
    from tunnel_od.geometry import bed
    src = inspect.getsource(bed.estimate_floor_z)
    # защита обрезки в C++ (kFloorGuard*) повторяет запасную оценку пола
    assert 'm.sum() < 50' in src
    sig = inspect.signature(bed.estimate_floor_z).parameters
    assert (sig['near'].default, sig['far'].default, sig['half_width'].default) == (2.0, 15.0, 3.0)


# ------------------------------------------------------------------ реальные записи

def _bag_or_skip(name):
    from bags import bag_path
    if not bag_path(name).is_dir():
        pytest.skip(f'нет записи {name}')


def _frames(name, count):
    from bags import open_cloud_bag
    with open_cloud_bag(name) as (reader, conn):
        for i, (c, t, raw) in enumerate(reader.messages(connections=[conn])):
            if i >= count:
                break
            yield reader.deserialize(raw, c.msgtype), t


BAGS = ['doubleT_obstacle', 'doubleT_platform', 'roundT_doubleT', 'roundT_pressureGate_roundT',
        'roundT_squareT_pressureGate_squareT', 'squareT_platform_squareT_switch', 'new_data']


@pytest.mark.parametrize('bag', BAGS)
def test_real_frames_legacy_and_contract(bag):
    _bag_or_skip(bag)
    for m, _ in _frames(bag, 2):
        py = parse_pointcloud2(m.data, m.point_step, m.fields)
        assert bits_equal(py, native.parse_msg(m))
        stamp = int(m.header.stamp.sec) * 10 ** 9 + int(m.header.stamp.nanosec)
        conv = legacy_to_contract(m.data, m.point_step, m.fields, stamp)
        assert bits_equal(py, native_contract(*conv, stamp_ns=stamp))


def test_axes_sign_on_real_frame():
    """Знак поворота legacy -> REP-103 (x_rep = -y, y_rep = x):
    1) рельсы, найденные ядром, в REP-103 лежат впереди (x > 0) по обе стороны оси y = 0;
    2) лидар Hesai вращается по часовой стрелке (вид сверху): в REP-103 (z вверх, угол
       против часовой) азимут столбца со временем убывает -- как у legacy atan2(x, -y).
       При x ядра = -y_rep (зеркально) он бы возрастал."""
    _bag_or_skip('doubleT_platform')
    m, t = next(_frames('doubleT_platform', 1))
    x, y, z = parse_pointcloud2(m.data, m.point_step, m.fields)
    det = ObstacleDetector()
    det.detect(x, y, z, refit_path=True, stamp=0.0)
    tp = det.track_path()
    fwd = tp.fit_fwd[tp.fit_fwd < 40]
    lat = tp.center_at(fwd)
    # точка оси в осях ядра (lat, -fwd) -> REP-103 поворотом +90 град вокруг z (TF, который публикует узел)
    q = (0.0, 0.0, math.sqrt(0.5), math.sqrt(0.5))
    R = np.array([[1 - 2 * (q[1] ** 2 + q[2] ** 2), 2 * (q[0] * q[1] - q[2] * q[3])],
                  [2 * (q[0] * q[1] + q[2] * q[3]), 1 - 2 * (q[0] ** 2 + q[2] ** 2)]])
    rep = R @ np.vstack([lat, -fwd])
    assert np.all(rep[0] > 1.0) and np.allclose(rep[0], fwd, atol=1e-6) and np.all(np.abs(rep[1]) < 1.0)
    # то же -- формулой адаптера
    assert np.allclose(rep[1], lat, atol=1e-6)

    a = np.frombuffer(bytes(m.data), np.uint8).reshape(-1, m.point_step)
    X, Y = (a[:, o:o + 4].copy().view(np.float32).ravel() for o in (0, 4))
    x_rep, y_rep = -Y, X
    az = np.degrees(np.arctan2(y_rep, x_rep)).reshape(-1, COLUMN_HEIGHT)
    ok = (X != 0).reshape(-1, COLUMN_HEIGHT)
    col = np.array([np.median(r[k]) if k.any() else np.nan for r, k in zip(az, ok)])
    good = np.isfinite(col)
    slope = np.polyfit(np.arange(len(col))[good], col[good], 1)[0]
    assert slope < 0          # -0.05 град на столбец (0.1 на пару)


def _stream_summary(bag, max_frames=None):
    from input_report import monitor_bag
    return monitor_bag(bag, max_frames=max_frames)


def test_diagnostics_legacy_recording():
    _bag_or_skip('doubleT_platform')
    s = _stream_summary('doubleT_platform', max_frames=40)
    ch = s['checks']
    assert ch['format']['message'].startswith('legacy_hesai/legacy')
    assert ch['M1']['level'] == 'warn' and ch['M1']['unordered_frames'] == 40 and ch['M1']['zero_share'] > 0.3
    assert ch['M2']['mode'] == 'dual_by_distance' and ch['M2']['pair_coincident_share'] > 0.9
    assert ch['M3']['level'] == 'warn'
    assert ch['M4']['stamp_year'] == 2000 and ch['M4']['level'] == 'warn'
    assert ch['M5']['level'] == 'ok'


@pytest.mark.parametrize('bag', ['squareT_platform_squareT_switch', 'roundT_squareT_pressureGate_squareT'])
def test_diagnostics_finds_gaps(bag):
    _bag_or_skip(bag)
    s = _stream_summary(bag)
    assert s['checks']['M4']['gaps'] > 0 and 'M4' in s['violations']


def test_diagnostics_contract_recording_is_clean():
    from bags import DATA
    if not (DATA / 'contract_v1' / 'doubleT_platform').is_dir():
        pytest.skip('нет записи контракта v1: tools/to_contract_v1.py --bag doubleT_platform '
                    '--out data/contract_v1/doubleT_platform --speed')
    s = _stream_summary('contract_v1/doubleT_platform')
    assert s['violations'] == [], s
