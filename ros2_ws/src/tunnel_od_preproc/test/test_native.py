"""Библиотека tunnel_od_preproc из Python (ctypes) против parse_pointcloud2 -- в образе.
На реальных записях то же проверяют tests/test_preproc.py и python -m evaluation preproc."""
import numpy as np
import pytest

from tunnel_od_preproc import native

pytest.importorskip('tunnel_od')
from tunnel_od import parse_pointcloud2      # noqa: E402

pytestmark = pytest.mark.skipif(not native.available(), reason=f'нет библиотеки: {native.load_error}')
H = 128


class _F:
    def __init__(self, name, offset, datatype=7):
        self.name, self.offset, self.datatype, self.count = name, offset, datatype, 1


DT = np.dtype([('x', '<f4'), ('y', '<f4'), ('z', '<f4'), ('intensity', '<f4'), ('ring', '<u2'), ('timestamp', '<f8')])
FIELDS = [_F('x', 0), _F('y', 4), _F('z', 8), _F('intensity', 12), _F('ring', 16, 4), _F('timestamp', 18, 8)]


def cloud(n, seed):
    rng = np.random.default_rng(seed)
    a = np.zeros(n, DT)
    a['x'] = rng.normal(0, 3, n)
    a['y'] = -rng.uniform(-5, 200, n)
    a['z'] = rng.normal(-1, 1, n)
    k = n - n % (2 * H)
    for f in 'xyz':
        v = a[f][:k].reshape(-1, 2, H)
        v[:, 1] = np.where(rng.random(v[:, 1].shape) < 0.7, v[:, 0] + rng.choice([0.0, 0.0099, 0.0101], v[:, 1].shape),
                           v[:, 1])
        a[f][:k] = v.reshape(-1)
    empty = rng.random(n) < 0.4
    for f in 'xyz':
        a[f][empty] = 0.0
    a['x'][rng.random(n) < 0.01] = np.nan
    a['ring'] = np.arange(n) % H
    a['timestamp'] = 946687297.9 + 1e-6 * np.arange(n)
    return a


def same(a, b):
    return len(a[0]) == len(b[0]) and all(np.array_equal(p.view(np.uint32), q.view(np.uint32)) for p, q in zip(a, b))


@pytest.mark.parametrize('n', [8 * H, 30 * H, 2 * H + 64])
def test_legacy_bitwise_equal_to_python(n):
    data = cloud(n, n).tobytes()
    fmt = 'legacy_hesai' if n % 256 == 0 else 'auto'
    assert same(parse_pointcloud2(data, 26, FIELDS), native.parse_fast(data, 26, FIELDS, fmt=fmt))


def test_contract_v1_same_as_legacy():
    a = cloud(16 * H, 7)
    py = parse_pointcloud2(a.tobytes(), 26, FIELDS)
    x, y, z = (a[f].copy() for f in 'xyz')
    valid = np.isfinite(x) & np.isfinite(y) & np.isfinite(z) & ((x != 0) | (y != 0) | (z != 0))
    c = lambda v: v.reshape(-1, 2, H)
    dup = (np.abs(c(x)[:, 1] - c(x)[:, 0]) < 0.01) & (np.abs(c(y)[:, 1] - c(y)[:, 0]) < 0.01) \
        & (np.abs(c(z)[:, 1] - c(z)[:, 0]) < 0.01)
    valid.reshape(-1, 2, H)[:, 1] &= ~dup
    out = np.zeros(len(a), np.dtype({'names': ['x', 'y', 'z', 'return_id', 'ring'],
                                     'formats': ['<f4', '<f4', '<f4', 'u1', '<u2'],
                                     'offsets': [0, 4, 8, 13, 14], 'itemsize': 24}))
    out['x'], out['y'], out['z'] = np.where(valid, -y, np.nan), np.where(valid, x, np.nan), np.where(valid, z, np.nan)
    out['return_id'] = (np.arange(len(a)) // H) % 2
    out['ring'] = np.arange(len(a)) % H
    W = len(a) // H
    data = out.reshape(W, H).T.copy().tobytes()
    fields = [_F('ring', 14, 4), _F('return_id', 13, 2), _F('z', 8), _F('y', 4), _F('x', 0)]
    got = native.parse_fast(data, 24, fields, H, W, with_stats=True)
    assert got[3]['format'] == 'contract_v1' and got[3]['axes'] == 'rep103'
    assert same(py, got[:3])


def test_monitor_reports_legacy_violations():
    mon = native.Monitor()
    data = cloud(8 * H, 1).tobytes()
    for i in range(25):
        native.parse_fast(data, 26, FIELDS, stamp_ns=946687297_900000000 + i * 100_000_000, frame_id='hesai_lidar',
                          monitor=mon)
    s = mon.summary()
    assert {'M1', 'M2', 'M3', 'M4'} <= set(s['violations'])
    assert s['checks']['M4']['stamp_year'] == 2000
    assert any('часы датчика' in w for w in mon.new_warnings())
