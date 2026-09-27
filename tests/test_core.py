"""Тесты ядра без записей: синтетический тоннель (tunnel_od.sim.lidar_sim) и простые случаи.

    pip install -e ./core pytest && pytest tests
"""
import warnings

import numpy as np
import pytest

from tunnel_od import ObstacleDetector, parse_pointcloud2
from tunnel_od.detection.tracking import EvidenceTracker
from tunnel_od.geometry.path import extend_path_by_walls
from tunnel_od.pointcloud import COLUMN_HEIGHT
from tunnel_od.sim.lidar_sim import simulate_frame

warnings.filterwarnings('ignore', category=RuntimeWarning)

VARIANTS = [{}, {'method': 'background', 'tracker': 'evidence'}]


class _Field:
    def __init__(self, name, offset):
        self.name, self.offset = name, offset


def _cloud_bytes(x, y, z):
    """PointCloud2 как в записях: x, y, z, intensity (float32), ring (uint16), timestamp (float64)."""
    dt = np.dtype([('x', '<f4'), ('y', '<f4'), ('z', '<f4'), ('intensity', '<f4'),
                   ('ring', '<u2'), ('timestamp', '<f8')])
    a = np.zeros(len(x), dt)
    a['x'], a['y'], a['z'] = x, y, z
    return a.tobytes(), dt.itemsize, [_Field(n, dt.fields[n][1]) for n in dt.names]


def test_parse_drops_zero_points_and_dual_return_duplicates():
    n_cols = 4
    x = np.arange(2 * n_cols * COLUMN_HEIGHT, dtype=np.float32) * 0.01 + 1.0
    y = np.full_like(x, -5.0)
    z = np.zeros_like(x)
    x.reshape(-1, 2, COLUMN_HEIGHT)[:, 1] = x.reshape(-1, 2, COLUMN_HEIGHT)[:, 0]   # второе отражение = первое
    for a in (x, y, z):                                                              # нет отражения (оба)
        a[:10] = a[COLUMN_HEIGHT:COLUMN_HEIGHT + 10] = 0.0
    data, step, fields = _cloud_bytes(x, y, z)
    assert step == 26
    px, py, pz = parse_pointcloud2(data, step, fields)
    assert len(px) == n_cols * COLUMN_HEIGHT - 10
    assert np.all((px != 0) | (py != 0) | (pz != 0))


def _run(det, frames):
    res = None
    for kw in frames:
        x, y, z, *_ = simulate_frame(**kw)
        res = det.detect(x, y, z, refit_path=True, stamp=None)
    return res


@pytest.mark.parametrize('kwargs', VARIANTS)
def test_empty_tunnel_no_alarm_and_path_found(kwargs):
    det = ObstacleDetector(**kwargs)
    for _ in range(6):
        res = _run(det, [{}])
        assert not res['obstacle']
    assert res['path_available']
    assert abs(res['gauge_m'] - 1.52) < 0.05
    assert res['path_range_m'] > 100


@pytest.mark.parametrize('kwargs', VARIANTS)
def test_obstacle_on_axis_detected_with_distance(kwargs):
    det = ObstacleDetector(**kwargs)
    _run(det, [{}] * 3)
    res = _run(det, [{'obstacle_forward': 30.0, 'obstacle_radius': 0.35}] * 4)
    assert res['obstacle']
    assert abs(res['distance_m'] - (30.0 - 0.35)) < 0.3


@pytest.mark.parametrize('kwargs', VARIANTS)
def test_obstacle_outside_corridor_ignored(kwargs):
    det = ObstacleDetector(**kwargs)
    _run(det, [{}] * 3)
    res = _run(det, [{'obstacle_forward': 30.0, 'obstacle_lateral': 1.9, 'obstacle_radius': 0.3}] * 5)
    assert not res['obstacle']


def test_evidence_tracker_confirms_weak_static_object_and_rejects_flicker():
    expected = lambda d: 3.0
    tr = EvidenceTracker()
    d, confirmed_at = 150.0, None
    for k in range(12):
        ob = {'distance_m': d, 'lateral_m': 0.1, 'n_points': 2}
        tr.update([ob], displacement=1.5, expected=expected)
        if ob['confirmed'] and confirmed_at is None:
            confirmed_at = k
        d -= 1.5                      # поезд приближается к неподвижному объекту
    assert confirmed_at is not None and confirmed_at <= 8

    tr = EvidenceTracker()
    rng = np.random.default_rng(0)
    for k in range(30):              # одиночные точки в случайных местах не подтверждаются
        ob = {'distance_m': float(rng.uniform(20, 200)), 'lateral_m': float(rng.uniform(-1, 1)), 'n_points': 1}
        tr.update([ob], displacement=1.5, expected=expected)
        assert not ob['confirmed']


def test_evidence_tracker_strong_object_confirms_like_three_of_five():
    tr = EvidenceTracker()
    for k in range(3):
        ob = {'distance_m': 40.0 - 1.5 * k, 'lateral_m': 0.0, 'n_points': 50}
        tr.update([ob], displacement=1.5, expected=lambda d: 7.5)
    assert ob['confirmed']


def test_far_axis_follows_curve_by_walls():
    """Стены на +-2м от оси, ось -- дуга радиуса 400м. Рельсы (fit) известны до 40м;
    дальше точки стен редкие (как у датчика: столбцы через несколько метров)."""
    R = 400.0
    center = lambda s: s ** 2 / (2 * R)
    s = np.r_[np.arange(2, 60, 0.2), np.arange(60, 180, 6.0)]
    xs, ys, zs = [], [], []
    for side in (1, -1):
        for h in np.linspace(0.6, 2.4, 6):
            xs.append(center(s) + side * 2.0); ys.append(-s); zs.append(np.full_like(s, h))
    x, y, z = map(np.concatenate, (xs, ys, zs))
    fit_fwd = np.arange(2.0, 40.0, 1.0)
    fwd, cl = extend_path_by_walls(x, y, z, fit_fwd, center(fit_fwd), lambda f: np.zeros_like(f))
    assert fwd[-1] > 160
    far = fwd > 120
    assert np.max(np.abs(cl[far] - center(fwd[far]))) < 0.3
