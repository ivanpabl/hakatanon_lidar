"""Тесты ядра без записей: синтетический тоннель (tunnel_od.sim.lidar_sim) и простые случаи.

    pip install -e ./core pytest && pytest tests
"""
import warnings

import numpy as np
import pytest

from tunnel_od import ObstacleDetector, parse_pointcloud2
from tunnel_od.detection.tracking import EvidenceTracker
from tunnel_od.geometry.path import extend_path_by_walls, splice_far_axis
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


def test_evidence_tracker_holds_alarm_track_over_short_gap():
    """Трек с тревогой без объекта в кадре держится hold кадров с прогнозом дистанции."""
    tr = EvidenceTracker(hold=3, hold_min=2)
    for k in range(4):
        ob = {'distance_m': 60.0 - 1.0 * k, 'lateral_m': 0.1, 'n_points': 20}
        tr.update([ob], displacement=1.0, expected=lambda d: 5.0)
        if ob['confirmed']:
            tr.set_alarm({ob['track_id']})
    assert ob['confirmed']
    for k in range(3):
        tr.update([], displacement=1.0, expected=lambda d: 5.0)
        held = tr.held()
        assert len(held) == 1 and abs(held[0]['distance_m'] - (57.0 - 1.0 * (k + 1))) < 1e-6
    tr.update([], displacement=1.0, expected=lambda d: 5.0)
    assert tr.held() == []


def test_evidence_tracker_holds_only_tracks_that_raised_alarm():
    """Подтверждённый трек без тревоги (например, за концом оси) не держится."""
    tr = EvidenceTracker(hold=3, hold_min=1)
    for k in range(4):
        ob = {'distance_m': 60.0, 'lateral_m': 0.0, 'n_points': 20}
        tr.update([ob], displacement=0.0, expected=lambda d: 5.0)
    assert ob['confirmed']
    tr.update([], displacement=0.0, expected=lambda d: 5.0)
    assert tr.held() == []


def test_evidence_tracker_does_not_hold_short_alarm():
    """Тревога короче hold_min кадров (ложная вспышка) не удлиняется."""
    tr = EvidenceTracker(hold=3, hold_min=5)
    for k in range(4):
        ob = {'distance_m': 60.0, 'lateral_m': 0.0, 'n_points': 20}
        tr.update([ob], displacement=0.0, expected=lambda d: 5.0)
        if ob['confirmed']:
            tr.set_alarm({ob['track_id']})
    tr.update([], displacement=0.0, expected=lambda d: 5.0)
    assert tr.held() == []


def test_detector_alarm_survives_single_missed_frame():
    det = ObstacleDetector()
    _run(det, [{}] * 3)
    res = _run(det, [{'obstacle_forward': 30.0, 'obstacle_radius': 0.35}] * 8)
    assert res['obstacle']
    res = _run(det, [{}])
    assert res['obstacle'] and abs(res['distance_m'] - (30.0 - 0.35)) < 0.5
    assert any(o.get('held') for o in res['objects'])
    res = _run(det, [{}] * 4)
    assert not res['obstacle']


def test_detector_alarm_hold_off():
    det = ObstacleDetector(alarm_hold=0)
    _run(det, [{}] * 3)
    assert _run(det, [{'obstacle_forward': 30.0, 'obstacle_radius': 0.35}] * 4)['obstacle']
    assert not _run(det, [{}])['obstacle']


def test_splice_far_axis_keeps_longer_previous_axis():
    old_f = np.arange(2.0, 200.0, 1.0)
    old_c = 0.001 * old_f
    new_f = np.arange(2.0, 60.0, 1.0)
    new_c = 0.001 * (new_f + 3.0) + 0.05           # поезд проехал 3 м; небольшой сдвиг вбок
    f, c, ok = splice_far_axis(new_f, new_c, old_f, old_c, shift=3.0)
    assert ok and f[-1] == pytest.approx(196.0)
    assert np.all(np.diff(f) > 0)
    assert np.interp(59.0, f, c) == pytest.approx(new_c[-1])
    assert abs(np.interp(61.0, f, c) - np.interp(59.0, f, c)) < 0.01   # без ступеньки на стыке

    far_off = 0.001 * new_f + 0.8                    # другая ось (стрелка) -- не склеиваем
    f, c, ok = splice_far_axis(new_f, far_off, old_f, old_c, shift=0.0)
    assert not ok and f[-1] == new_f[-1]

    f, c, ok = splice_far_axis(old_f, old_c, new_f, new_c, shift=0.0)   # новая длиннее
    assert not ok and f[-1] == old_f[-1]


def test_detector_holds_far_axis_when_refit_is_short():
    det = ObstacleDetector(path_hold=5)
    _run(det, [{}] * 3)
    full = det.path_range
    assert full > 100
    det._fit_fwd, det._fitted_cl = det._fit_fwd[det._fit_fwd < 50], det._fitted_cl[det._fit_fwd < 50]
    det._path_range = float(det._fit_fwd[-1])
    det._path_frame = det._frame + 1                # как будто пришёл новый короткий путь
    x, y, z, *_ = simulate_frame()
    res = det.check_frame(x, y, z)
    assert res['path_range_m'] > full - 5


def test_held_alarm_survives_short_axis():
    det = ObstacleDetector()
    _run(det, [{}] * 3)
    assert _run(det, [{'obstacle_forward': 60.0, 'obstacle_radius': 0.35}] * 8)['obstacle']
    x, y, z, *_ = simulate_frame()
    det._fit_fwd, det._fitted_cl = det._fit_fwd[det._fit_fwd < 50], det._fitted_cl[det._fit_fwd < 50]
    det._path_range = float(det._fit_fwd[-1])       # новый путь короче объекта, объект пропал
    res = det.check_frame(x, y, z)
    assert res['obstacle'] and res['path_range_m'] < 55
