# © 2026 команда «Голуби», github.com/ivanpabl/hakatanon_lidar. Все права защищены, условия — в файле LICENSE. GLB-K5-4660c47a8c6c
"""Устойчивость к плохому входу: пустой/крошечный кадр, время назад, повтор кадра, разрыв.

    pytest tests/test_robustness.py
"""
import math
import warnings

import numpy as np

from tunnel_od import ObstacleDetector
from tunnel_od.detection.background import AXIS_HEIGHT
from tunnel_od.geometry.ego_motion import MAX_SPEED, EgoMotion

warnings.filterwarnings('ignore', category=RuntimeWarning)


def test_empty_and_tiny_cloud_unknown_without_exception():
    det = ObstacleDetector()
    for n in (0, 1, 5):
        x = y = z = np.zeros(n, np.float32)
        for refit in (True, False):
            res = det.detect(x, y, z, refit_path=refit, stamp=1.0)
            assert res['status'] == 'unknown'
            assert not res['obstacle'] and res['objects'] == []


def _scene(seed=0):
    """Стенка тоннеля с неровностями вдоль пути: (s, lat, z_rel) в координатах сцены."""
    rng = np.random.default_rng(seed)
    s = np.repeat(np.arange(0.0, 200.0, 0.05), 24)
    th = np.tile(np.linspace(-np.pi, np.pi, 24, endpoint=False), len(s) // 24)
    bump = np.interp(s, np.arange(0.0, 201.0, 0.5), rng.uniform(-0.3, 0.3, 402))
    rho = 2.5 + bump
    return s, rho * np.cos(th), AXIS_HEIGHT + rho * np.sin(th)


def _feed(ego, stamps, pos):
    s, lat, z = _scene()
    out = []
    for t, p in zip(stamps, pos):
        fwd = s - p
        m = (fwd > 0) & (fwd < 120)
        out.append(ego.update(fwd[m], lat[m], z[m], stamp=t))
    return out


def _sane(v):
    return v is None or (math.isfinite(v) and 0.0 <= v <= MAX_SPEED * 1.5)


def test_ego_motion_backward_repeated_and_gap_timestamps():
    ego = EgoMotion()
    v = 10.0
    stamps = [0.1 * k for k in range(12)]
    pos = [v * t for t in stamps]
    # повтор кадра, время назад, разрыв в 5 с -- поезд при этом продолжает ехать
    stamps += [stamps[-1], stamps[-1] - 0.5, stamps[-1] + 5.0]
    pos += [pos[-1] + 1.0, pos[-1] + 2.0, pos[-1] + 3.0]
    stamps += [stamps[-1] + 0.1 * k for k in range(1, 10)]
    pos += [pos[-1] + v * 0.1 * k for k in range(1, 10)]
    out = _feed(ego, stamps, pos)
    for speed, disp in out:
        assert _sane(speed)
        assert disp is None or (math.isfinite(disp) and abs(disp) < 5.0)
    assert out[11][0] is not None and abs(out[11][0] - v) < 1.5
    assert out[-1][0] is not None and abs(out[-1][0] - v) < 1.5


def test_object_output_has_size_confidence_position():
    from tunnel_od.sim.lidar_sim import simulate_frame
    det = ObstacleDetector()
    for i in range(4):
        x, y, z, *_ = simulate_frame(obstacle_forward=40.0, obstacle_radius=0.4)
        res = det.detect(x, y, z, refit_path=True, stamp=0.1 * i)
    assert res['status'] == 'stop'
    o = min(res['objects'], key=lambda o: abs(o['distance_m'] - 39.6))
    assert set(o['size_m']) == {'length', 'width', 'height'} and all(v >= 0 for v in o['size_m'].values())
    assert o['confidence'] == 1.0
    p = o['position_m']
    assert abs(p['x']) < 0.3 and abs(p['y'] + 40.0) < 0.5 and math.isfinite(p['z'])
    for o in res['objects']:
        assert 0.0 <= o['confidence'] <= 1.0
