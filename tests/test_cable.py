"""Оборванный кабель, свисающий со свода в габарит (sim.shapes.Cable), в чистом синтетическом тоннеле.

Кабель тоньше шага лидара по азимуту (0.15 град = 7.9 см на 30 м): попадёт ли в него столбец
лучей, зависит от смещения вбок. lat=0.4 -- смещение, при котором столбец на 30 м его задевает
(как и lat=0; при lat=0.1-0.3 кабель 2 см в синтетике не виден совсем).
"""
import warnings

import numpy as np
import pytest

from tunnel_od import ObstacleDetector
from tunnel_od.sim.lidar_sim import RAIL_HEIGHT, Z_FLOOR, simulate_frame
from tunnel_od.sim.shapes import Cable, make_shape

warnings.filterwarnings('ignore', category=RuntimeWarning)

RAIL_TOP = Z_FLOOR + RAIL_HEIGHT
DET = {'height': 3.0, 'far_top': 1.5}


@pytest.fixture(scope='module')
def frames():
    empty = simulate_frame()[:3]
    x, y, z, _, _, n = simulate_frame(shapes=[make_shape('cable', (0.02, 3.0, 0.0, 2.0), 30.0, 0.4, RAIL_TOP)])
    return empty, (x, y, z), n


def _run(frames_seq):
    det = ObstacleDetector(**DET)
    res = None
    for xyz in frames_seq:
        res = det.detect(*xyz, refit_path=True)
    return res


def test_cable_geometry():
    c = Cable(fwd=30.0, lat=0.4, z0=RAIL_TOP, diameter=0.02, length=3.0, bottom=2.0)
    p = np.array([0.4, 30.0, RAIL_TOP + 2.5])
    d = p / np.linalg.norm(p)
    t = c.intersect(np.stack([d, [0.0, 1.0, 0.0]]))
    assert t[0] == pytest.approx(np.linalg.norm(p) - 0.01, abs=2e-3)
    assert np.isinf(t[1])
    bent = Cable(fwd=30.0, lat=0.4, z0=RAIL_TOP, diameter=0.02, length=3.0, tilt_deg=10.0, sag=0.3)
    pts = bent.points()
    assert pts[0] == pytest.approx([0.4, 30.0, RAIL_TOP + 2.0])
    assert abs(pts[len(pts) // 2][0] - (0.4 + 1.5 * np.sin(np.radians(10.0)) - 0.3 * np.cos(np.radians(10.0)))) < 1e-9


def test_cable_2cm_at_30m_gives_stop(frames):
    empty, cable, n_hits = frames
    assert n_hits > 0
    res = _run([empty] * 3 + [cable] * 6)
    assert res['obstacle']
    assert abs(res['distance_m'] - 30.0) < 0.3
    stop = [o for o in res['objects'] if o['level'] == 'stop']
    assert stop and abs(stop[0]['lateral_m'] - 0.4) < 0.2 and stop[0]['low_m'] > 1.8


def test_no_cable_no_stop(frames):
    empty, _, _ = frames
    res = _run([empty] * 9)
    assert not res['obstacle'] and res['status'] == 'clear'
