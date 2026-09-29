"""Правила против ложных СТОП (R1-R3, разбор fp_autopsy): переключаемые параметры ObstacleDetector.
R1 (с гейтом по lat) и R2 включены по умолчанию, R3 выключен; OFF -- поведение базы a61fe29."""
import warnings

import numpy as np
import pytest

from tunnel_od import ObstacleDetector
from tunnel_od.detection.clustering import min_points_at
from tunnel_od.detection.decision import decide, object_level
from tunnel_od.detection.tracking import EvidenceTracker
from tunnel_od.sim.lidar_sim import simulate_frame

warnings.filterwarnings('ignore', category=RuntimeWarning)


def _persist_run(top_m, n=8, lateral_m=0.0, **kw):
    """Слабый (1 точка при ожидаемых 5) неподвижный объект: подтверждение возможно только по persist."""
    tr = EvidenceTracker(threshold=2.2, decay=0.8, ego_check=True, persist_hits=6, **kw)
    travel, ob = 0.0, None
    for k in range(n):
        ob = {'distance_m': 40.0 - 1.2 * k, 'lateral_m': lateral_m, 'n_points': 1, 'low_m': 0.1, 'height_m': top_m}
        tr.update([ob], displacement=1.2, expected=lambda d: 5.0, travel=travel)
        travel += 1.2
    return ob


def test_r1_persist_min_top_rejects_low_object_and_off_by_default():
    ob = _persist_run(0.3)
    assert ob['evidence'] < 2.2 and ob['persistent'] and ob['confirmed']
    ob = _persist_run(0.3, persist_min_top=0.5)
    assert not ob['persistent'] and not ob['confirmed']
    ob = _persist_run(2.9, persist_min_top=0.5)
    assert ob['persistent'] and ob['confirmed']


def test_r1_lat_gate_keeps_low_object_on_axis_and_rejects_at_rail_head():
    ob = _persist_run(0.2, persist_min_top=0.5, persist_min_top_lat=0.55)
    assert ob['persistent'] and ob['confirmed']  # на оси (lateral 0) низкий объект подтверждается
    ob = _persist_run(0.2, persist_min_top=0.5, persist_min_top_lat=0.55, lateral_m=0.72)
    assert not ob['persistent'] and not ob['confirmed']  # у головки рельса -- нет
    ob = _persist_run(0.2, persist_min_top=0.5, persist_min_top_lat=None, lateral_m=0.0)
    assert not ob['persistent']  # без гейта по lat правило действует для всех


def _o(d, **kw):
    o = {'distance_m': d, 'confirmed': True, 'beyond_path': False, 'held': False, 'edge_line': False,
         'hits': 6, 'evidence': 3.0, 'low_m': 0.3, 'track_id': 1}
    o.update(kw)
    return o


def test_r2_front_lift_demotes_to_caution():
    lv = lambda o, **kw: object_level(o, in_path=True, pending_score=1.1, **kw)
    lifted = _o(140, front_maxh=0.5)
    assert lv(lifted) == ('stop', 'in_gauge')                              # выключено
    assert lv(lifted, front_lift_m=0.1) == ('caution', 'front_lift')
    assert lv(_o(140, front_maxh=0.35), front_lift_m=0.1) == ('stop', 'in_gauge')   # ниже порога
    assert lv(_o(140, front_maxh=2.6, low_m=2.4), front_lift_m=0.1) == ('stop', 'in_gauge')  # низ >= 2 м
    assert lv(_o(140, front_maxh=None), front_lift_m=0.1) == ('stop', 'in_gauge')  # признака нет
    assert lv(_o(140), front_lift_m=0.1) == ('stop', 'in_gauge')
    assert lv(_o(140, front_maxh=0.5, implausible=True), front_lift_m=0.1) == ('caution', 'surface_behind')
    assert lv(_o(140, front_maxh=0.5, held=True, level='stop', reason='in_gauge'), front_lift_m=0.1) == ('stop', 'in_gauge')


def _decide(objs, **kw):
    return decide(objs, path_available=True, path_range=150.0, curvature=0.0, pending_score=1.1, **kw)


def test_r3_far_stop_needs_previous_frame():
    r = _decide([_o(50)])
    assert r['status'] == 'stop' and r['stop_ids'] == {1}                    # выключено
    r = _decide([_o(50)], stop_confirm_far_m=30.0)
    assert r['status'] == 'caution' and r['distance_m'] is None and r['stop_ids'] == {1}
    o = _o(50)
    r = _decide([o], stop_confirm_far_m=30.0, prev_stop_ids=r['stop_ids'])
    assert r['status'] == 'stop' and (o['level'], o['reason']) == ('stop', 'in_gauge')
    o = _o(50)
    _decide([o], stop_confirm_far_m=30.0, prev_stop_ids={7})
    assert (o['level'], o['reason']) == ('caution', 'far_unconfirmed')
    o = _o(20)
    assert _decide([o], stop_confirm_far_m=30.0)['status'] == 'stop'         # ближе порога -- сразу


def _run(det, frames):
    res = None
    for kw in frames:
        x, y, z, *_ = simulate_frame(**kw)
        res = det.detect(x, y, z, refit_path=True, stamp=None)
    return res


@pytest.mark.parametrize('kwargs', [{}, {'persist_min_top_m': 0.5, 'front_lift_caution_m': 0.1,
                                         'front_lift_max_low_m': 2.0, 'stop_confirm_far_m': 30.0}])
def test_detector_accepts_rules_and_still_stops_on_real_obstacle(kwargs):
    det = ObstacleDetector(**kwargs)
    res = _run(det, [{}] * 3)
    assert res['status'] == 'clear'
    res = _run(det, [{'obstacle_forward': 30.0, 'obstacle_radius': 0.35}] * 5)
    assert res['status'] == 'stop'
    stop = [o for o in res['objects'] if o['level'] == 'stop']
    assert stop and all(o['reason'] == 'in_gauge' for o in stop)
    assert det._prev_stop_ids == {o['track_id'] for o in stop}


def test_r3_memory_survives_tiny_frame():
    det = ObstacleDetector(stop_confirm_far_m=30.0)
    _run(det, [{}] * 3)
    res = _run(det, [{'obstacle_forward': 34.0, 'obstacle_radius': 0.35}] * 5)
    assert res['status'] == 'stop' and res['distance_m'] >= 30.0
    ids = det._prev_stop_ids
    assert det.check_frame(np.zeros(3), np.zeros(3), np.zeros(3))['status'] == 'unknown'
    assert det._prev_stop_ids == ids
    res = _run(det, [{'obstacle_forward': 34.0, 'obstacle_radius': 0.35}])
    assert res['status'] == 'stop' and all(o['reason'] != 'far_unconfirmed' for o in res['objects'])


# --- exp4: накопление evidence вдали (far_acc_*), по умолчанию выключено

def test_expected_points_far_acc():
    det = ObstacleDetector()
    for d in (50.0, 150.0):
        assert det._expected_points(d) == min_points_at(d, det.min_points_k, det.min_points_floor)
    det = ObstacleDetector(far_acc_from=100.0, far_acc_floor=1.5)
    assert det._expected_points(50.0) == min_points_at(50.0, det.min_points_k, det.min_points_floor)
    assert det._expected_points(150.0) == 1.5


def test_far_min_hits_in_evidence_tracker():
    def run(n, **kw):
        tr = EvidenceTracker(threshold=2.2, decay=1.0, min_hits=3, **kw)
        for _ in range(n):
            ob = {'distance_m': 150.0, 'lateral_m': 0.0, 'n_points': 20}
            tr.update([ob], displacement=0.0, expected=lambda d: 5.0)
        return ob['confirmed']
    assert run(3)
    assert not run(3, far_min_hits=(100.0, 5)) and run(5, far_min_hits=(100.0, 5))
    assert run(3, far_min_hits=(200.0, 5))  # ближе порога -- как раньше
    tr = EvidenceTracker(2.2, 0.8, 3, 1.0)   # позиционный вызов: 4-й аргумент -- gate_fwd, не far_min_hits
    assert tr.gate_fwd == 1.0 and tr.far_min_hits is None


@pytest.mark.parametrize('floor', [1.5, 1.0])
def test_too_small_follows_expected_points(floor):
    """too_small считается тем же _expected_points, что и evidence: с far_acc_floor=1.0 объект из
    1 точки вдали не too_small, с 1.5 -- too_small (1 < 1.5), но evidence он всё равно копит."""
    det = ObstacleDetector(far_acc_from=100.0, far_acc_floor=floor)
    _run(det, [{}] * 2)
    res = _run(det, [{'obstacle_forward': 150.0, 'obstacle_radius': 0.35}] * 6)
    far = [o for o in res['objects'] if not o.get('held') and o['n_points'] and o['distance_m'] > 100.0]
    assert far and all(o['too_small'] == (o['n_points'] < det._expected_points(o['distance_m'])) for o in far)
    assert any(o['level'] == 'stop' for o in far)
    assert all(o['too_small'] == (o['n_points'] < floor) for o in far)


OFF = {'persist_min_top_m': 0.0, 'front_lift_caution_m': None, 'front_lift_max_low_m': 2.0,
       'stop_confirm_far_m': None, 'far_acc_from': None, 'far_acc_floor': 1.5, 'far_acc_min_hits': 4,
       'range_hold': 0}


ON = {'persist_min_top_m': 0.5, 'persist_min_top_lat_m': 0.55, 'front_lift_caution_m': 0.1,
      'front_lift_max_low_m': 2.0, 'stop_confirm_far_m': None}


def test_defaults_are_r1_lat_gated_plus_r2():
    d = ObstacleDetector()
    assert (d._tracker.persist_min_top, d._tracker.persist_min_top_lat) == (0.5, 0.55)
    assert d.front_lift_caution_m == 0.1 and d.stop_confirm_far_m is None


def test_defaults_identical_to_explicit_on_full_result():
    """Байт-в-байт: детектор с {} и с явно заданными значениями по умолчанию дают одинаковый результат
    (весь dict, включая objects) на пустом, крошечном и препятственном кадрах; OFF на настоящем
    препятствии тоже СТОП."""
    a, b = ObstacleDetector(), ObstacleDetector(**ON)
    frames = [{}] * 3 + [{'obstacle_forward': 40.0, 'obstacle_radius': 0.35}] * 5 + [{}]
    for kw in frames:
        x, y, z, *_ = simulate_frame(**kw)
        ra, rb = a.detect(x, y, z, refit_path=True), b.detect(x, y, z, refit_path=True)
        assert ra == rb
    tiny = np.zeros(3)
    assert a.check_frame(tiny, tiny, tiny) == b.check_frame(tiny, tiny, tiny)
    off = ObstacleDetector(**OFF)
    for kw in frames[:-1]:
        x, y, z, *_ = simulate_frame(**kw)
        res = off.detect(x, y, z, refit_path=True)
    assert res['status'] == 'stop'
