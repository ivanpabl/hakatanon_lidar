"""Правила против ложных СТОП (R1-R3, разбор fp_autopsy): переключаемые параметры ObstacleDetector,
по умолчанию выключены -- поведение базы не меняется."""
import warnings

import pytest

from tunnel_od import ObstacleDetector
from tunnel_od.detection.decision import decide, object_level
from tunnel_od.detection.tracking import EvidenceTracker
from tunnel_od.sim.lidar_sim import simulate_frame

warnings.filterwarnings('ignore', category=RuntimeWarning)


def _persist_run(top_m, n=8, **kw):
    """Слабый (1 точка при ожидаемых 5) неподвижный объект: подтверждение возможно только по persist."""
    tr = EvidenceTracker(threshold=2.2, decay=0.8, ego_check=True, persist_hits=6, **kw)
    travel, ob = 0.0, None
    for k in range(n):
        ob = {'distance_m': 40.0 - 1.2 * k, 'lateral_m': 0.0, 'n_points': 1, 'low_m': 0.1, 'height_m': top_m}
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
