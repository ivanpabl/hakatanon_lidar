"""Тесты ядра без записей: синтетический тоннель (tunnel_od.sim.lidar_sim) и простые случаи.

    pip install -e ./core pytest && pytest tests
"""
import warnings

import numpy as np
import pytest

from tunnel_od import ObstacleDetector, parse_pointcloud2
from tunnel_od.detection.decision import axis_curvature, decide, object_level, sight_distance
from tunnel_od.detection.tracking import EvidenceTracker
from tunnel_od.geometry.path import extend_path_by_walls, splice_far_axis
from tunnel_od.pointcloud import COLUMN_HEIGHT, FrameRepeat, dedupe_rounded, hesai_columns
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


def _cloud16(x, y, z):
    """16-байтное облако синтетики организаторов: x, y, z, intensity (float32), без ring и времени."""
    dt = np.dtype([('x', '<f4'), ('y', '<f4'), ('z', '<f4'), ('intensity', '<f4')])
    a = np.zeros(len(x), dt)
    a['x'], a['y'], a['z'] = x, y, z
    return a.tobytes(), dt.itemsize, [_Field(n, dt.fields[n][1]) for n in dt.names]


def test_parse_unordered_16_byte_cloud_drops_duplicates():
    rng = np.random.default_rng(0)
    base = rng.uniform(-20, 20, (200, 3)).astype(np.float32)
    pts = np.vstack([base, base[:100]])                      # 300 точек: не кратно 256, 100 дублей
    data, step, fields = _cloud16(pts[:, 0], pts[:, 1], pts[:, 2])
    assert step == 16
    px, py, pz = parse_pointcloud2(data, step, fields)
    assert len(px) == 200
    assert np.array_equal(px, base[:, 0])                    # порядок первых вхождений сохранён


def test_parse_ordered_cloud_keeps_close_points_outside_dual_pair():
    """Упорядоченное облако: дубли снимаются только внутри пары столбцов, запасной способ не включается."""
    n_cols = 4
    x = np.arange(n_cols * COLUMN_HEIGHT, dtype=np.float32) * 0.01 + 1.0      # 512 точек: пары (0,1), (2,3)
    x.reshape(n_cols, COLUMN_HEIGHT)[2] = x.reshape(n_cols, COLUMN_HEIGHT)[0]   # столбец 2 = столбец 0 (другая пара)
    y, z = np.full_like(x, -5.0), np.zeros_like(x)
    data, step, fields = _cloud_bytes(x, y, z)
    px, _, _ = parse_pointcloud2(data, step, fields)
    assert len(px) == n_cols * COLUMN_HEIGHT


def test_parse_empty_and_all_zero_unordered_cloud():
    data, step, fields = _cloud16(np.zeros(0), np.zeros(0), np.zeros(0))
    assert all(len(a) == 0 for a in parse_pointcloud2(data, step, fields))
    data, step, fields = _cloud16(np.zeros(300), np.zeros(300), np.zeros(300))
    assert all(len(a) == 0 for a in parse_pointcloud2(data, step, fields))
    assert all(len(a) == 0 for a in dedupe_rounded(np.zeros(0, np.float32), np.zeros(0, np.float32), np.zeros(0, np.float32)))


def test_hesai_columns_truth_table():
    """Hesai dual-return -- только кратное 2*COLUMN_HEIGHT облако с полем timestamp
    (fields=None -- запись по умолчанию, старый 26-байтный layout, тоже Hesai)."""
    _, _, fields26 = _cloud_bytes(np.zeros(1), np.zeros(1), np.zeros(1))
    _, _, fields16 = _cloud16(np.zeros(1), np.zeros(1), np.zeros(1))
    assert hesai_columns(512, fields26) is True
    assert hesai_columns(300, fields26) is False
    assert hesai_columns(512, fields16) is False
    assert hesai_columns(512, None) is True


def test_parse_16_byte_512_point_cloud_dedupes_by_rounding_not_columns():
    """16-байтное облако кратно 2*COLUMN_HEIGHT=256, но без timestamp -- это не Hesai
    (синтетика организаторов), дубли снимает округление, а не пара столбцов."""
    rng = np.random.default_rng(1)
    base = rng.uniform(-20, 20, (312, 3)).astype(np.float32)
    pts = np.vstack([base, base[:200]])                      # 512 точек: кратно 256, 200 дублей
    assert len(pts) == 512 and len(pts) % (2 * COLUMN_HEIGHT) == 0
    data, step, fields = _cloud16(pts[:, 0], pts[:, 1], pts[:, 2])
    px, py, pz = parse_pointcloud2(data, step, fields)
    assert len(px) == 312


def test_frame_repeat_detects_bitwise_same_cloud():
    rep = FrameRepeat()
    a = np.arange(1000, dtype=np.uint8).tobytes()
    b = bytes(reversed(a))
    assert not rep.check(a)
    assert rep.check(a)
    assert not rep.check(b)
    assert rep.check(np.frombuffer(b, np.uint8))             # numpy-массив (rosbags) и bytes -- одно и то же


def _o(d, **kw):
    o = {'distance_m': d, 'confirmed': False, 'beyond_path': False, 'held': False, 'edge_line': False,
         'hits': 0, 'evidence': 0.0}
    o.update(kw)
    return o


def _decide(objs, path=True, path_range=150.0, curvature=0.0):
    return decide(objs, path_available=path, path_range=path_range if path else None, curvature=curvature,
                  pending_score=1.1)


def test_object_level_table_and_reason_order():
    lv = lambda o, in_path=True: object_level(o, in_path=in_path, pending_score=1.1)
    assert lv(_o(50, confirmed=True)) == ('stop', 'in_gauge')
    assert lv(_o(50, confirmed=True), in_path=False) == ('caution', 'beyond_path')
    assert lv(_o(50, confirmed=True, ego_carried=True)) == ('caution', 'ego_carried')
    assert lv(_o(50, confirmed=True, ego_carried=True), in_path=False) == ('caution', 'beyond_path')
    assert lv(_o(50, hits=2, evidence=1.1)) == ('caution', 'pending')
    assert lv(_o(50, hits=1, evidence=5.0)) == (None, None)
    assert lv(_o(50, hits=3, evidence=1.0)) == (None, None)
    assert lv(_o(50, edge_line=True, hits=5, evidence=5.0)) == (None, None)
    assert lv(_o(50, held=True, level='stop', reason='in_gauge'), in_path=False) == ('stop', 'in_gauge')


def test_decide_status_priority():
    assert _decide([_o(50, confirmed=True), _o(80, confirmed=True, beyond_path=True)])['status'] == 'stop'
    held = _o(50, confirmed=True, held=True, level='stop', reason='in_gauge')
    assert _decide([held], path=False)['status'] == 'stop'                       # stop > unknown
    assert _decide([_o(80, confirmed=True, beyond_path=True)], path=False)['status'] == 'unknown'
    assert _decide([_o(80, confirmed=True, beyond_path=True)], path_range=60.0)['status'] == 'caution'
    assert _decide([_o(80, hits=1)])['status'] == 'clear'


def test_decide_distances():
    r = _decide([_o(70, confirmed=True), _o(40, confirmed=True), _o(30, confirmed=True, beyond_path=True)])
    assert r['distance_m'] == 40 and r['caution_distance_m'] == 30
    r = _decide([_o(80, hits=2, evidence=2.0)])
    assert r['distance_m'] is None and r['caution_distance_m'] == 80


def test_sight_and_clear_to():
    r = _decide([], path=False)
    assert r['sight_m'] == 0.0 and r['clear_to_m'] == 0.0
    assert _decide([], path_range=250.0)['sight_m'] == 200.0
    assert _decide([], path_range=143.0)['sight_m'] == 143.0
    assert _decide([], path_range=250.0, curvature=1 / 300)['sight_m'] == pytest.approx(60.0)   # sqrt(8*300*1.5)
    assert _decide([], path_range=250.0, curvature=1 / 6000)['sight_m'] == 200.0                # |k| < 1/5000
    r = _decide([_o(40, confirmed=True)], path_range=150.0)
    assert r['clear_to_m'] == 40 and r['sight_m'] == 150.0
    assert sight_distance(None, 0.0) == 0.0


def test_decide_clear_to_never_negative():
    held = _o(-2.0, confirmed=True, held=True, level='stop', reason='in_gauge')
    r = _decide([held])
    assert r['status'] == 'stop' and r['clear_to_m'] == 0.0


def test_axis_curvature_of_arc():
    R = 300.0
    f = np.arange(2.0, 150.0, 1.0)
    assert axis_curvature(f, f ** 2 / (2 * R)) == pytest.approx(1 / R, rel=1e-3)
    assert axis_curvature(f, 0.01 * f) == pytest.approx(0.0, abs=1e-9)
    assert axis_curvature(np.arange(2.0, 45.0, 1.0), np.zeros(43)) == 0.0         # до 45 м: на 40-120 мало точек


def test_detector_status_fields_clear_and_stop():
    det = ObstacleDetector()
    res = _run(det, [{}] * 3)
    assert res['status'] == 'clear' and not res['obstacle']
    assert 100 < res['clear_to_m'] <= 200 and res['sight_m'] >= res['clear_to_m']
    assert res['travel_m'] == pytest.approx(0.0, abs=1.0)
    res = _run(det, [{'obstacle_forward': 30.0, 'obstacle_radius': 0.35}] * 4)
    assert res['status'] == 'stop' and res['obstacle'] is True
    assert res['clear_to_m'] == pytest.approx(res['distance_m'])
    stop = [o for o in res['objects'] if o['level'] == 'stop']
    assert stop and all(o['reason'] == 'in_gauge' for o in stop)
    assert all('rail_z_m' in o for o in res['objects'])


def test_detector_unknown_without_path():
    det = ObstacleDetector()
    x, y, z, *_ = simulate_frame()
    res = det.check_frame(x, y, z)                    # путь ещё не считался
    assert res['status'] == 'unknown' and not res['obstacle']
    assert res['sight_m'] == 0.0 and res['clear_to_m'] == 0.0


def test_held_object_inherits_level():
    det = ObstacleDetector()
    _run(det, [{}] * 3)
    _run(det, [{'obstacle_forward': 30.0, 'obstacle_radius': 0.35}] * 8)
    res = _run(det, [{}])
    held = [o for o in res['objects'] if o.get('held')]
    assert held and held[0]['level'] == 'stop' and held[0]['reason'] == 'in_gauge' and 'track_id' in held[0]


def _ego_run(disp, dist, n=8, travel_none=False, **kw):
    tr = EvidenceTracker(ego_check=True, **kw)
    travel, ob = 0.0, None
    for k in range(n):
        ob = {'distance_m': dist(k), 'lateral_m': 0.0, 'n_points': 20}
        tr.update([ob], displacement=disp, expected=lambda d: 5.0, travel=None if travel_none else travel)
        travel += disp
    return ob


def test_ego_constant_distance_is_carried():
    ob = _ego_run(1.2, lambda k: 30.0)
    assert ob['ego_carried'] and ob['ego_slope'] == pytest.approx(0.0, abs=1e-6)


def test_ego_approaching_object_is_not_carried():
    ob = _ego_run(1.2, lambda k: 60.0 - 1.2 * k)
    assert not ob['ego_carried'] and ob['ego_slope'] == pytest.approx(-1.0, abs=1e-6)


def test_ego_no_decision_when_train_stands_or_travel_unknown():
    ob = _ego_run(0.0, lambda k: 30.0)
    assert ob['ego_slope'] is None and not ob['ego_carried']
    ob = _ego_run(1.2, lambda k: 30.0, travel_none=True)
    assert ob['ego_slope'] is None and not ob['ego_carried']


def test_ego_short_travel_no_decision():
    ob = _ego_run(1.2, lambda k: 30.0, n=4)          # пробег 3,6 м < 4 м
    assert ob['ego_slope'] is None and not ob['ego_carried']


def test_ego_check_off_only_measures():
    tr = EvidenceTracker(ego_check=False)
    travel = 0.0
    for k in range(8):
        ob = {'distance_m': 30.0, 'lateral_m': 0.0, 'n_points': 20}
        tr.update([ob], displacement=1.2, expected=lambda d: 5.0, travel=travel)
        travel += 1.2
    assert ob['ego_slope'] == pytest.approx(0.0, abs=1e-6) and not ob['ego_carried']


def test_detector_passes_ego_params():
    det = ObstacleDetector(ego_check=True, ego_min_travel=5.0, ego_max_slope=-0.5)
    assert det._tracker.ego_check and det._tracker.ego_min_travel == 5.0 and det._tracker.ego_max_slope == -0.5


def test_held_track_not_held_past_near_cutoff():
    tr = EvidenceTracker(hold=3, hold_min=1)
    for k in range(4):                      # подтверждённый объект подъезжает к 3 м, тревога
        ob = {'distance_m': 6.0 - k, 'lateral_m': 0.0, 'n_points': 50}
        tr.update([ob], displacement=1.0, expected=lambda d: 5.0)
        tr.set_alarm({ob['track_id']})
    tr.update([], displacement=1.0)          # пропал: прогноз 2 м -- ещё держится
    assert [round(t['distance_m'], 1) for t in tr.held(min_distance=2.0)] == [2.0]
    tr.update([], displacement=1.0)          # прогноз 1 м -- уже проехали, не держится
    assert tr.held(min_distance=2.0) == [] and len(tr.held()) == 1
