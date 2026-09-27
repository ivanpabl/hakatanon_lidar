"""Тесты чистых функций офлайн-инструментов (tools/), без записей."""
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / 'tools'))
sys.path.insert(0, str(Path(__file__).resolve().parent.parent / 'research'))

from bags import EMPTY_BAGS, safe_messages          # noqa: E402
from eval_fake_obj import evaluate, group_x, summary_line          # noqa: E402
from compare_row import approach_median, first_stop_56, shares          # noqa: E402
from selflabel import episodes, label, segments          # noqa: E402


def test_safe_messages_stops_on_read_error():
    def broken():
        yield 1
        yield 2
        raise OSError('zstd checksum mismatch')

    errors = []
    assert list(safe_messages(broken(), errors)) == [1, 2]
    assert errors == ['OSError: zstd checksum mismatch']


def test_empty_bags_are_five_recordings_without_obstacle():
    assert len(EMPTY_BAGS) == 5 and 'doubleT_obstacle' not in EMPTY_BAGS and 'new_data' not in EMPTY_BAGS


def _ref():
    return [{'id': 1, 'desc': 'куб', 'expect': 'stop', 'x_m': 100.0},
            {'id': 5, 'desc': 'вне габарита', 'expect': 'not_stop', 'x_m': 200.0},
            {'id': 9, 'desc': 'не виден', 'expect': 'stop', 'x_m': 300.0}]


def test_group_x_splits_on_gap():
    pts = [(100.0, 1, {}), (101.0, 2, {}), (200.0, 3, {}), (102.0, 4, {}), (201.0, 5, {})]
    groups = group_x(pts, gap=15.0)
    assert [len(g) for g in groups] == [3, 2]


def test_evaluate_first_stop_share_and_false_frames():
    frames = [{'frame': k, 'travel_m': 2.0 * k, 'status': 'clear', 'sight_m': 150.0} for k in range(40)]
    objs = {}
    for k in range(5, 20):                                   # объект 1: X = 100, СТОП с кадра 5
        objs[k] = [{'distance_m': 100.0 - 2.0 * k, 'level': 'stop', 'reason': 'in_gauge'}]
        frames[k]['status'] = 'stop'
    for k in range(10, 12):                                  # объект 5: X = 200, ВНИМАНИЕ
        objs.setdefault(k, []).append({'distance_m': 200.0 - 2.0 * k, 'level': 'caution', 'reason': 'beyond_path'})
    objs[30] = [{'distance_m': 20.0, 'level': 'stop', 'reason': 'in_gauge'}]   # X = 80: ни в одно окно
    frames[30]['status'] = 'stop'
    rows, false_frames = evaluate(_ref(), frames, objs)
    r1, r5, r9 = rows
    assert r1['level'] == 'stop' and r1['first_frame'] == 5 and r1['first_stop_m'] == 90.0 and r1['sight_m'] == 150.0
    assert r1['stop_share'] == 1.0 and r1['ok']
    assert r5['level'] == 'caution' and r5['reason'] == 'beyond_path' and r5['ok']
    assert false_frames == [30]
    line = summary_line('t', rows, false_frames, 40)
    assert 'СТОП 1/2' in line and 'ложных СТОП-кадров 1 из 40' in line


def test_evaluate_unseen_object_is_none():
    frames = [{'frame': k, 'travel_m': float(k), 'status': 'clear', 'sight_m': 150.0} for k in range(5)]
    rows, false_frames = evaluate(_ref(), frames, {})
    assert rows[2]['level'] == 'none' and rows[2]['first_stop_m'] is None and not rows[2]['ok']
    assert false_frames == []


def test_evaluate_stop_on_out_of_gauge_object_is_not_ok():
    frames = [{'frame': 0, 'travel_m': 0.0, 'status': 'stop', 'sight_m': 150.0}]
    objs = {0: [{'distance_m': 199.0, 'level': 'stop', 'reason': 'in_gauge'}]}
    rows, _ = evaluate(_ref(), frames, objs)
    assert rows[1]['level'] == 'stop' and not rows[1]['ok']
    assert 'СТОП вне габарита: 5' in summary_line('t', rows, [], 1)


def test_compare_row_helpers():
    rows = [{'bag': 'doubleT_obstacle', 'frame': str(k), 'status': s, 'distance_m': d}
            for k, (s, d) in enumerate([('clear', ''), ('stop', '120.0'), ('stop', '56.2'), ('caution', ''), ('unknown', '')])]
    assert shares(rows) == pytest.approx((40.0, 20.0, 20.0))
    assert first_stop_56(rows) == (2, 56.2)
    ap = [{'shape': 'человек стоит', 'start_m': '170', 'detected': '1', 'first_m': '160'},
          {'shape': 'человек стоит', 'start_m': '170', 'detected': '1', 'first_m': '150'},
          {'shape': 'человек стоит', 'start_m': '90', 'detected': '1', 'first_m': '85'},
          {'shape': 'человек стоит', 'start_m': '170', 'detected': '0', 'first_m': ''}]
    assert approach_median(ap, 'человек стоит', 170) == 155.0
    assert approach_median(ap, 'куб 0.4') is None


def test_compare_row_missing_runs_show_dash(tmp_path, monkeypatch):
    import compare_row
    monkeypatch.setattr(compare_row, 'RUNS', tmp_path)                  # FAST=1: нет alarms_<tag>.csv
    cells = [c.strip() for c in compare_row.row('t', '{}').split('|')]
    assert cells[4] == '-' and cells[5] == '-'


def _rows(n, dt=0.1, disp=1.0, none_at=()):
    rows, travel = [], 0.0
    for k in range(n):
        d = None if k in none_at else disp
        travel += d or 0.0
        rows.append({'bag': 'new_data', 'frame': k, 't_s': k * dt, 'travel_m': travel, 'displacement_m': d})
    return rows


def test_segments_break_on_long_odometry_gap_and_restart():
    assert len(set(segments(_rows(30, none_at=range(10, 15))))) == 1          # 0,5 с без смещения
    seg = segments(_rows(40, none_at=range(10, 26)))                          # 1,6 с без смещения
    assert seg[9] == seg[25] == 0 and seg[26] == 1
    rows = _rows(10) + [dict(r, frame=100 + r['frame']) for r in _rows(10)]   # отрезок alarms.py --segments
    assert segments(rows)[10] == 1


def test_episodes_gap_rule():
    st = lambda frames: [(f, f, 7, 50.0, 'in_gauge') for f in frames]
    assert len(episodes(st([0, 1, 2, 8, 9]))) == 1        # пропущено 5 кадров (3..7) -- эпизод не рвётся
    assert len(episodes(st([0, 1, 2, 9, 10]))) == 2       # пропущено 6
    two = episodes(st([0, 1]) + [(f, f, 8, 30.0, 'in_gauge') for f in (0, 1)])
    assert len(two) == 2                                   # разные треки -- разные эпизоды


def test_label_false_proven_only_in_same_segment():
    rows = _rows(200)                                      # 1 м за кадр
    seg = segments(rows)
    ep = episodes([(10, 10, 1, 50.0, 'in_gauge')])[0]     # X = 11 + 50 = 61; нужно travel >= 61 + 3 + 1 = 65
    assert label(ep, rows, seg) == 'false_proven'
    assert label(ep, rows[:60], seg[:60]) == 'unresolved'
    rows2 = _rows(200, none_at=range(20, 40))              # сегмент рвётся до проезда X
    assert label(episodes([(10, 10, 1, 50.0, 'in_gauge')])[0], rows2, segments(rows2)) == 'unresolved'
