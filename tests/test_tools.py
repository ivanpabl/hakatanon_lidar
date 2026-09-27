"""Тесты чистых функций офлайн-инструментов (tools/), без записей."""
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / 'tools'))

from bags import EMPTY_BAGS, safe_messages          # noqa: E402
from eval_fake_obj import evaluate, group_x, summary_line          # noqa: E402


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
