"""Защита по входу узла детектора (ros2_ws/.../tunnel_od_detector/input_guard.py), без ROS 2:
watchdog «нет данных от лидара», прогрев, догон очереди, аргументы ros2 bag play.

    PYTHONPATH=core pytest tests/test_input_guard.py
"""
import json
import sys
from collections import deque
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / 'ros2_ws' / 'src' / 'tunnel_od_detector'))

from tunnel_od_detector.input_guard import (DEFAULT_READ_AHEAD, REASON_NO_DATA, REASON_NO_PATH,  # noqa: E402
                                            REASON_SHORT_SIGHT, REASON_WARMUP, STATE_FAULT, STATE_OK,
                                            STATE_WARMUP, Warmup, Watchdog, annotate, catch_up_enabled,
                                            fault_result, play_args, take_latest, watchdog_params)
from tunnel_od_detector.util import Stats, dumps, status_text  # noqa: E402

CONTRACT_STATUSES = {'stop', 'caution', 'clear', 'unknown'}


# --- watchdog ---------------------------------------------------------------------------------

def test_watchdog_grace_before_first_frame_then_timeout():
    wd = Watchdog(timeout_s=0.5, grace_s=2.0, t_start=100.0)
    assert wd.check(101.9) == (False, pytest.approx(1.9), False)
    fault, age, changed = wd.check(102.1)
    assert fault and changed and age == pytest.approx(2.1) and wd.episodes == 1
    assert wd.check(102.4)[2] is False                      # всё ещё fault, состояние не менялось
    assert wd.on_frame(102.5) is True                       # кадр -- провал закончился
    assert wd.check(102.9) == (False, pytest.approx(0.4), False)
    assert wd.check(103.1)[0] is True and wd.episodes == 2  # 0.6 с без кадра > 0.5
    assert wd.on_frame(103.2) is True and wd.on_frame(103.25) is False
    assert wd.check(103.3) == (False, pytest.approx(0.05), False)


def test_watchdog_grace_never_below_timeout_and_period():
    wd = Watchdog(timeout_s=1.0, grace_s=0.0, t_start=0.0)
    assert not wd.check(0.9)[0] and wd.check(1.1)[0]
    assert Watchdog(0.5).period_s() == pytest.approx(0.25)
    assert Watchdog(0.05).period_s() == pytest.approx(0.05)
    assert Watchdog(0.0).timeout_s > 0


def test_fault_result_is_contract_compatible_json():
    out = fault_result(0.73, 0.5, topic='/tunnel_od/cloud')
    assert out['status'] in CONTRACT_STATUSES and out['status'] == 'unknown'
    assert out['node_state'] == STATE_FAULT and out['reason'] == REASON_NO_DATA and out['fault'] is True
    assert out['obstacle'] is False and out['distance_m'] is None and out['clear_to_m'] == 0.0
    assert out['objects'] == [] and out['since_last_cloud_s'] == pytest.approx(0.73)
    assert out['frame'] is None and out['stamp'] is None   # снимок -- не кадр, номер не наследуется
    assert json.loads(dumps(out))['since_last_cloud_s'] == pytest.approx(0.73)
    text, rgb = status_text(out)
    assert text == 'NO LIDAR DATA 0.7 s' and rgb == (1.0, 0.2, 0.2)


def test_watchdog_params_for_bag_play_cover_start_pause_and_rate():
    """Штатный прогон: плеер на паузе start_delay с -- это не провал входа; порог под rate."""
    p = watchdog_params(6.0, rate=1.0)
    assert p == {'watchdog_grace_s': 8.0, 'watchdog_timeout_s': 0.5}
    wd = Watchdog(p['watchdog_timeout_s'], p['watchdog_grace_s'], t_start=0.0)
    assert not wd.check(6.0)[0] and not wd.check(7.9)[0]      # кадры пошли на 6-й с, запас 2 с
    assert wd.check(8.1)[0]                                    # а вот если их всё нет -- fault
    assert watchdog_params(3.0, rate=0.25)['watchdog_timeout_s'] == pytest.approx(2.0)
    assert watchdog_params(3.0, rate=2.0)['watchdog_timeout_s'] == 0.5   # не ниже 0.5
    with pytest.raises(ValueError):
        watchdog_params(3.0, rate=0)


# --- прогрев ----------------------------------------------------------------------------------

def test_warmup_lasts_min_frames_and_ends_without_path():
    w = Warmup(min_frames=3)
    assert w.active
    assert w.update(False) and w.update(True) and w.update(True)   # 3 кадра -- прогрев
    assert not w.active and w.path_seen and w.update(True) is False  # 4-й: решение детектора
    w = Warmup(min_frames=2)                                          # ось не строится вовсе (пустая запись)
    assert w.update(False) and w.update(False)
    assert w.update(False) is False and not w.path_seen               # прогрев кончился по кадрам
    assert annotate({'status': 'unknown', 'path_available': False}, w.update(False))['reason'] == REASON_NO_PATH
    assert Warmup(0).active is False and Warmup(0).update(False) is False


def test_warmup_reset_after_fault():
    w = Warmup(min_frames=1)
    assert w.update(True) is True and not w.active and w.restarts == 0
    w.reset()
    assert w.active and w.restarts == 1 and w.update(True) is True and w.update(True) is False


def test_annotate_warmup_masks_clear_but_not_stop():
    out = annotate({'status': 'clear', 'clear_to_m': 120.0, 'path_available': True}, warming=True)
    assert out['status'] == 'unknown' and out['clear_to_m'] == 0.0
    assert out['node_state'] == STATE_WARMUP and out['reason'] == REASON_WARMUP
    out = annotate({'status': 'caution', 'clear_to_m': 80.0, 'path_available': True}, warming=True)
    assert out['status'] == 'unknown'
    out = annotate({'status': 'stop', 'distance_m': 56.0, 'clear_to_m': 56.0, 'path_available': True}, warming=True)
    assert out['status'] == 'stop' and out['clear_to_m'] == 56.0 and out['node_state'] == STATE_WARMUP
    assert status_text(out)[0] == 'STOP 56 m'
    assert status_text({'status': 'unknown', 'node_state': 'warmup'})[0] == 'WARMING UP'


def test_annotate_ok_reasons_for_detector_unknown():
    out = annotate({'status': 'clear', 'clear_to_m': 120.0, 'path_available': True}, warming=False)
    assert out == {'status': 'clear', 'clear_to_m': 120.0, 'path_available': True, 'node_state': STATE_OK, 'reason': None}
    assert annotate({'status': 'unknown', 'path_available': False}, False)['reason'] == REASON_NO_PATH
    assert annotate({'status': 'unknown', 'path_available': True}, False)['reason'] == REASON_SHORT_SIGHT
    for st in CONTRACT_STATUSES:
        assert annotate({'status': st, 'path_available': True}, False)['status'] in CONTRACT_STATUSES
        assert annotate({'status': st, 'path_available': True}, True)['status'] in CONTRACT_STATUSES


# --- догон очереди ----------------------------------------------------------------------------

def test_take_latest_keeps_newest_counts_dropped():
    q = deque([('m0', 0.0), ('m1', 0.1), ('m2', 0.2)])
    assert take_latest(q) == (('m2', 0.2), 2) and not q
    assert take_latest(q) == (None, 0)
    q.append(('m3', 0.3))
    assert take_latest(q) == (('m3', 0.3), 0)


def test_take_latest_feeds_stats_like_node():
    s = Stats()
    q = deque([1, 2, 3, 4])
    item, n = take_latest(q)
    s.dropped_catchup += n
    s.dropped += 1                                   # один кадр отброшен колбэком: очередь была полна
    out = s.summary()
    assert item == 4 and out['dropped_stale'] == 1 and out['dropped_catchup'] == 3 and out['dropped_total'] == 4
    assert out['fault_snapshots'] == 0 and out['fault_episodes'] == 0 and out['warmup_frames'] == 0


def test_catch_up_enabled_auto_follows_max_pending():
    assert catch_up_enabled('auto', 2) and not catch_up_enabled('auto', 100000)
    assert catch_up_enabled('true', 100000) and catch_up_enabled(True, 100000)
    assert not catch_up_enabled('false', 2) and not catch_up_enabled(False, 2)
    with pytest.raises(ValueError):
        catch_up_enabled('maybe', 2)


# --- ros2 bag play ----------------------------------------------------------------------------

def test_play_args_read_ahead_and_flags():
    cmd = play_args('/bag', rate=0.5, read_ahead='10', loop=True, qos_file='/tmp/q.yaml')
    assert cmd[:4] == ['ros2', 'bag', 'play', '/bag']
    assert cmd[cmd.index('--read-ahead-queue-size') + 1] == '10'
    assert cmd[cmd.index('--rate') + 1] == '0.5'
    assert '--start-paused' in cmd and '--loop' in cmd and '--disable-keyboard-controls' in cmd
    assert cmd[cmd.index('--qos-profile-overrides-path') + 1] == '/tmp/q.yaml'
    cmd = play_args('/bag')
    assert cmd[cmd.index('--read-ahead-queue-size') + 1] == str(DEFAULT_READ_AHEAD) == '20'
    assert '--loop' not in cmd and '--qos-profile-overrides-path' not in cmd
    assert '--start-paused' not in play_args('/bag', start_paused=False)
    for bad in (0, -1, 'x'):
        with pytest.raises(ValueError):
            play_args('/bag', read_ahead=bad)


def test_play_launch_declares_read_ahead_argument():
    """launch-файл: аргумент read_ahead объявлен и передаётся в play_args (без запуска ros2)."""
    import ast
    src = (Path(__file__).resolve().parent.parent / 'ros2_ws/src/tunnel_od_detector/launch/play.launch.py').read_text()
    tree = ast.parse(src)
    declared = {n.args[0].value for n in ast.walk(tree)
                if isinstance(n, ast.Call) and getattr(n.func, 'id', '') in ('arg', 'DeclareLaunchArgument')
                and n.args and isinstance(n.args[0], ast.Constant)}
    used = {n.args[0].value for n in ast.walk(tree)
            if isinstance(n, ast.Call) and getattr(n.func, 'id', '') == 'lc'
            and n.args and isinstance(n.args[0], ast.Constant)}
    assert 'read_ahead' in declared and used <= declared, used - declared
    assert 'play_args(' in src and "'--read-ahead-queue-size', '20'" not in src
