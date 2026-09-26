"""Части узла без ROS 2: параметры детектора и сериализация результата."""
import json
import math

import numpy as np
import pytest

from tunnel_od_detector.util import Stats, build_detector_kwargs, dumps, to_jsonable


def _target(self, near_cutoff=2.0, zone=None, method='zone', path_margin=10.0):
    pass


def test_kwargs_pass_through_and_unknown_dropped():
    warns = []
    kw = build_detector_kwargs({'near_cutoff': 3.0, 'method': 'background', 'new_arg': 1},
                               target=_target, warn=warns.append)
    assert kw == {'near_cutoff': 3.0, 'method': 'background'}
    assert len(warns) == 1 and 'new_arg' in warns[0]


def test_kwargs_zone_flat_list_and_none():
    kw = build_detector_kwargs({'zone': [0.15, 0.35, 1.2, 0.35, 3.7, 1.35], 'path_margin': 'none'}, target=_target)
    assert kw['zone'] == ((0.15, 0.35, 1.2), (0.35, 3.7, 1.35))
    assert kw['path_margin'] is None
    with pytest.raises(ValueError):
        build_detector_kwargs({'zone': [0.1, 0.2]})


def test_kwargs_json_override():
    kw = build_detector_kwargs({'near_cutoff': 3.0}, '{"near_cutoff": 5, "zone": [[0.1, 1, 0.9]]}', target=_target)
    assert kw == {'near_cutoff': 5, 'zone': ((0.1, 1.0, 0.9),)}


def test_kwargs_zone_constant():
    pytest.importorskip('tunnel_od')
    import tunnel_od
    assert build_detector_kwargs({'zone': 'GAUGE_METRO'})['zone'] == tunnel_od.GAUGE_METRO


def test_json_numpy_nan_unknown():
    class Foo:
        def __str__(self):
            return 'foo'
    res = {'obstacle': np.bool_(True), 'distance_m': np.float32(56.25), 'n': np.int64(3),
           'nan': float('nan'), 'inf': np.float64(np.inf), 'arr': np.arange(3, dtype=np.int16),
           'objects': [{'distance_m': np.float64(1.5), 'confirmed': np.bool_(False)}],
           'tuple': (1, 2), 'none': None, 'obj': Foo(), 3: 'int key', 'clear_to_m': np.float32(120.0)}
    out = json.loads(dumps(res))
    assert out['obstacle'] is True and out['distance_m'] == 56.25 and out['n'] == 3
    assert out['nan'] is None and out['inf'] is None
    assert out['arr'] == [0, 1, 2] and out['tuple'] == [1, 2] and out['obj'] == 'foo' and out['3'] == 'int key'
    assert out['objects'][0] == {'distance_m': 1.5, 'confirmed': False}
    assert to_jsonable(np.array([np.nan, 1.0])) == [None, 1.0]


def test_stats():
    s = Stats()
    for i, t in enumerate([0.0, 0.1, 0.2, 0.5, 0.6]):     # 0.2 -> 0.5: пропущено 2 кадра
        s.on_receive(t)
        s.on_processed(10.0 + i, 5.0, i == 4, 56.0 if i == 4 else None)
    out = s.summary()
    assert out['received'] == 5 and out['processed'] == 5 and out['stamp_gaps'] == 2
    assert out['alarm_frames'] == 1 and out['min_alarm_distance_m'] == 56.0
    assert math.isclose(out['latency_ms_p50'], 12.0)
