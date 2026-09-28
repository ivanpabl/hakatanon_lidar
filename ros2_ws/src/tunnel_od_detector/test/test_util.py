"""Части узла без ROS 2: параметры детектора и сериализация результата."""
import json
import math

import numpy as np
import pytest

from tunnel_od_detector.util import (Stats, build_detector_kwargs, canonical_xyz, dumps, is_canonical,
                                     limit_alarms, required_fwd_range, to_jsonable)


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


class _Field:
    def __init__(self, name, offset, datatype=7):
        self.name, self.offset, self.datatype = name, offset, datatype


class _Cloud:
    def __init__(self, data, point_step=12, fields=None):
        self.data, self.point_step, self.is_bigendian = data, point_step, False
        self.fields = fields if fields is not None else [_Field('x', 0), _Field('y', 4), _Field('z', 8)]


def test_canonical_xyz_views_without_copy():
    xyz = np.arange(3 * 256, dtype=np.float32).reshape(-1, 3)     # 256 точек: parse_pointcloud2 удалил бы "дубли"
    buf = xyz.tobytes()
    x, y, z = canonical_xyz(_Cloud(buf))
    assert len(x) == 256 and np.array_equal(x, xyz[:, 0]) and np.array_equal(z, xyz[:, 2])
    assert not x.flags.owndata
    assert not is_canonical(_Cloud(buf, 16))
    with pytest.raises(ValueError):
        canonical_xyz(_Cloud(buf, 12, [_Field('y', 0), _Field('x', 4), _Field('z', 8)]))


def test_default_crop_is_safe_for_core():
    """Обрезка tunnel_od_preproc по умолчанию (2..250 м) не задевает то, что берёт ядро."""
    pytest.importorskip('tunnel_od')
    lo, hi = required_fwd_range({})
    assert lo >= 2.0 - 1e-9 and hi <= 250.0 + 1e-9
    assert required_fwd_range({'max_range': 300.0})[1] == 300.0


def _res(*objs):
    return {'obstacle': True, 'distance_m': min(o['distance_m'] for o in objs if o['confirmed']),
            'objects': [dict(o) for o in objs]}


def test_limit_alarms_drops_far_candidate_keeps_near():
    far = {'distance_m': 197.0, 'confirmed': True, 'beyond_path': False}
    near = {'distance_m': 56.0, 'confirmed': True, 'beyond_path': False}
    res = limit_alarms(_res(far), 153.0)
    assert res['obstacle'] is False and res['distance_m'] is None
    assert res['objects'][0]['beyond_path'] and res['objects'][0]['beyond_recent_path']
    res = limit_alarms(_res(far, near), 153.0)
    assert res['obstacle'] is True and res['distance_m'] == 56.0 and res['alarm_range_m'] == 153.0


def test_limit_alarms_ignores_unconfirmed_and_none_limit():
    cand = {'distance_m': 90.0, 'confirmed': False, 'beyond_path': False}
    near = {'distance_m': 40.0, 'confirmed': True, 'beyond_path': False}
    res = limit_alarms(_res(cand, near), 50.0)
    assert res['obstacle'] and res['distance_m'] == 40.0 and 'beyond_recent_path' not in res['objects'][0]
    res = _res(near)
    assert limit_alarms(res, None) is res and res['obstacle']


def test_limit_alarms_only_near_end_of_axis():
    # один короткий пересчёт (ось 52 м) не гасит объект на 56 м далеко от конца оси (121 м)
    near = {'distance_m': 56.0, 'confirmed': True, 'beyond_path': False}
    res = limit_alarms(_res(near), 52.0, 0.8 * 121.0)
    assert res['obstacle'] and res['distance_m'] == 56.0
    # объект у конца оси (197 м при оси 153-204 м) -- без тревоги
    far = {'distance_m': 197.0, 'confirmed': True, 'beyond_path': False}
    res = limit_alarms(_res(far), 153.0, 0.8 * 204.0)
    assert not res['obstacle']
