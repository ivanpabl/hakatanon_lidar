"""Запуск детектора: C++-приём облака (tunnel_od_preproc) + узел tunnel_od_detector.

    ros2 launch tunnel_od_detector detector.launch.py
    ros2 launch tunnel_od_detector detector.launch.py topic:=/lidar_points input_axes:=rep103
    ros2 launch tunnel_od_detector detector.launch.py use_cpp_preproc:=false      # без C++-узла

config по умолчанию: $TUNNEL_OD_CONFIG, иначе /opt/tunnel_od/config/detector.yaml (образ Docker).
topic, result_file, stats_file, refit_every, refit_mode -- перекрывают config, если не пустые.

use_cpp_preproc:=true (по умолчанию): C++-узел tunnel_od_preproc в component container
разбирает облако любого поддерживаемого формата и публикует каноническое облако на
/tunnel_od/cloud; Python-узел берёт его без разбора. Python-узел в контейнер компонентов
rclcpp не загрузить, поэтому это два процесса, кадр между ними идёт через DDS (FastDDS SHM).
Драйвер лидара, собранный как компонент rclcpp, можно добавить в тот же контейнер
(intra_process:=true -- тогда кадр драйвер -> tunnel_od_preproc передаётся без копии).
use_cpp_preproc:=false: Python-узел подписан прямо на облако лидара и разбирает его сам
(parse_backend: auto -- нативной библиотекой tunnel_od_preproc, python -- parse_pointcloud2).
"""
import os

import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode

DEFAULT_CONFIG = os.environ.get('TUNNEL_OD_CONFIG', '/opt/tunnel_od/config/detector.yaml')
CANONICAL_TOPIC = '/tunnel_od/cloud'


def _section(cfg, name):
    if not cfg or not os.path.isfile(cfg):
        return {}
    with open(cfg, encoding='utf-8') as fh:
        data = yaml.safe_load(fh) or {}
    return dict((data.get(name) or {}).get('ros__parameters') or {})


def _flatten(d, prefix=''):
    out = {}
    for k, v in d.items():
        if isinstance(v, dict):
            out.update(_flatten(v, f'{prefix}{k}.'))
        else:
            out[f'{prefix}{k}'] = v
    return out


def _crop_check(det_params, pre, actions):
    """Обрезка по дальности безопасна, только если не задевает диапазон, который ядро
    использует при этих параметрах (util.required_fwd_range). Иначе -- выключаем."""
    if not pre.get('crop_enabled', True):
        return
    try:
        from tunnel_od_detector.util import required_fwd_range
        lo, hi = required_fwd_range({k: v for k, v in det_params.items() if k in ('near_cutoff', 'max_range')})
    except Exception as e:  # ядро не импортируется -- не режем
        pre['crop_enabled'] = False
        actions.append(LogInfo(msg=f'[tunnel_od] обрезка облака выключена: не проверить диапазон ядра ({e})'))
        return
    cmin, cmax = float(pre.get('crop_fwd_min', 2.0)), float(pre.get('crop_fwd_max', 250.0))
    if cmin > lo or cmax < hi:
        pre['crop_enabled'] = False
        actions.append(LogInfo(msg=f'[tunnel_od] обрезка облака [{cmin}, {cmax}] м выключена: ядро при этих '
                                   f'параметрах использует точки в [{lo}, {hi}] м'))


def _nodes(context):
    lc = lambda name: LaunchConfiguration(name).perform(context)
    cfg = lc('config')
    params = [cfg] if cfg and os.path.isfile(cfg) else []
    use_cpp = lc('use_cpp_preproc').lower() in ('1', 'true', 'yes', 'on')
    log_level = lc('log_level')
    actions = []

    det = {}
    for name in ('result_file', 'stats_file', 'refit_mode', 'parse_backend'):
        v = lc(name)
        if v:
            det[name] = v
    if lc('refit_every'):
        det['refit_every'] = int(lc('refit_every'))
    if lc('max_pending'):
        det['max_pending'] = int(lc('max_pending'))

    if use_cpp:
        pre = _section(cfg, 'tunnel_od_preproc')
        for name in ('input_format', 'input_axes', 'speed_topic'):
            v = lc(name)
            if v:
                pre[name] = v
        if lc('topic'):
            pre['input_topic'] = lc('topic')
        if lc('stats_file'):
            base, ext = os.path.splitext(lc('stats_file'))
            pre['stats_file'] = f'{base}_preproc{ext or ".json"}'
        _crop_check(_flatten(_section(cfg, 'tunnel_od_detector').get('detector') or {}), pre, actions)
        intra = lc('intra_process').lower() in ('1', 'true', 'yes', 'on')
        actions.append(ComposableNodeContainer(
            name='tunnel_od_container', namespace='', package='rclcpp_components',
            executable='component_container', output='screen', emulate_tty=True,
            arguments=['--ros-args', '--log-level', log_level],
            composable_node_descriptions=[ComposableNode(
                package='tunnel_od_preproc', plugin='tunnel_od::PreprocNode', name='tunnel_od_preproc',
                parameters=[pre], extra_arguments=[{'use_intra_process_comms': intra}])]))
        det.update({'topic': CANONICAL_TOPIC, 'canonical_input': True})
    else:
        if lc('topic'):
            det['topic'] = lc('topic')
        det['canonical_input'] = False
        for name in ('input_format', 'input_axes', 'speed_topic'):
            if lc(name):
                actions.append(LogInfo(msg=f'[tunnel_od] {name} действует только с use_cpp_preproc:=true'))

    actions.append(Node(
        package='tunnel_od_detector', executable='detector_node', name='tunnel_od_detector',
        output='screen', emulate_tty=True, parameters=params + [det],
        arguments=['--ros-args', '--log-level', log_level],
    ))
    return actions


def generate_launch_description():
    arg = lambda name, default, desc: DeclareLaunchArgument(name, default_value=default, description=desc)
    return LaunchDescription([
        arg('config', DEFAULT_CONFIG, 'YAML с параметрами узлов'),
        arg('topic', '', 'топик PointCloud2 лидара ("" -- автопоиск)'),
        arg('use_cpp_preproc', 'true', 'C++-приём облака (tunnel_od_preproc): true | false'),
        arg('input_format', '', 'auto | legacy_hesai | contract_v1 | generic ("" -- из config)'),
        arg('input_axes', '', 'auto | legacy | rep103 ("" -- из config)'),
        arg('speed_topic', '', 'скорость поезда (TwistStamped / Odometry), только запись в результат'),
        arg('intra_process', 'false', 'intra-process в контейнере компонентов (для драйвера-компонента)'),
        arg('parse_backend', '', 'без C++-узла: auto | native | python'),
        arg('refit_mode', '', 'async | sync ("" -- из config)'),
        arg('refit_every', '', 'пересчёт пути раз в N кадров (sync)'),
        arg('max_pending', '', 'очередь кадров детектора ("" -- из config)'),
        arg('result_file', '', 'JSONL с результатом каждого кадра'),
        arg('stats_file', '', 'итоговая статистика JSON (C++-узел -- в <имя>_preproc.json)'),
        arg('log_level', 'info', ''),
        OpaqueFunction(function=_nodes),
    ])
