"""Запуск узла tunnel_od_detector.

    ros2 launch tunnel_od_detector detector.launch.py
    ros2 launch tunnel_od_detector detector.launch.py config:=/path/detector.yaml topic:=/lidar_points

config по умолчанию: $TUNNEL_OD_CONFIG, иначе /opt/tunnel_od/config/detector.yaml (образ Docker).
topic, result_file, stats_file -- перекрывают значения из config, если не пустые.
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

DEFAULT_CONFIG = os.environ.get('TUNNEL_OD_CONFIG', '/opt/tunnel_od/config/detector.yaml')


def _node(context):
    cfg = LaunchConfiguration('config').perform(context)
    params = [cfg] if cfg and os.path.isfile(cfg) else []
    overrides = {}
    for name in ('topic', 'result_file', 'stats_file'):
        v = LaunchConfiguration(name).perform(context)
        if v:
            overrides[name] = v
    refit = LaunchConfiguration('refit_every').perform(context)
    if refit:
        overrides['refit_every'] = int(refit)
    if overrides:
        params.append(overrides)
    return [Node(
        package='tunnel_od_detector', executable='detector_node', name='tunnel_od_detector',
        output='screen', emulate_tty=True, parameters=params,
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level').perform(context)],
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=DEFAULT_CONFIG, description='YAML с параметрами узла'),
        DeclareLaunchArgument('topic', default_value='', description='топик PointCloud2 ("" -- автопоиск)'),
        DeclareLaunchArgument('refit_every', default_value='', description='пересчёт пути раз в N кадров'),
        DeclareLaunchArgument('result_file', default_value='', description='JSONL с результатом каждого кадра'),
        DeclareLaunchArgument('stats_file', default_value='', description='итоговая статистика JSON'),
        DeclareLaunchArgument('log_level', default_value='info'),
        OpaqueFunction(function=_node),
    ])
