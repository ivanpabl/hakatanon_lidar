# © 2026 команда «Голуби», github.com/ivanpabl/hakatanon_lidar. Все права защищены, условия — в файле LICENSE. GLB-K5-4660c47a8c6c
"""Показ в браузере: запись по кругу + детектор + RViz2 на виртуальном экране, экран -- через noVNC.

    ros2 launch tunnel_od_detector demo.launch.py bag:=/bag     # затем http://localhost:6080

Xvfb (виртуальный экран :99) -> x11vnc (VNC этого экрана) -> websockify + noVNC (VNC в браузере)
-> play.launch.py с rviz:=true, loop:=true. Нужен образ стадии viz (docker compose up demo).
"""
from pathlib import Path

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, LogInfo,
                            SetEnvironmentVariable, TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

HERE = Path(__file__).resolve().parent
DISPLAY = ':99'


def generate_launch_description():
    port = LaunchConfiguration('port')
    return LaunchDescription([
        DeclareLaunchArgument('bag', description='каталог записи ros2 bag'),
        DeclareLaunchArgument('rate', default_value='1.0'),
        DeclareLaunchArgument('port', default_value='6080', description='порт noVNC'),
        DeclareLaunchArgument('size', default_value='1600x900x24', description='размер экрана'),
        SetEnvironmentVariable('DISPLAY', DISPLAY),
        SetEnvironmentVariable('LIBGL_ALWAYS_SOFTWARE', '1'),
        ExecuteProcess(cmd=['Xvfb', DISPLAY, '-screen', '0', LaunchConfiguration('size'), '-nolisten', 'tcp'],
                       output='log', name='xvfb'),
        TimerAction(period=1.0, actions=[
            ExecuteProcess(cmd=['x11vnc', '-display', DISPLAY, '-forever', '-shared', '-nopw', '-quiet',
                                '-localhost', '-rfbport', '5900'], output='log', name='x11vnc'),
            ExecuteProcess(cmd=['websockify', '--web', '/usr/share/novnc', port, 'localhost:5900'],
                           output='log', name='novnc'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(HERE / 'play.launch.py')),
                launch_arguments={'bag': LaunchConfiguration('bag'), 'rate': LaunchConfiguration('rate'),
                                  'rviz': 'true', 'loop': 'true', 'probe': 'false'}.items()),
            LogInfo(msg=['[tunnel_od] показ: http://localhost:', port, '/vnc.html?autoconnect=1&resize=scale']),
        ]),
    ])
