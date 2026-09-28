"""Видео прогона: запись + детектор + RViz на виртуальном экране, экран пишется в mp4.

    ros2 launch tunnel_od_detector record.launch.py bag:=/bag
    ros2 launch tunnel_od_detector record.launch.py bag:=/bag start_offset:=67 duration:=8 title:="низкая балка"

Xvfb (экран :99) -> play.launch.py с RViz (config/record.rviz: без панелей, вид из-за кабины)
-> ffmpeg пишет экран в <out>/<запись>[_<с какой секунды>].mp4, поверх -- строка статуса из
/tunnel_od/result (status_overlay): решение, дистанция, задержка, дальность оси, скорость.
Меню и строка состояния RViz обрезаются. ffmpeg пишет ровно столько, сколько идёт запись (или
duration секунд), дописывает файл и завершается -- после этого останавливается всё. Нужен образ viz.
"""
import os
import signal

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, ExecuteProcess, IncludeLaunchDescription, LogInfo,
                            OpaqueFunction, RegisterEventHandler, SetEnvironmentVariable, TimerAction)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown, matches_action
from launch.events.process import SignalProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from tunnel_od_detector.bag import bag_info

HERE = os.path.dirname(os.path.abspath(__file__))
DISPLAY = ':99'
W, H = 1600, 900
CROP = '1550:790:25:70'
FONT = '/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf'
OVERLAY = '/tmp/tunnel_od_overlay'
START_DELAY = 10.0


def _actions(context):
    lc = lambda name: LaunchConfiguration(name).perform(context)
    bag, out, start = lc('bag'), lc('out'), float(lc('start_offset'))
    rate, length = float(lc('rate')), float(lc('duration'))
    name, _, frame, bag_len = bag_info(bag)
    length = length if length > 0 else max(bag_len - start, 0.0)
    os.makedirs(out, exist_ok=True)
    video = os.path.join(out, f'{name}{f"_{start:.0f}" if start > 0 else ""}.mp4')
    txt = f'fontfile={FONT}:reload=1:box=1:boxcolor=black@0.55:boxborderw=12'
    vf = (f'crop={CROP},'
          f'drawtext={txt}:textfile={OVERLAY}/alarm.txt:fontcolor=0xff4040:fontsize=44:x=24:y=24,'
          f'drawtext={txt}:textfile={OVERLAY}/clear.txt:fontcolor=0x50ff70:fontsize=44:x=24:y=24,'
          f'drawtext={txt}:textfile={OVERLAY}/info.txt:fontcolor=white:fontsize=22:x=24:y=h-56,'
          f'drawtext={txt}:textfile={OVERLAY}/truth.txt:fontcolor=0xffd060:fontsize=28:x=w-tw-24:y=24')
    ffmpeg = ExecuteProcess(cmd=['ffmpeg', '-loglevel', 'error', '-y', '-f', 'x11grab', '-draw_mouse', '0',
                                 '-video_size', f'{W}x{H}', '-framerate', lc('fps'), '-i', DISPLAY,
                                 '-t', f'{length / rate + 2.0:.1f}', '-vf', vf,
                                 '-c:v', 'libx264', '-preset', 'veryfast', '-crf', '23', '-pix_fmt', 'yuv420p',
                                 video], output='log', name='ffmpeg')
    rviz = Node(package='rviz2', executable='rviz2', output='log', arguments=['-d', lc('rviz_config'), '-f', frame])
    stop = [LogInfo(msg=f'[tunnel_od] видео записано: {video}'),
            EmitEvent(event=SignalProcess(signal_number=signal.SIGINT, process_matcher=matches_action(rviz))),
            TimerAction(period=2.0, actions=[EmitEvent(event=Shutdown(reason='видео записано'))])]
    return [
        rviz,
        Node(package='tunnel_od_detector', executable='status_overlay', output='log',
             parameters=[{'out_dir': OVERLAY, 'title': lc('title') or name}]),
        TimerAction(period=1.0, actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(HERE, 'play.launch.py')),
            launch_arguments={'bag': bag, 'out': out, 'rate': lc('rate'), 'rviz': 'false', 'probe': 'false',
                              'start_offset': lc('start_offset'),
                              'duration': lc('duration'), 'start_delay': str(START_DELAY),
                              'stop_at_end': 'false'}.items())]),
        TimerAction(period=START_DELAY, actions=[ffmpeg, LogInfo(msg=f'[tunnel_od] видео: {video}')]),
        RegisterEventHandler(OnProcessExit(target_action=ffmpeg, on_exit=stop)),
    ]


def generate_launch_description():
    arg = lambda name, default, desc: DeclareLaunchArgument(name, default_value=default, description=desc)
    return LaunchDescription([
        DeclareLaunchArgument('bag', description='каталог записи ros2 bag'),
        arg('out', '/out/video', 'куда писать mp4 и результат прогона'),
        arg('rate', '1.0', 'скорость проигрывания'),
        arg('start_offset', '0', 'с какой секунды записи начать'),
        arg('duration', '0', 'сколько секунд записи снять (0 -- до конца)'),
        arg('title', '', 'подпись в строке статуса (по умолчанию -- имя записи)'),
        arg('fps', '15', 'кадров видео в секунду'),
        arg('rviz_config', '/opt/tunnel_od/config/record.rviz', 'вид RViz для видео'),
        SetEnvironmentVariable('DISPLAY', DISPLAY),
        SetEnvironmentVariable('LIBGL_ALWAYS_SOFTWARE', '1'),
        ExecuteProcess(cmd=['Xvfb', DISPLAY, '-screen', '0', f'{W}x{H}x24', '-nolisten', 'tcp'],
                       output='log', name='xvfb'),
        OpaqueFunction(function=_actions),
    ])
