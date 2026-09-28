"""Прогон записи: узел детектора + ros2 bag play в одном запуске, итог в out.

    ros2 launch tunnel_od_detector play.launch.py bag:=/bag
    ros2 launch tunnel_od_detector play.launch.py bag:=/bag rate:=0.5 rviz:=true

Порядок:
1. detector.launch.py (C++-приём облака + узел детектора), результат каждого кадра -- в
   <out>/<запись>_result.jsonl, итог -- в <out>/<запись>_stats.json.
2. Через start_delay/2 с -- ros2 bag play на паузе (--start-paused): плеер заполняет очередь чтения,
   а DDS за это время связывает плеер с узлом (QoS VOLATILE: кадры до связи теряются).
3. Через start_delay с -- снятие с паузы (/rosbag2_player/resume). Без паузы плеер Humble отдаёт
   первые 1-2 с записи пачкой, и узел их отбрасывает.
4. Плеер закончил -- через 3 с (дообработать последние кадры) запуск останавливается; узел при
   остановке пишет итог. Страховка: длительность записи / rate + 60 с.
--read-ahead-queue-size 20: по умолчанию плеер Humble читает вперёд 1000 сообщений (вся запись,
3-5 ГБ в памяти, ~20 с тишины на старте).
"""
import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, ExecuteProcess, IncludeLaunchDescription,
                            LogInfo, OpaqueFunction, RegisterEventHandler, TimerAction)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from tunnel_od_detector.bag import bag_info

HERE = Path(__file__).resolve().parent
RVIZ_CONFIG = os.environ.get('TUNNEL_OD_RVIZ', '/opt/tunnel_od/config/tunnel_od.rviz')


def _on(value):
    return str(value).lower() in ('1', 'true', 'yes', 'on')


def _actions(context):
    lc = lambda name: LaunchConfiguration(name).perform(context)
    bag, out = lc('bag'), lc('out')
    rate, delay, loop = float(lc('rate')), float(lc('start_delay')), _on(lc('loop'))
    start, length = float(lc('start_offset')), float(lc('duration'))
    name, topic, frame, duration = bag_info(bag)
    os.makedirs(out, exist_ok=True)
    actions = [LogInfo(msg=f'[tunnel_od] запись {name}: {topic}, frame_id {frame}, {duration:.0f} с, rate {rate}')]

    actions.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(HERE / 'detector.launch.py')),
        launch_arguments={'topic': topic,
                          'result_file': f'{out}/{name}_result.jsonl',
                          'stats_file': f'{out}/{name}_stats.json'}.items()))
    if _on(lc('probe')):
        actions.append(Node(package='tunnel_od_preproc', executable='latency_probe', output='log',
                            parameters=[{'input_topic': topic, 'out_file': f'{out}/{name}_e2e.json'}]))
    if _on(lc('rviz')):
        actions.append(Node(package='rviz2', executable='rviz2', output='log',
                            arguments=['-d', lc('rviz_config') or RVIZ_CONFIG, '-f', frame]))

    play_cmd = ['ros2', 'bag', 'play', bag, '--rate', str(rate), '--read-ahead-queue-size', '20',
                '--disable-keyboard-controls', '--start-paused'] + (['--loop'] if loop else [])
    if start > 0:
        play_cmd += ['--start-offset', str(start)]
    player = ExecuteProcess(cmd=play_cmd, output='screen', name='player')
    resume = ExecuteProcess(cmd=['ros2', 'service', 'call', '/rosbag2_player/resume',
                                 'rosbag2_interfaces/srv/Resume'], output='log', name='resume')
    actions += [
        TimerAction(period=delay / 2, actions=[player]),
        TimerAction(period=delay, actions=[resume, LogInfo(msg='[tunnel_od] воспроизведение запущено')]),
    ]
    if not loop and _on(lc('stop_at_end')):
        stop = EmitEvent(event=Shutdown(reason='запись закончилась'))
        actions += [
            RegisterEventHandler(OnProcessExit(target_action=player, on_exit=[
                LogInfo(msg=f'[tunnel_od] запись закончилась; итог: {out}/{name}_stats.json'),
                TimerAction(period=3.0, actions=[stop])])),
            *([TimerAction(period=delay + length / rate, actions=[
                LogInfo(msg=f'[tunnel_od] {length:.0f} с записи проиграны'), stop])] if length > 0 else []),
            TimerAction(period=delay + duration / rate + 60, actions=[
                LogInfo(msg='[tunnel_od] плеер не закончил вовремя, остановка'), stop]),
        ]
    return actions


def generate_launch_description():
    arg = lambda name, default, desc: DeclareLaunchArgument(name, default_value=default, description=desc)
    return LaunchDescription([
        DeclareLaunchArgument('bag', description='каталог записи ros2 bag (metadata.yaml + .db3)'),
        arg('rate', '1.0', 'скорость проигрывания'),
        arg('out', '/out', 'куда писать результат и итог'),
        arg('start_delay', '6.0', 'через сколько секунд снять плеер с паузы'),
        arg('probe', 'true', 'замер e2e-задержки (latency_probe) -> <out>/<запись>_e2e.json'),
        arg('rviz', 'false', 'открыть RViz2 (нужен образ viz и экран)'),
        arg('loop', 'false', 'проигрывать запись по кругу (для показа), без остановки'),
        arg('start_offset', '0', 'с какой секунды записи начать'),
        arg('duration', '0', 'сколько секунд записи проиграть (0 -- до конца)'),
        arg('rviz_config', '', 'конфиг RViz (по умолчанию config/tunnel_od.rviz)'),
        arg('stop_at_end', 'true', 'остановить запуск, когда запись проиграна (record останавливает сам)'),
        OpaqueFunction(function=_actions),
    ])
