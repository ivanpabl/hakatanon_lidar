"""Строка статуса для записи видео: /tunnel_od/result -> три текстовых файла для ffmpeg drawtext.

<dir>/alarm.txt -- «ПРЕПЯТСТВИЕ N м» (красным), <dir>/clear.txt -- «Путь свободен» (зелёным),
<dir>/info.txt -- подпись, кадр, задержка, дальность оси, скорость (белым); <dir>/truth.txt -- истинная
дистанция до синтетического объекта из /synthetic/object, если он есть в записи. ffmpeg перечитывает файлы
на каждом кадре видео; запись через os.replace, чтобы он не прочитал файл наполовину.
"""
import json
import os

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


def _put(path, text):
    tmp = path + '.tmp'
    with open(tmp, 'w', encoding='utf-8') as f:
        f.write(text or ' ')
    os.replace(tmp, path)


class StatusOverlay(Node):
    def __init__(self):
        super().__init__('tunnel_od_status_overlay')
        self.dir = self.declare_parameter('out_dir', '/tmp/tunnel_od_overlay').value
        self.title = self.declare_parameter('title', '').value
        os.makedirs(self.dir, exist_ok=True)
        self._write(' ', 'ожидание кадров…', self.title)
        _put(os.path.join(self.dir, 'truth.txt'), ' ')
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(String, '/tunnel_od/result', self._on_result, qos)
        self.create_subscription(String, '/synthetic/object', self._on_truth, qos)

    def _on_truth(self, msg):
        t = json.loads(msg.data)
        _put(os.path.join(self.dir, 'truth.txt'),
             f"синтетика: {t.get('shape', 'объект')} на {t['distance_m']:.1f} м")

    def _write(self, alarm, clear, info):
        _put(os.path.join(self.dir, 'alarm.txt'), alarm)
        _put(os.path.join(self.dir, 'clear.txt'), clear)
        _put(os.path.join(self.dir, 'info.txt'), info)

    def _on_result(self, msg):
        r = json.loads(msg.data)
        info = [f"кадр {r.get('frame', 0)}"]
        if r.get('latency_ms') is not None:
            info.append(f"задержка {r['latency_ms']:.0f} мс")
        if r.get('path_range_m'):
            info.append(f"ось пути до {r['path_range_m']:.0f} м")
        if r.get('speed_mps') is not None:
            info.append(f"скорость {r['speed_mps']:.1f} м/с")
        info = '   '.join(([self.title] if self.title else []) + info)
        if r.get('obstacle') and r.get('distance_m') is not None:
            self._write(f"ПРЕПЯТСТВИЕ  {r['distance_m']:.1f} м", ' ', info)
        else:
            self._write(' ', 'Путь свободен', info)


def main():
    rclpy.init()
    node = StatusOverlay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
