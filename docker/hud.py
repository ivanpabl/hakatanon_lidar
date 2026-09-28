"""Строка статуса для видео: подписка на /tunnel_od/result, текст в файлы для ffmpeg drawtext.

    python3 hud.py /tmp/hud        # -> /tmp/hud_alarm.txt, /tmp/hud_clear.txt, /tmp/hud_info.txt

Три файла -- три цвета надписи: тревога (красный), путь свободен (зелёный), служебная строка (белый).
Запись через os.replace: ffmpeg перечитывает файл на каждом кадре и не видит его наполовину записанным.
"""
import json
import os
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

TITLE = os.environ.get('TITLE', '')


def put(path, text):
    tmp = path + '.tmp'
    with open(tmp, 'w', encoding='utf-8') as f:
        f.write(text or ' ')
    os.replace(tmp, path)


class Hud(Node):
    def __init__(self, base):
        super().__init__('tunnel_od_hud')
        self.base = base
        self.create_subscription(String, '/tunnel_od/result', self.on_result,
                                 QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))

    def on_result(self, msg):
        r = json.loads(msg.data)
        if r.get('obstacle') and r.get('distance_m') is not None:
            put(self.base + '_alarm.txt', f"ПРЕПЯТСТВИЕ  {r['distance_m']:.1f} м")
            put(self.base + '_clear.txt', ' ')
        else:
            put(self.base + '_alarm.txt', ' ')
            put(self.base + '_clear.txt', 'Путь свободен')
        info = [f"кадр {r.get('frame', 0)}"]
        if r.get('latency_ms') is not None:
            info.append(f"задержка {r['latency_ms']:.0f} мс")
        if r.get('path_range_m'):
            info.append(f"ось пути до {r['path_range_m']:.0f} м")
        if r.get('speed_mps') is not None:
            info.append(f"скорость {r['speed_mps']:.1f} м/с")
        put(self.base + '_info.txt', (TITLE + '   ' if TITLE else '') + '   '.join(info))


def main():
    base = sys.argv[1] if len(sys.argv) > 1 else '/tmp/hud'
    put(base + '_alarm.txt', ' ')
    put(base + '_clear.txt', 'ожидание кадров…')
    put(base + '_info.txt', TITLE or ' ')
    rclpy.init()
    node = Hud(base)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
