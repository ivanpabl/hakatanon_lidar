"""Ядро обнаружения препятствий в тоннеле метро по облаку 3D-лидара. Без ROS 2.

    from tunnel_od import ObstacleDetector, parse_pointcloud2
"""
from .detection.detector import ObstacleDetector
from .detection.zone import GAUGE_METRO, RECT_DEFAULT
from .geometry.path import TrackPath
from .pointcloud import parse_pointcloud2

__all__ = ['ObstacleDetector', 'parse_pointcloud2', 'TrackPath', 'GAUGE_METRO', 'RECT_DEFAULT']
