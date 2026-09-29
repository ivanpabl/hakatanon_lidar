# © 2026 команда «Голуби», github.com/ivanpabl/hakatanon_lidar. Все права защищены, условия — в файле LICENSE. GLB-K5-4660c47a8c6c
from glob import glob

from setuptools import setup

package_name = 'tunnel_od_detector'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tunnel_od',
    maintainer_email='noreply@example.com',
    description='ROS 2-узел обнаружения препятствий в тоннеле метро (обёртка над tunnel_od)',
    license='Proprietary',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'detector_node = tunnel_od_detector.node:main',
        ],
    },
)
