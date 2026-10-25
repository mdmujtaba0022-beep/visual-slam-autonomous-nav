from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'autonomous_nav'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml') + glob('config/*.rviz')),
        (os.path.join('share', package_name, 'worlds'),
            glob('worlds/*.world') + glob('worlds/*.sdf')),
        (os.path.join('share', package_name, 'maps'),
            glob('maps/*.yaml') + glob('maps/*.pgm')),
    ],
    install_requires=['setuptools'],
    tests_require=['pytest'],
    zip_safe=True,
    maintainer='nick',
    maintainer_email='nick@todo.com',
    description='Visual SLAM and autonomous navigation for TurtleBot3',
    license='MIT',
    entry_points={
        'console_scripts': [
            'frontier_explorer = autonomous_nav.frontier_explorer:main',
            'waypoint_manager = autonomous_nav.waypoint_manager:main',
            'orb_slam_bridge = autonomous_nav.orb_slam_bridge:main',
            'exploration_metrics = autonomous_nav.metrics:main',
        ],
    },
)
