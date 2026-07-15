from glob import glob
import os

from setuptools import find_packages, setup


package_name = 'turtlebot_fleet_sim'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'config', 'maps', 'turtlebot3_world_v1'),
         glob('config/maps/turtlebot3_world_v1/*')),
        (os.path.join('share', package_name, 'config', 'maps', 'asymmetric_indoor_v1'),
         glob('config/maps/asymmetric_indoor_v1/*')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.sdf')),
    ],
    install_requires=['setuptools', 'PyYAML'],
    tests_require=['pytest'],
    zip_safe=True,
    maintainer='CyberParvez',
    maintainer_email='noreply@example.com',
    description='Strict YAML-driven multi-TurtleBot3 Gazebo Harmonic simulation.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'fleet_readiness = turtlebot_fleet_sim.fleet_readiness:main',
            'sensor_contract = turtlebot_fleet_sim.sensor_contract:main',
            'teleop = turtlebot_fleet_sim.teleop:main',
        ],
    },
)
