from glob import glob
from setuptools import find_packages, setup

setup(
    name='fleet_localization', version='0.1.0', packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/fleet_localization']),
        ('share/fleet_localization', ['package.xml', 'README.md']),
        ('share/fleet_localization/launch', glob('launch/*.launch.py')),
        ('share/fleet_localization/config', glob('config/*.yaml')),
        ('share/fleet_localization/rviz', glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools', 'PyYAML'], tests_require=['pytest'], zip_safe=True,
    maintainer='Parvez', maintainer_email='parvez@example.com', license='Apache-2.0',
    entry_points={'console_scripts': [
        'readiness = fleet_localization.readiness:main',
        'health = fleet_localization.health:main',
        'map_save = fleet_localization.map_save:main',
    ]},
)
