import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'birobot_moveit_config'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Birobot Dev',
    maintainer_email='dev@birobot.org',
    description='MoveIt 2 configuration package for Birobot dual-arm workcell',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'motion_planning_monitor = birobot_moveit_config.motion_planning_monitor:main',
        ],
    },
)
