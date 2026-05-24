import os
from glob import glob

from setuptools import setup

package_name = 'go2_behaviours'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    description='Go2 robot behaviour nodes for PAR project',
    license='MIT',
    entry_points={
        'console_scripts': [
            'sport_client_wrapper_node = go2_behaviours.sport_client_wrapper_node:main',
            'safety_monitor_node = go2_behaviours.safety_monitor_node:main',
            'behaviour_executor_node = go2_behaviours.behaviour_executor_node:main',
            'behaviour_planner_node = go2_behaviours.behaviour_planner_node:main',
            'sit_stand_loop_node = go2_behaviours.sit_stand_loop_node:main',
        ],
    },
)
