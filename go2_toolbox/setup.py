import os
from glob import glob

from setuptools import setup

package_name = 'go2_toolbox'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Mattias Tofte',
    maintainer_email='mattiastofte@gmail.com',
    description='Go2 robot toolbox for SLAM, odometry, and utility nodes.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'odom_to_tf_bridge = go2_toolbox.odom_to_tf_bridge:main',
        ],
    },
)