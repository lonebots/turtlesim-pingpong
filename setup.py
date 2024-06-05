from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'turtlesim_pingpong'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jishnu',
    maintainer_email='jishnusurajila@gmail.com',
    description='ros2 based turtlesim ping-pong game',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "ball_node = turtlesim_pingpong.ball_node:main",
            "pong_node = turtlesim_pingpong.pong_node:main", 
        ],
    },
)
