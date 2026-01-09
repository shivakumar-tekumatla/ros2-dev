from setuptools import setup
import os
from glob import glob

package_name = 'ros2_tutorials'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    description='Simple ROS 2 tutorials with publisher and subscriber nodes',
    entry_points={
        'console_scripts': [
            'publisher_node = ros2_tutorials.publisher_node:main',
            'subscriber_node = ros2_tutorials.subscriber_node:main',
        ],
    },
)

