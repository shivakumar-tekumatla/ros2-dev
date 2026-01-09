from setuptools import setup

package_name = 'ros2_tutorials'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    entry_points={
        'console_scripts': [
            'publisher_node = ros2_tutorials.publisher_node:main',
            'subscriber_node = ros2_tutorials.subscriber_node:main',
        ],
    },
)


# entry_points={
#     'console_scripts': [
#         'publisher_node = ros2_tutorials.publisher_node:main',
#     ],
# },

