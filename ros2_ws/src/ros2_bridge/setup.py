from setuptools import setup
import os
from glob import glob

package_name = 'ros2_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        ('lib/' + package_name, ['scripts/human_bridge_node']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='basti',
    maintainer_email='bastien.caspani@gmail.com',
    description='Bridge between external UDP vision app and ROS2',
    license='MIT',
    tests_require=['pytest'],
    scripts=['scripts/human_bridge_node'],
)
