from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'mavspy_ros2'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='cgoodin@cavs.msstate.edu',
    description='ROS2 node wrapping the MAVS simulator for closed-loop simulation with the NATURE stack',
    license='MIT',
    entry_points={
        'console_scripts': [
            'mavs_sim_node = mavspy_ros2.mavs_sim_node:main',
        ],
    },
)
