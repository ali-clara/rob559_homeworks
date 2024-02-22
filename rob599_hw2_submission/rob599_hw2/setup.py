from setuptools import find_packages, setup

from glob import glob
import os

package_name = 'rob599_hw2'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='aliclara',
    maintainer_email='jonesal9@oregonstate.edu',
    description='Package for rob599 homework 2. Intro to ROS2 nodes, services, and actions',
    license='GPL-3.0-only',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'twist_publisher = rob599_hw2.twist_publisher:main',
            'speed_limiter = rob599_hw2.speed_limiter:main',
            'speed_checker = rob599_hw2.speed_checker:main',
            'braking_service_client = rob599_hw2.braking_service_client:main',
            'nasa_action_server = rob599_hw2.nasa_action_server:main',
            'nasa_action_client = rob599_hw2.nasa_action_client:main',
        ],
    },
)

