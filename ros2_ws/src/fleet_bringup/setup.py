from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'fleet_bringup'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'params'),
            glob('params/*.yaml')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Eduardo',
    maintainer_email='wanderley.eduardo@gmail.com',
    description='Launch files for multi-robot fleet simulation.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [],
    },
)
