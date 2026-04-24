from setuptools import setup
import os
from glob import glob

package_name = 'fleet_orchestrator'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'resource/package.dsv']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'hook'),
         glob('resource/hook/*.sh') + glob('resource/hook/*.dsv')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Eduardo',
    maintainer_email='eduardo@example.com',
    description='Orquestrador de frota: record/save/play rotas e Nav2 por robot_id.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'fleet_orchestrator = fleet_orchestrator.main:main',
        ],
    },
)
