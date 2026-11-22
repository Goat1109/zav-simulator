from setuptools import setup
import os
from glob import glob

package_name = 'argo_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    package_data={package_name: ['cyclonedds_flight_manager.xml']},
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='zenith',
    maintainer_email='zenith@todo.todo',
    description='ARGO Drone Flight Manager, Keyboard Teleoperation and Offboard Bridge Package',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'teleop_keyboard = argo_control.teleop_keyboard:main',
            'flight_manager = argo_control.argo_flight_manager:main',
            'lio_px4_bridge = argo_control.lio_px4_bridge:main',
        ],
    },
)
