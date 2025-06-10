from setuptools import setup
from glob import glob
import os

package_name = 'mall_robot_test'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='xavierd',
    maintainer_email='xavierd@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gamepad_controller_node = mall_robot_test.gamepad_controller_node:main',
            'cmd_vel_to_can_node = mall_robot_test.cmd_vel_to_can_node:main',
            'imu_node = mall_robot_test.imu_node:main',
            'ultrasonic_sensors_node = mall_robot_test.ultrasonic_sensors_node:main',
            'battery_control_node = mall_robot_test.battery_control_node:main',
            'can_motor_status_node = mall_robot_test.can_motor_status_node:main',
        ],
    },
)
