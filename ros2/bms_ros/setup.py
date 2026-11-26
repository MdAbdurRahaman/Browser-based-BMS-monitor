from glob import glob
from setuptools import setup

package_name = 'bms_ros'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name, ['.env.example']),
    ],
    install_requires=['setuptools', 'pyserial'],
    zip_safe=True,
    maintainer='Md Abdur Rahaman',
    maintainer_email='abdur.dubotech@gmail.com',
    description='ROS 2 driver for JBD and Daly smart BMS over USB serial',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'bms_node = bms_ros.bms_node:main',
            'bms_probe = bms_ros.bms_node:probe_main',
        ],
    },
)
