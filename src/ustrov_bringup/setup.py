import os
from glob import glob
from setuptools import setup, find_packages

package_name = 'ustrov_bringup'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            'sim_main = ustrov_bringup.sim_main:main',
            'px4_main = ustrov_bringup.px4_main:main',
            'publish_target_pose = ustrov_bringup.publish_target_pose:main',
        ],
    },
)
