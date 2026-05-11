from setuptools import setup, find_packages

package_name = 'ustrov_sensor_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            'imu_bridge = ustrov_sensor_bridge.imu_bridge:main',
            'dvl_bridge = ustrov_sensor_bridge.dvl_bridge:main',
            'depth_bridge = ustrov_sensor_bridge.depth_bridge:main',
        ],
    },
)
