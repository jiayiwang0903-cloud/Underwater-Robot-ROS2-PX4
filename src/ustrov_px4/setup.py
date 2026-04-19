from setuptools import setup, find_packages

package_name = 'ustrov_px4'

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
            'manual_control = ustrov_px4.manual_control:main',
            'monitor_px4 = ustrov_px4.monitor_px4:main',
        ],
    },
)
