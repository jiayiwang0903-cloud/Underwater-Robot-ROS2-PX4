from setuptools import setup, find_packages

package_name = 'ustrov_sim'

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
            'sim_sensors = ustrov_sim.sim_sensors:main',
            'error_analyzer = ustrov_sim.error_analyzer:main',
            'usv_simulator = ustrov_sim.usv_simulator:main',
            'visual_ekf_node = ustrov_sim.visual_ekf_node:main',
        ],
    },
)
