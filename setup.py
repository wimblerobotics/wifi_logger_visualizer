from setuptools import setup
from setuptools import find_packages

package_name = 'wifi_logger_visualizer'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/heat_mapper_launch.py',  # Correct launch file
        ]),
        ('share/' + package_name + '/resource', ['resource/' + package_name]),  # Include resource file
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Michael Wimble',
    maintainer_email='mike@wimblerobotics.com',
    description='WiFi Logger and Visualizer Package',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'heat_mapper_node = wifi_logger_visualizer.heat_mapper_node:main',  # Ensure consistency
            'wifi_logger_node = wifi_logger_visualizer.wifi_logger_node:main',
            'wifi_visualizer_node = wifi_logger_visualizer.wifi_visualizer_node:main',
        ],
    },
)
