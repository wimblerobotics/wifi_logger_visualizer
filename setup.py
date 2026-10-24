from setuptools import setup
from setuptools import find_packages
from glob import glob

package_name = 'wifi_logger_visualizer'

setup(
    name=package_name,
    version='2.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
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
            'wifi_logger_node = wifi_logger_visualizer.wifi_logger_node:main',
            'wifi_visualizer_node = wifi_logger_visualizer.wifi_visualizer_node:main',
            'wifi_survey_node = wifi_logger_visualizer.survey_node:main',
            'wifi_web_viewer = wifi_logger_visualizer.web_viewer:main',
        ],
    },
)
