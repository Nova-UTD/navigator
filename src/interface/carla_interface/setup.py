from setuptools import setup
from glob import glob
import os

package_name = 'carla_interface'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Nova at UT Dallas',
    maintainer_email='project.nova@utdallas.edu',
    description='Simple package to hold launch scripts for the CARLA Leaderboard.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'liaison_node = carla_interface.liaison_node:main',
            'route_reader_node = carla_interface.route_reader:main'
        ],
    },
)
