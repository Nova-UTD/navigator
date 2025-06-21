from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'lidar_SLAM'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'resource'), glob('resource/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Aditya Pulipaka',
    maintainer_email='adipu@utexas.edu',
    description='Lidar-Based SLAM package using a pcd map given in the resource directory',
    license='MIT License',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
          'localization_node = lidar_SLAM.localization_node:main',
          'mapping_node = lidar_SLAM.mapping_node:main'
        ],
    },
)
