from setuptools import setup
import os
from glob import glob

package_name = 'autonomous_cruise'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'params'), glob('params/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Siddarth Nandyala',
    maintainer_email='siddarth.nandyala@utdallas.edu',
    author='Siddarth Nandyala',
    author_email='siddarth.nandyala@utdallas.edu',
    description='End-to-end autonomous cruise controller with lateral control, longitudinal control, and intersection handling for GEM e6 vehicle',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'autonomous_cruise_node = autonomous_cruise.autonomous_cruise_node:main',
            'autonomous_cruise_intersection = autonomous_cruise.autonomous_cruise_intersection_node:main',
        ],
    },
)
