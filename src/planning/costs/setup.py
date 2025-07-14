from setuptools import setup
from glob import glob
import os

package_name = 'costs'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='main',
    maintainer_email='will.heitman@utdallas.edu',
    description='Package to process, combine, and publish cost maps',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'grid_summation_node = costs.grid_summation_node:main',
            'route_costmap_node = costs.route_costmap_node:main',
            'junction_manager = costs.junction_manager:main',
        ],
    },
)
