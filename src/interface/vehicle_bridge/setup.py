from setuptools import setup
from glob import glob
import os

package_name = 'vt_vehicle_bridge'

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
    maintainer='Will Heitman',
    maintainer_email='will.heitman@utdallas.edu',
    description='This node subscribes to vehicle state data and commands, reformats them, then republishes it.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'svl_bridge_exe = vt_vehicle_bridge.svl_bridge:main',
        ],
    },
)
