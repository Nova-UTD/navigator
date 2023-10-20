from setuptools import setup
from glob import glob
import os

package_name = 'helpers'

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
    description='See package.xml',
    license='See package.xml',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'routing_listener = helpers.routing_listener:main',
            'routing_monitor = helpers.routing_monitor:main'
        ],
    },
)
