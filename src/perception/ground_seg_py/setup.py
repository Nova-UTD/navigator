from setuptools import setup
from glob import glob
import os

package_name = 'ground_seg_py'

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
    maintainer_email='stepan.antonyuk@gmail.com',
    description='See package.xml',
    license='See package.xml',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ground_seg_py_node = ground_seg_py.ground_seg_py_node:main',
        ],
    },
)
