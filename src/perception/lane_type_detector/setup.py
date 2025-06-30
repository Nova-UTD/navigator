from setuptools import find_packages, setup

package_name = 'lane_type_detector'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='project.nova@utdallas.edu',
    description='Tools to detect the various road lane types',
    license='MIT License',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lane_type_detector = lane_type_detector.lane_type_detector:main'
        ],
    },
)
