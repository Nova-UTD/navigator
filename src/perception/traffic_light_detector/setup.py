from setuptools import find_packages, setup

package_name = 'traffic_light_detector'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Project Nova',
    maintainer_email='project.nova@utdallas.edu',
    description='Classifies traffic lights seen in camera images',
    license='MIT License',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'traffic_light_node = traffic_light_detector.traffic_light_node:main'
        ],
    },
)
