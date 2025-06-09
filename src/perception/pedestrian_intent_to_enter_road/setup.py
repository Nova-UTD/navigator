from setuptools import find_packages, setup

package_name = 'pedestrian_intent_to_enter_road'

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
    description='',
    license='MIT License',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pedestrian_intent_to_enter_road = pedestrian_intent_to_enter_road.pedestrian_intent_to_enter_road:main'
        ],
    },
)
