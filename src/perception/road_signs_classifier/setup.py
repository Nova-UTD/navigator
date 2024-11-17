from setuptools import find_packages, setup

package_name = 'road_signs_classifier'

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
    description='Tools to perform road sign classifications',
    license='MIT License',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'road_signs_classifier = road_signs_classifier.road_signs_classifier:main'
        ],
    },
)
