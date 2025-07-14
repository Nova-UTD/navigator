from setuptools import find_packages, setup

package_name = 'occupancy_grid'

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
    maintainer='Pranav Boyapati',
    maintainer_email='pkb230000@utdallas.edu',
    description='Generates occupancy grid based on depth map and segmented images',
    license='MIT License',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'occupancy_grid=occupancy_grid.occupancy_grid:main'
        ],
    },
)