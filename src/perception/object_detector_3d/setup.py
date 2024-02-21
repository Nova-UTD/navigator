from setuptools import find_packages, setup

package_name = 'object_detector_3d'

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
    maintainer='Ragib Arnab',
    maintainer_email='ragib.arnab@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lidar_object_detector_3d_node = object_detector_3d.lidar_objdet3d_node:main'
        ],
    },
)
