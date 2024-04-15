from setuptools import find_packages, setup

package_name = 'lidar_detection'

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
    maintainer=['Gueren Sanford', 'Ragib Arnab'],
    maintainer_email=['guerensanford@gmail.com', 'ragib.arnab@gmail.com'],
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lidar_detection_node = lidar_detection.lidar_detection_node:main',
        ],
    },
)
