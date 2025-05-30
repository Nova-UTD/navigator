from setuptools import find_packages, setup

package_name = 'depth_processing'

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
    maintainer='Saishravan Muthukrishnan',
    maintainer_email='sxm210236@utdallas.edu',
    description='Generates depth map based on /ouster/range_image',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'depth_processing_node=depth_processing.depth_processing:main',
        ],
    },
)
