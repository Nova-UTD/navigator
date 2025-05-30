from setuptools import find_packages, setup

package_name = 'image_segmentation'

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
    description='Generates segmented image from rosbag image data',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'image_seg_node = image_segmentation.sam2.sam2.image_segmentation:main'
        ],
    },
)
