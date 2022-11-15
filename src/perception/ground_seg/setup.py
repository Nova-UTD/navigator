from setuptools import setup

package_name = 'ground_seg'

setup(
    name=package_name, 
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resources_index/packages', ['resource/' + package_name]),
        ('share/' + package_naem, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='santonyuk',
    maintainer_email='stepan.antonyuk@gmil.com',
    description='Deletes all the ground points',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': ['ground_seg = ground_seg.ground_seg:main'],
    },
)
