from setuptools import setup

package_name = 'nav2_path_planner'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='See package.xml',
    maintainer_email='See package.xml',
    description='See package.xml',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'nav2_path_planner_node = nav2_path_planner.nav2_path_planner_node:main'
        ],
    },
)
