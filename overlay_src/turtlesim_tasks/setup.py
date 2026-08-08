from setuptools import setup
package_name = 'turtlesim_tasks'
setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/sim_bringup.launch.py',
            'launch/follower_bringup.launch.py'
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='You',
    maintainer_email='you@example.com',
    description='Turtlesim task nodes',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'mouse_goal_publisher = turtlesim_tasks.mouse_goal_publisher:main',
            'turtle1_controller   = turtlesim_tasks.turtle1_controller:main',
            'spawn_second         = turtlesim_tasks.spawn_second:main',
            'follower_node        = turtlesim_tasks.follower_node:main',
            'mode_toggle          = turtlesim_tasks.mode_toggle:main',
        ],
    },
)
