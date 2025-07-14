from setuptools import setup

package_name = 'yolov8_brake_detection'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    install_requires=[
        'setuptools', 
        'torch', 
        'opencv-python', 
        'cv-bridge', 
        'rclpy', 
        'sensor_msgs',  # Dependencies for brake light detection node
    ],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    entry_points={
        'console_scripts': [
            'yolov8_node = yolov8_brake_detection.yolov8_node:main',
        ],
    },
)
