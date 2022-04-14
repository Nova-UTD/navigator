from setuptools import setup

package_name = 'darknet_inference'
submodules = 'darknet_inference/tool'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, submodules],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/names', ['data/names/coco.names']),
        ('share/names', ['data/names/voc.names']),
        ('share/weights', ['data/weights/yolov4.weights']),
        ('share/weights', ['data/weights/yolov4-tiny.weights']),
        ('share/cfg', ['data/cfg/yolov4.cfg']),
        ('share/cfg', ['data/cfg/yolov4-tiny.cfg']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rarnab',
    maintainer_email='rarnab@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'darknet_inference_node = darknet_inference.inference:main',
        ],
    },
)
