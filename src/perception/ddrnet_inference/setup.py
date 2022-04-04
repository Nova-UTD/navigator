from setuptools import setup

package_name = 'ddrnet_inference'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/weights', ['data/weights/best_val_smaller.pth']),
        ('share/weights', ['data/weights/best_val.pth']),
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
            'ddrnet_inference_node = ddrnet_inference.inference:main',
        ],
    },
)
