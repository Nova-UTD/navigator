from setuptools import setup

package_name = 'dynamic_grid'
setup(
	name='dynamic_grid',
	version='0.0.0',
	zip_safe=True,
	maintainer='asomasundaram',
	maintainer_email='ashwin.som2001@gmail.com',
	data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
	description='Package creates static, dynamic occupancy grid using LiDAR data',
	license='TBD',
	install_requires=['setuptools','numpy'],
	tests_require=['pytest'],
     )
