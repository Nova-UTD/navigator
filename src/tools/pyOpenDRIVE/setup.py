from setuptools import find_packages, setup, Extension
from Cython.Build import cythonize

package_name = 'pyOpenDRIVE'

extensions = [
    Extension(name="pyOpenDRIVE/*", sources=["pyOpenDRIVE/*.pyx"], include_dirs=["pyOpenDRIVE/include"], extra_compile_args=["-fpermissive"])
]

setup(
    ext_modules=cythonize(extensions),
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
    maintainer='root',
    maintainer_email='josh.pahman@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
