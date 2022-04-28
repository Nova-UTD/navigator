#!/usr/bin/env python
# -*- coding: utf-8 -*-

from setuptools import setup, find_packages

requirements = ['numpy', 'python-lzf']

setup_requirements = ['pytest-runner', ]

test_requirements = ['pytest', ]

setup(
    author="Daniel Maturana",
    author_email='dimatura@gmail.com',
    classifiers=[
        'Development Status :: 2 - Pre-Alpha',
        "Intended Audience :: Science/Research",
        'License :: OSI Approved :: BSD License',
        'Natural Language :: English',
        'Programming Language :: Python :: 2',
        'Programming Language :: Python :: 2.7',
        'Topic :: Scientific/Engineering',
        'Topic :: Scientific/Engineering :: Artificial Intelligence',
        'Topic :: Multimedia :: Graphics :: 3D Modeling',
        'Topic :: Multimedia :: Graphics :: Capture',
        'Topic :: Multimedia :: Graphics :: Graphics Conversion',
    ],
    description="Read and write PCL .pcd files in python.",
    install_requires=requirements,
    license="BSD license",
    include_package_data=True,
    keywords='pypcd',
    name='pypcd',
    packages=find_packages(include=['pypcd']),
    setup_requires=setup_requirements,
    test_suite='tests',
    tests_require=test_requirements,
    url='https://github.com/dimatura/pypcd',
    version='0.1.1',
    zip_safe=False,
)
