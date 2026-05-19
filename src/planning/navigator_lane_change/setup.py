from setuptools import setup, find_packages
import os
from glob import glob

package_name = "navigator_lane_change"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", glob("launch/*.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Nova UTD",
    maintainer_email="team@nova-utd.org",
    description="Lane-change behavior module for Navigator (shadow mode)",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            f"lane_change_node = {package_name}.lane_change_node:main",
        ],
    },
)
