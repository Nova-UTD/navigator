from setuptools import setup, find_packages

package_name = "pedestrian_crossing"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Anirudh Devatha",
    maintainer_email="axd240055@utdallas.edu",
    description="Pedestrian crossing detection and TTC-based status node.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            f"pedestrian_crossing_node = {package_name}.pedestrian_crossing_node:main",
        ],
    },
)
