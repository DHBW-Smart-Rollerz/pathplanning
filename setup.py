import glob
import os

from setuptools import find_packages, setup

package_name = "pathplanning"

setup(
    name=package_name,
    version="1.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            os.path.join("share", package_name, "launch"),
            glob.glob(os.path.join("launch", "*launch.py")),
        ),
        (
            os.path.join("share", package_name, "config"),
            glob.glob(os.path.join("config", "*.*")),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Smart Rollerz",
    maintainer_email="info@dhbw-smartrollerz.org",
    description="Pathplanning for the smarty pipeline",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            f"pathplanning_node = {package_name}.pathplanning_node:main",
        ],
    },
)
