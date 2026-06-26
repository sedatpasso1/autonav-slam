from setuptools import setup, find_packages
from glob import glob

setup(
    name="autonav_slam",
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/autonav_slam"]),
        ("share/autonav_slam", ["package.xml"]),
        ("share/autonav_slam/launch", glob("launch/*.py")),
        ("share/autonav_slam/config", glob("config/*.yaml")),
    ],
    install_requires=[
        "setuptools",
        "kiss-icp>=1.0.0",
        "numpy>=1.21",
    ],
    entry_points={
        "console_scripts": [
            "slam_node       = autonav_slam.slam_node:main",
            "fast_lio2_node  = autonav_slam.fast_lio2_node:main",
        ],
    },
)
