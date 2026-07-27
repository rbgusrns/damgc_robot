from glob import glob

from setuptools import find_packages, setup


package_name = "follower_control"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="kde",
    maintainer_email="kde@example.com",
    description="Follower velocity command safety guard with a local watchdog.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "velocity_guard_node = follower_control.velocity_guard_node:main",
        ],
    },
)
