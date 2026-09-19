from glob import glob

from setuptools import find_packages, setup


package_name = "rescue_robot_survivor"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        ("share/" + package_name, ["package.xml", "README.md"]),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/docs", glob("docs/*.md")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="maze",
    maintainer_email="maze@todo.todo",
    description="Leader person detection, depth distance, and camera XYZ.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "person_detector_node = "
            "rescue_robot_survivor.person_detector_node:main",
            "survivor_map_transform_node = "
            "rescue_robot_survivor.survivor_map_transform_node:main",
            "survivor_map_visualizer_node = "
            "rescue_robot_survivor.survivor_map_visualizer_node:main",
            "survivor_registry_node = "
            "rescue_robot_survivor.survivor_registry_node:main",
            "survivor_registry_visualizer_node = "
            "rescue_robot_survivor.survivor_registry_visualizer_node:main",
        ],
    },
)
