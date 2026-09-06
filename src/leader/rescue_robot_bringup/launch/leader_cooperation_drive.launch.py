"""Launch the Leader motor bridge and cooperative-transport coordinator."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    LogInfo,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def _include(package_name, launch_file, launch_arguments=None):
    launch_path = os.path.join(
        get_package_share_directory(package_name),
        "launch",
        launch_file,
    )
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(launch_path),
        launch_arguments=(launch_arguments or {}).items(),
    )


def generate_launch_description() -> LaunchDescription:
    """Start only the Leader components needed for manual cooperative driving."""
    use_stm32_bridge = LaunchConfiguration("use_stm32_bridge")
    i2c_device = LaunchConfiguration("i2c_device")
    i2c_address = LaunchConfiguration("i2c_address")
    i2c_write_enabled = LaunchConfiguration("i2c_write_enabled")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_stm32_bridge",
                default_value="true",
                choices=["true", "false"],
                description="Launch the Leader STM32 command bridge",
            ),
            DeclareLaunchArgument("i2c_device", default_value="/dev/i2c-7"),
            DeclareLaunchArgument("i2c_address", default_value="66"),
            DeclareLaunchArgument(
                "i2c_write_enabled",
                default_value="true",
                choices=["true", "false"],
            ),
            LogInfo(
                msg=(
                    "Leader cooperation startup safety: waiting for explicit "
                    "/cooperation/enable and teleop input."
                )
            ),
            _include(
                "leader_cooperation",
                "leader_cooperation.launch.py",
            ),
            GroupAction(
                condition=IfCondition(use_stm32_bridge),
                actions=[
                    _include(
                        "stm32_bridge",
                        "stm32_bridge.launch.py",
                        {
                            "transport": "i2c",
                            "i2c_device": i2c_device,
                            "i2c_address": i2c_address,
                            "i2c_write_enabled": i2c_write_enabled,
                            "namespace": "leader",
                        },
                    )
                ],
            ),
        ]
    )
