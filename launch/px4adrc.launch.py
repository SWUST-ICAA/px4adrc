from pathlib import Path

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    package_root = Path(__file__).resolve().parents[1]
    controller_config = package_root / "config" / "px4adrc.yaml"
    reference_config = package_root / "config" / "flatness_reference.yaml"

    return LaunchDescription([
        Node(
            package="px4adrc",
            executable="px4adrc_node",
            name="px4adrc",
            output="screen",
            parameters=[str(controller_config)],
        ),
        Node(
            package="px4adrc",
            executable="flatness_reference_publisher.py",
            name="px4adrc_reference",
            output="screen",
            parameters=[str(reference_config)],
        ),
    ])
