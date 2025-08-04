import os

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    nodes_to_launch = []

    site_config_path = "trainer_hatch_transforms/"

    nodes_to_launch.append(Node(
        package="drt_ros2_launch_common",
        executable="multi_transform_static_publisher.py",
        name="bench_transforms",
        arguments=["mockups_offsets",
                   os.path.join("config", site_config_path, "bench_transforms.yaml"),
                   ]
    ))

    nodes_to_launch.append(Node(
        package="drt_ros2_launch_common",
        executable="multi_transform_static_publisher.py",
        name="hatch_4040_transforms",
        arguments=["mockups_offsets",
                   os.path.join("config", site_config_path, "hatch_4040_transforms.yaml"),
                   ]
    ))

    nodes_to_launch.append(Node(
        package="drt_ros2_launch_common",
        executable="multi_transform_static_publisher.py",
        name="hatch_4060_transforms",
        arguments=["mockups_offsets",
                   os.path.join("config", site_config_path, "hatch_4060_transforms.yaml"),
                   ]
    ))

    nodes_to_launch.append(Node(
        package="drt_ros2_launch_common",
        executable="multi_transform_static_publisher.py",
        name="intermediate_transforms",
        arguments=["mockups_offsets",
                   os.path.join("config", site_config_path, "intermediate_transforms.yaml"),
                   ]
    ))

    return LaunchDescription(nodes_to_launch)
