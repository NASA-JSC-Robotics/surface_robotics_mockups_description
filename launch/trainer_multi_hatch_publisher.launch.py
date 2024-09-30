import os

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    nodes_to_launch = []

    site_config_path = "trainer_multi_hatch_transforms"

    nodes_to_launch.append(Node(
        package="clr_trainer_hatch_offsets",
        executable="multi_transform_static_publisher.py",
        name="bench_transforms",
        arguments=["clr_trainer_hatch_offsets",
                   os.path.join("config", site_config_path, "bench_transforms.yaml"),
                   ]
    ))

    nodes_to_launch.append(Node(
        package="clr_trainer_hatch_offsets",
        executable="multi_transform_static_publisher.py",
        name="smol_hatch_4040_transforms",
        arguments=["clr_trainer_hatch_offsets",
                   os.path.join("config", site_config_path, "hatch_4040_transforms.yaml"),
                   "--prefix", "smol/",
                   ]
    ))

    nodes_to_launch.append(Node(
        package="clr_trainer_hatch_offsets",
        executable="multi_transform_static_publisher.py",
        name="lorge_hatch_4060_transforms",
        arguments=["clr_trainer_hatch_offsets",
                   os.path.join("config", site_config_path, "hatch_4060_transforms.yaml"),
                   "--prefix", "lorge/",
                   ]
    ))

    nodes_to_launch.append(Node(
        package="clr_trainer_hatch_offsets",
        executable="multi_transform_static_publisher.py",
        name="intermediate_transforms",
        arguments=["clr_trainer_hatch_offsets",
                   os.path.join("config", site_config_path, "intermediate_transforms.yaml"),
                   ]
    ))

    return LaunchDescription(nodes_to_launch)
