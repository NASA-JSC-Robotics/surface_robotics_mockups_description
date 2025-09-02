from launch import LaunchDescription
from mockups_launch_common.static_transformer import load_transforms


def generate_launch_description():

    declared_arguments = []

    transform_offset_node = load_transforms("mockups_launch_common", "config/" + "example_transforms.yaml")

    nodes_to_launch = [*transform_offset_node]

    return LaunchDescription(declared_arguments + nodes_to_launch)
