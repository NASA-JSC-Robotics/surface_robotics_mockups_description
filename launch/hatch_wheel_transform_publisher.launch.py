from launch import LaunchDescription
from launch_ros.actions import Node
from mockup_transform_publishers.launch_common import load_transforms

def generate_launch_description():
    transform_nodes = load_transforms("mockup_transform_publishers", "config/hatch_wheel_transforms.yaml")

    declared_arguments = []

    return LaunchDescription(declared_arguments + transform_nodes)

