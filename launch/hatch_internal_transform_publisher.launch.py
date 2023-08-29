from launch import LaunchDescription
from launch_ros.actions import Node
from clr_trainer_hatch_offsets.launch_common import load_transforms

def generate_launch_description():
    transform_nodes = load_transforms("clr_trainer_hatch_offsets", "config/hatch_internal_transforms.yaml")

    declared_arguments = []

    return LaunchDescription(declared_arguments + transform_nodes)

