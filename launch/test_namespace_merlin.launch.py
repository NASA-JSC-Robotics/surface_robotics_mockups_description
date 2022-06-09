from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    group = GroupAction(actions=[
        PushRosNamespace("/merlin_mockup"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [get_package_share_directory("merlin_mockup_description"), '/launch/view_merlin_mockup.launch.py']),
        ),
    ])

    return LaunchDescription([group])
   