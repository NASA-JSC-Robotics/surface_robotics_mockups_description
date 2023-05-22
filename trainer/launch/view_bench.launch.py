from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    declared_arguments = []

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
<<<<<<<< HEAD:hatch_4040/launch/view.launch.py
            PathJoinSubstitution([FindPackageShare("hatch_4040"), "urdf", "hatch_4040.urdf.xacro"]),
========
            PathJoinSubstitution([FindPackageShare("trainer"), "urdf", "bench_seat.urdf.xacro"]),
>>>>>>>> humble-feature/DEXTIVR-207:trainer/launch/view_bench.launch.py
        ]
    )

    robot_description = {"robot_description": robot_description_content}

    rviz_config_file = PathJoinSubstitution(
<<<<<<<< HEAD:hatch_4040/launch/view.launch.py
        [FindPackageShare("hatch_4040"), "rviz", "visualization.rviz"]
========
        [FindPackageShare("trainer"), "rviz", "visualization.rviz"]
>>>>>>>> humble-feature/DEXTIVR-207:trainer/launch/view_bench.launch.py
    )

    joint_state_publisher_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
    )
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description]
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
    )

    nodes_to_start = [
        joint_state_publisher_node,
        robot_state_publisher_node,
        rviz_node,
    ]

    return LaunchDescription(declared_arguments + nodes_to_start)
