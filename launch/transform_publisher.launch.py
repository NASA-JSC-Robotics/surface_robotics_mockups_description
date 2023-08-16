from launch import LaunchDescription
from mockup_transform_publishers.launch_common import load_transforms
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition, UnlessCondition
import os



def generate_launch_description():

    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            "use_bench",
            default_value="true",
            description="Launches the file that handles the static transform publishers for the bench seat",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_hatch_handle",
            default_value="true",
            description="Launches the file that handles the static transform publishers for the hatch grab handle",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_hatch_wheel",
            default_value="true",
            description="Launches the file that handles the static transform publishers for the hatch grab wheel",
        )
    )

    use_bench = LaunchConfiguration("use_bench")
    use_hatch_handle = LaunchConfiguration("use_hatch_handle")
    use_hatch_wheel = LaunchConfiguration("use_hatch_wheel")

    bench_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory("mockup_transform_publishers"), 'launch','bench_transform_publisher.launch.py')),
        condition=IfCondition(use_bench)
    )

    hatch_handle_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory("mockup_transform_publishers"), 'launch','hatch_handle_transform_publisher.launch.py')),
        condition=IfCondition(use_hatch_handle)
    )
    
    hatch_wheel_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory("mockup_transform_publishers"), 'launch','hatch_wheel_transform_publisher.launch.py')),
        condition=IfCondition(use_hatch_wheel)
    )

    launch_files = [
        bench_launch,
        hatch_handle_launch,
        hatch_wheel_launch,
    ]

    return LaunchDescription(declared_arguments + launch_files)

