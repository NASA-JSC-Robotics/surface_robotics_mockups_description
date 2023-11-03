from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
import os
import yaml

def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path) as file:
            try:
                return yaml.safe_load(file)
            except yaml.YAMLError as exc:
                print(exc)
    except OSError:  # parent of IOError, OSError *and* WindowsError where available
        print("Was not able to load the yaml file at " + absolute_file_path)
        return None

def load_transforms(package_name, file_path, prefix=''):
    tf_yaml = load_yaml(package_name, file_path)
    nodes = []
    if tf_yaml == None:
        return nodes
    for tf in tf_yaml:
        new_node = Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name=tf["name"],
            arguments=["--x",  str(tf["tx"]), 
                       "--y",  str(tf["ty"]), 
                       "--z",  str(tf["tz"]), 
                       "--qx", str(tf["rx"]), 
                       "--qy", str(tf["ry"]),
                       "--qz", str(tf["rz"]),
                       "--qw", str(tf["rw"]),
                       "--frame-id", prefix + tf["frame_id"],
                       "--child-frame-id", prefix + tf["name"]]
        )        
        nodes.append(new_node)

    return nodes