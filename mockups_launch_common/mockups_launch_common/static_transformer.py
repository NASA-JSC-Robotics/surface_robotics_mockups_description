from mockups_launch_common.yaml_loader import load_yaml
from launch_ros.actions import Node


def load_transforms(package_name, file_path, prefix=""):
    tf_yaml = load_yaml(package_name, file_path)
    nodes = []
    if tf_yaml is None:
        return nodes
    for tf in tf_yaml:
        try:
            all(key in tf for key in ("name", "frame_id", "tx", "ty", "tz", "rx", "ry", "rz", "rw"))
        except BaseException as error:  # TODO: find out what the actual exception should be here
            # BaseException chosen as catch all
            print("Error: not all translation/rotation elements found in transform yaml.")
            print(f"Error caught: {error}")
        new_node = Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name=tf["name"],
            arguments=[
                "--x",
                str(tf["tx"]),
                "--y",
                str(tf["ty"]),
                "--z",
                str(tf["tz"]),
                "--qx",
                str(tf["rx"]),
                "--qy",
                str(tf["ry"]),
                "--qz",
                str(tf["rz"]),
                "--qw",
                str(tf["rw"]),
                "--frame-id",
                prefix + tf["frame_id"],
                "--child-frame-id",
                prefix + tf["name"],
            ],
        )
        nodes.append(new_node)

    return nodes
