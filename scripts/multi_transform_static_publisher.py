#!/usr/bin/env python3

import argparse
import os
import sys
import rclpy
import yaml

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import TransformStamped
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from tf2_ros import StaticTransformBroadcaster


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


def load_transforms(package_name, file_path, prefix=""):

    # Grab yaml transforms from file
    tf_yaml = load_yaml(package_name, file_path)
    if not tf_yaml:
        return None

    # Parse them all into static transforms
    transforms = []
    for tf in tf_yaml:
        tform = TransformStamped()
        tform.header.frame_id = prefix + tf["frame_id"]
        tform.child_frame_id = prefix + tf["name"]
        tform.transform.translation.x = float(tf["tx"])
        tform.transform.translation.y = float(tf["ty"])
        tform.transform.translation.z = float(tf["tz"])
        tform.transform.rotation.x = float(tf["rx"])
        tform.transform.rotation.y = float(tf["ry"])
        tform.transform.rotation.z = float(tf["rz"])
        tform.transform.rotation.w = float(tf["rw"])
        transforms.append(tform)

    return transforms


class MultiTransformNode(Node):
    def __init__(self, package_name, file_path, prefix):
        """
        Given a list of static tf transforms, construct a static transform publisher and publish them
        all to tf2.
        """
        super().__init__("static_transforms_publisher")
        self.get_logger().info(
            f"Loading static transforms from {package_name}:{file_path}")

        self.transforms = load_transforms(package_name, file_path, prefix)
        if not self.transforms:
            raise ValueError(
                f"No transforms were able to be loaded from {package_name}:{file_path}")
        self.get_logger().info(
            f"Loaded {len(self.transforms)} static transforms, publishing to /tf_static")

        # Publish transforms and keep the last message.
        self.broadcaster = StaticTransformBroadcaster(self)
        self.broadcaster.sendTransform(self.transforms)


def main(args=None):
    rclpy.init(args=args)

    parser = argparse.ArgumentParser()
    parser.add_argument('package_name', type=str,
                        help='The name of the package to pull transforms from')
    parser.add_argument('file_path', type=str,
                        help='The static transforms yaml file path in the package')
    parser.add_argument('--prefix', type=str, default="",
                        help='Optional transform prefix, defaults to an empty string')

    # Removes ros args and the script name from node arguments
    args = parser.parse_args(rclpy.utilities.remove_ros_args(args)[1:])

    node = MultiTransformNode(args.package_name, args.file_path, args.prefix)

    executor = SingleThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    except rclpy.executors.ExternalShutdownException:
        sys.exit(1)
    finally:
        executor.shutdown()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
