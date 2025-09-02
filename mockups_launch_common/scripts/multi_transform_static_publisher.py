#!/usr/bin/env python3

import argparse
import sys
import rclpy

from mockups_launch_common.yaml_loader import load_tf_transforms

from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from tf2_ros import StaticTransformBroadcaster


class MultiTransformNode(Node):
    def __init__(self, package_name, file_path, prefix):
        """
        Given a list of static tf transforms, construct a static transform publisher and publish them
        all to tf2.
        """
        super().__init__("static_transforms_publisher")
        self.get_logger().info(f"Loading static transforms from {package_name}:{file_path}")

        self.transforms = load_tf_transforms(package_name, file_path, prefix)
        if not self.transforms:
            raise ValueError(f"No transforms were able to be loaded from {package_name}:{file_path}")
        self.get_logger().info(f"Loaded {len(self.transforms)} static transforms, publishing to /tf_static")

        # Publish transforms and keep the last message.
        self.broadcaster = StaticTransformBroadcaster(self)
        self.broadcaster.sendTransform(self.transforms)


def main(args=None):
    rclpy.init(args=args)

    parser = argparse.ArgumentParser()
    parser.add_argument("package_name", type=str, help="The name of the package to pull transforms from")
    parser.add_argument("file_path", type=str, help="The static transforms yaml file path in the package")
    parser.add_argument("--prefix", type=str, default="", help="Optional transform prefix, defaults to an empty string")

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


if __name__ == "__main__":
    main()
