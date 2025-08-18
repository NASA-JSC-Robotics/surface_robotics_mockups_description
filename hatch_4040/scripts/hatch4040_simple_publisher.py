#!/usr/bin/env python3

from time import sleep

import rclpy

from sensor_msgs.msg import JointState


def main(args=None):
    rclpy.init(args=args)

    node = rclpy.create_node("hatch4040_publisher")

    publisher = node.create_publisher(JointState, "joint_states", 10)

    msg = JointState()

    while rclpy.ok():
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.name = [
            "hatch_4040_frame_face_joint",
            "external_rotary_joint",
            "external_rotary_handle_joint",
            "internal_rotary_joint",
            "internal_rotary_handle_joint",
        ]
        msg.position = [0.0, 0.0, 0.0, 0.0, 0.0]
        msg.velocity = [0.0, 0.0, 0.0, 0.0, 0.0]
        msg.effort = [0.0, 0.0, 0.0, 0.0, 0.0]
        publisher.publish(msg)
        sleep(0.5)  # seconds

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
