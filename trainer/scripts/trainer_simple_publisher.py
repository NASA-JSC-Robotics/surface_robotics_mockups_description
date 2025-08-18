#!/usr/bin/env python3

from time import sleep

import rclpy

from sensor_msgs.msg import JointState


def main(args=None):
    rclpy.init(args=args)

    node = rclpy.create_node("trainer_publisher")

    publisher = node.create_publisher(JointState, "joint_states", 10)

    msg = JointState()

    i = 0
    while rclpy.ok():
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.name = [
            "bench_lid_joint",
            "cabinet_1_joint",
            "round_latch_joint",
            "cabinet_2_joint",
            "push_latch_button_joint",
            "push_latch_handle_joint",
            "cabinet_3_joint",
            "cabinet_4_joint",
            "paddle_latch_handle_joint",
            "drawer_1_joint",
            "drawer_2_joint",
            "recessed_latch_handle_joint",
        ]
        msg.position = [1.56, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.00015707963267948965]

        msg.velocity = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        msg.effort = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        publisher.publish(msg)
        sleep(0.5)  # seconds

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
