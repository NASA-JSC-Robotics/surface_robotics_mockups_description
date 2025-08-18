#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
import copy
import math
import threading


class TrainerManager(Node):
    def __init__(self):
        super().__init__("trainer_manager")

        # set default parameter of prefix to empty
        self.declare_parameter("prefix", "")
        self.prefix = self.get_parameter("prefix").get_parameter_value().string_value

        # create the joint state publisher
        self.publisher_ = self.create_publisher(JointState, "/joint_states", 10)

        # create subscriptions's to revolute/prismatic joints
        self.bench_sub = self.create_subscription(Float64, self.prefix + "bench_position", self.bench_cb, 10)
        self.cabinet_1_sub = self.create_subscription(
            Float64, self.prefix + "cabinet_1_position", self.cabinet_1_cb, 10
        )
        self.round_latch_sub = self.create_subscription(
            Float64, self.prefix + "round_latch_position", self.round_latch_cb, 10
        )
        self.cabinet_2_sub = self.create_subscription(
            Float64, self.prefix + "cabinet_2_position", self.cabinet_2_cb, 10
        )
        self.push_latch_button_sub = self.create_subscription(
            Float64, self.prefix + "push_latch_button_position", self.push_latch_button_cb, 10
        )
        self.push_latch_handle_sub = self.create_subscription(
            Float64, self.prefix + "push_latch_handle_position", self.push_latch_handle_cb, 10
        )
        self.cabinet_3_sub = self.create_subscription(
            Float64, self.prefix + "cabinet_3_position", self.cabinet_3_cb, 10
        )
        self.cabinet_4_sub = self.create_subscription(
            Float64, self.prefix + "cabinet_4_position", self.cabinet_4_cb, 10
        )
        self.paddle_latch_handle_sub = self.create_subscription(
            Float64, self.prefix + "paddle_latch_handle_position", self.paddle_latch_handle_cb, 10
        )
        self.drawer_1_sub = self.create_subscription(Float64, self.prefix + "drawer_1_position", self.drawer_1_cb, 10)
        self.drawer_2_sub = self.create_subscription(Float64, self.prefix + "drawer_2_position", self.drawer_2_cb, 10)
        self.recessed_latch_handle_sub = self.create_subscription(
            Float64, self.prefix + "recessed_latch_handle_position", self.recessed_latch_handle_cb, 10
        )

        # initialize
        self.bench_position = 0.0
        self.cabinet_1_position = 0.0
        self.round_latch_position = 0.0
        self.cabinet_2_position = 0.0
        self.push_latch_button_position = 0.0
        self.push_latch_handle_position = 0.0
        self.cabinet_3_position = 0.0
        self.cabinet_4_position = 0.0
        self.paddle_latch_handle_position = 0.0
        self.drawer_1_position = 0.0
        self.drawer_2_position = 0.0
        self.recessed_latch_handle_position = 0.0

        # ranges
        self.bench_max = round(95.0 * math.pi / 180.0, 5)
        self.cabinet_1_max = round(math.pi, 5)
        self.round_latch_max = round(math.pi, 5)
        self.cabinet_2_max = round(math.pi, 5)
        self.push_latch_button_max = round(0.005, 5)
        self.push_latch_handle_max = round(math.pi / 3, 5)
        self.cabinet_3_max = round(math.pi, 5)
        self.cabinet_4_max = round(math.pi, 5)
        self.paddle_latch_handle_max = round(0.785375, 5)
        self.drawer_1_max = round(0.305, 5)
        self.drawer_2_max = round(0.2921, 5)
        self.recessed_latch_handle_max = round(math.pi / 2, 5)

        self.bench_min = 0.0
        self.cabinet_1_min = 0.0
        self.round_latch_min = 0.0
        self.cabinet_2_min = 0.0
        self.push_latch_button_min = 0.0
        self.push_latch_handle_min = 0.0
        self.cabinet_3_min = 0.0
        self.cabinet_4_min = 0.0
        self.paddle_latch_handle_min = 0.0
        self.drawer_1_min = 0.0
        self.drawer_2_min = 0.0
        self.recessed_latch_handle_min = 0.0

        # update the joint states
        if self.get_namespace() != "/":
            self._trainer_joint_states = {
                self.get_namespace()[1:] + "/" + self.prefix + "bench_lid_joint": copy.deepcopy(self.bench_position),
                self.get_namespace()[1:]
                + "/"
                + self.prefix
                + "cabinet_1_joint": copy.deepcopy(self.cabinet_1_position),
                self.get_namespace()[1:]
                + "/"
                + self.prefix
                + "round_latch_joint": copy.deepcopy(self.round_latch_position),
                self.get_namespace()[1:]
                + "/"
                + self.prefix
                + "cabinet_2_joint": copy.deepcopy(self.cabinet_2_position),
                self.get_namespace()[1:]
                + "/"
                + self.prefix
                + "push_latch_button_joint": copy.deepcopy(self.push_latch_button_position),
                self.get_namespace()[1:]
                + "/"
                + self.prefix
                + "push_latch_handle_joint": copy.deepcopy(self.push_latch_handle_position),
                self.get_namespace()[1:]
                + "/"
                + self.prefix
                + "cabinet_3_joint": copy.deepcopy(self.cabinet_3_position),
                self.get_namespace()[1:]
                + "/"
                + self.prefix
                + "cabinet_4_joint": copy.deepcopy(self.cabinet_4_position),
                self.get_namespace()[1:]
                + "/"
                + self.prefix
                + "paddle_latch_handle_joint": copy.deepcopy(self.paddle_latch_handle_position),
                self.get_namespace()[1:] + "/" + self.prefix + "drawer_1_joint": copy.deepcopy(self.drawer_1_position),
                self.get_namespace()[1:] + "/" + self.prefix + "drawer_2_joint": copy.deepcopy(self.drawer_2_position),
                self.get_namespace()[1:]
                + "/"
                + self.prefix
                + "recessed_latch_handle_joint": copy.deepcopy(self.recessed_latch_handle_position),
            }

        else:
            self._trainer_joint_states = {
                self.prefix + "bench_lid_joint": copy.deepcopy(self.bench_position),
                self.prefix + "cabinet_1_joint": copy.deepcopy(self.cabinet_1_position),
                self.prefix + "round_latch_joint": copy.deepcopy(self.round_latch_position),
                self.prefix + "cabinet_2_joint": copy.deepcopy(self.cabinet_2_position),
                self.prefix + "push_latch_button_joint": copy.deepcopy(self.push_latch_button_position),
                self.prefix + "push_latch_handle_joint": copy.deepcopy(self.push_latch_handle_position),
                self.prefix + "cabinet_3_joint": copy.deepcopy(self.cabinet_3_position),
                self.prefix + "cabinet_4_joint": copy.deepcopy(self.cabinet_4_position),
                self.prefix + "paddle_latch_handle_joint": copy.deepcopy(self.paddle_latch_handle_position),
                self.prefix + "drawer_1_joint": copy.deepcopy(self.drawer_1_position),
                self.prefix + "drawer_2_joint": copy.deepcopy(self.drawer_2_position),
                self.prefix + "recessed_latch_handle_joint": copy.deepcopy(self.recessed_latch_handle_position),
            }

        self.lock = threading.Lock()

        # create the timer for joint state publisher callback
        timer_period_sec = 0.5  # unit: seconds
        self.timer = self.create_timer(timer_period_sec, self.trainer_joint_state_cb)

    def trainer_joint_state_cb(self):
        """Publisher for the manager which publishes the joint state info."""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        if self.get_namespace() != "/":
            msg.name = [
                self.get_namespace()[1:] + "/" + self.prefix + "bench_lid_joint",
                self.get_namespace()[1:] + "/" + self.prefix + "cabinet_1_joint",
                self.get_namespace()[1:] + "/" + self.prefix + "round_latch_joint",
                self.get_namespace()[1:] + "/" + self.prefix + "cabinet_2_joint",
                self.get_namespace()[1:] + "/" + self.prefix + "push_latch_button_joint",
                self.get_namespace()[1:] + "/" + self.prefix + "push_latch_handle_joint",
                self.get_namespace()[1:] + "/" + self.prefix + "cabinet_3_joint",
                self.get_namespace()[1:] + "/" + self.prefix + "cabinet_4_joint",
                self.get_namespace()[1:] + "/" + self.prefix + "paddle_latch_handle_joint",
                self.get_namespace()[1:] + "/" + self.prefix + "drawer_1_joint",
                self.get_namespace()[1:] + "/" + self.prefix + "drawer_2_joint",
                self.get_namespace()[1:] + "/" + self.prefix + "recessed_latch_handle_joint",
            ]
        else:
            msg.name = [
                self.prefix + "bench_lid_joint",
                self.prefix + "cabinet_1_joint",
                self.prefix + "round_latch_joint",
                self.prefix + "cabinet_2_joint",
                self.prefix + "push_latch_button_joint",
                self.prefix + "push_latch_handle_joint",
                self.prefix + "cabinet_3_joint",
                self.prefix + "cabinet_4_joint",
                self.prefix + "paddle_latch_handle_joint",
                self.prefix + "drawer_1_joint",
                self.prefix + "drawer_2_joint",
                self.prefix + "recessed_latch_handle_joint",
            ]
        with self.lock:
            msg.position = [
                self.bench_position,
                self.cabinet_1_position,
                self.round_latch_position,
                self.cabinet_2_position,
                self.push_latch_button_position,
                self.push_latch_handle_position,
                self.cabinet_3_position,
                self.cabinet_4_position,
                self.paddle_latch_handle_position,
                self.drawer_1_position,
                self.drawer_2_position,
                self.recessed_latch_handle_position,
            ]
            msg.velocity = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

            msg.effort = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            for x, joint_name in enumerate(msg.name):
                if joint_name in self._trainer_joint_states.keys():
                    self._trainer_joint_states[joint_name] = copy.deepcopy(msg.position[x])

        self.publisher_.publish(msg)

        self.get_logger().debug(f"This is the hatch joint state message: {msg}")

    def bench_cb(self, msg: Float64):
        """callback for the bench door position

        Args:
            msg (Float64): This message transmits the revolute joint angle (in radians)
        """
        self.get_logger().debug(f"This is the angle of the bench joint: {msg}")
        if self.bench_min <= msg.data and msg.data <= self.bench_max:
            with self.lock:
                self.bench_position = msg.data
        elif self.bench_min >= msg.data:
            with self.lock:
                self.bench_position = copy.deepcopy(self.bench_min)
        elif self.bench_max <= msg.data:
            with self.lock:
                self.bench_position = copy.deepcopy(self.bench_max)
        else:
            raise Exception("bench joint angle out of range")

    def cabinet_1_cb(self, msg: Float64):
        """callback for cabinet 1 position

        Args:
            msg (Float64): This message transmits the revolute joint angle (in radians)
        """
        self.get_logger().debug(f"This is the value of the cabinet 1 joint: {msg}")
        if self.cabinet_1_min <= msg.data and msg.data <= self.cabinet_1_max:
            with self.lock:
                self.cabinet_1_position = msg.data
        elif self.cabinet_1_min >= msg.data:
            with self.lock:
                self.cabinet_1_position = copy.deepcopy(self.cabinet_1_min)
        elif self.cabinet_1_max <= msg.data:
            with self.lock:
                self.cabinet_1_position = copy.deepcopy(self.cabinet_1_max)
        else:
            raise Exception("cabinet 1 value out of range")

    def round_latch_cb(self, msg: Float64):
        """callback for round latch position

        Args:
            msg (Float64): This message transmits the revolute joint angle (in radians)
        """
        self.get_logger().debug(f"This is the value of the round latch joint: {msg}")
        if self.round_latch_min <= msg.data and msg.data <= self.round_latch_max:
            with self.lock:
                self.round_latch_position = msg.data
        elif self.round_latch_min >= msg.data:
            with self.lock:
                self.round_latch_position = copy.deepcopy(self.round_latch_min)
        elif self.round_latch_max <= msg.data:
            with self.lock:
                self.round_latch_position = copy.deepcopy(self.round_latch_max)
        else:
            raise Exception("round latch value out of range")

    def cabinet_2_cb(self, msg: Float64):
        """callback for cabinet 2 position

        Args:
            msg (Float64): This message transmits the revolute joint angle (in radians)
        """
        self.get_logger().debug(f"This is the value of the cabinet 2 joint: {msg}")
        if self.cabinet_2_min <= msg.data and msg.data <= self.cabinet_2_max:
            with self.lock:
                self.cabinet_2_position = msg.data
        elif self.cabinet_2_min >= msg.data:
            with self.lock:
                self.cabinet_2_position = copy.deepcopy(self.cabinet_2_min)
        elif self.cabinet_2_max <= msg.data:
            with self.lock:
                self.cabinet_2_position = copy.deepcopy(self.cabinet_2_max)
        else:
            raise Exception("cabinet 2 value out of range")

    def push_latch_button_cb(self, msg: Float64):
        """callback for push latch position

        Args:
            msg (Float64): This message transmits the revolute joint angle (in radians)
        """
        self.get_logger().debug(f"This is the value of the push latch joint: {msg}")
        if self.push_latch_button_min <= msg.data and msg.data <= self.push_latch_button_max:
            with self.lock:
                self.push_latch_button_position = msg.data
        elif self.push_latch_button_min >= msg.data:
            with self.lock:
                self.push_latch_button_position = copy.deepcopy(self.push_latch_button_min)
        elif self.push_latch_button_max <= msg.data:
            with self.lock:
                self.push_latch_button_position = copy.deepcopy(self.push_latch_button_max)
        else:
            raise Exception("push latch value out of range")

    def push_latch_handle_cb(self, msg: Float64):
        """callback for push latch position

        Args:
            msg (Float64): This message transmits the revolute joint angle (in radians)
        """
        self.get_logger().debug(f"This is the value of the push latch joint: {msg}")
        if self.push_latch_handle_min <= msg.data and msg.data <= self.push_latch_handle_max:
            with self.lock:
                self.push_latch_handle_position = msg.data
        elif self.push_latch_handle_min >= msg.data:
            with self.lock:
                self.push_latch_handle_position = copy.deepcopy(self.push_latch_handle_min)
        elif self.push_latch_handle_max <= msg.data:
            with self.lock:
                self.push_latch_handle_position = copy.deepcopy(self.push_latch_handle_max)
        else:
            raise Exception("push latch value out of range")

    def cabinet_3_cb(self, msg: Float64):
        """callback for cabinet 3 position

        Args:
            msg (Float64): This message transmits the revolute joint angle (in radians)
        """
        self.get_logger().debug(f"This is the value of the cabinet 3 joint: {msg}")
        if self.cabinet_3_min <= msg.data and msg.data <= self.cabinet_3_max:
            with self.lock:
                self.cabinet_3_position = msg.data
        elif self.cabinet_3_min >= msg.data:
            with self.lock:
                self.cabinet_3_position = copy.deepcopy(self.cabinet_3_min)
        elif self.cabinet_3_max <= msg.data:
            with self.lock:
                self.cabinet_3_position = copy.deepcopy(self.cabinet_3_max)
        else:
            raise Exception("cabinet 3 value out of range")

    def cabinet_4_cb(self, msg: Float64):
        """callback for cabinet 4 position

        Args:
            msg (Float64): This message transmits the revolute joint angle (in radians)
        """
        self.get_logger().debug(f"This is the value of the cabinet 4 joint: {msg}")
        if self.cabinet_4_min <= msg.data and msg.data <= self.cabinet_4_max:
            with self.lock:
                self.cabinet_4_position = msg.data
        elif self.cabinet_4_min >= msg.data:
            with self.lock:
                self.cabinet_4_position = copy.deepcopy(self.cabinet_4_min)
        elif self.cabinet_4_max <= msg.data:
            with self.lock:
                self.cabinet_4_position = copy.deepcopy(self.cabinet_4_max)
        else:
            raise Exception("cabinet 3 value out of range")

    def paddle_latch_handle_cb(self, msg: Float64):
        """callback for paddle latch position

        Args:
            msg (Float64): This message transmits the revolute joint angle (in radians)
        """
        self.get_logger().debug(f"This is the value of the paddle latch joint: {msg}")
        if self.paddle_latch_handle_min <= msg.data and msg.data <= self.paddle_latch_handle_max:
            with self.lock:
                self.paddle_latch_handle_position = msg.data
        elif self.paddle_latch_handle_min >= msg.data:
            with self.lock:
                self.paddle_latch_handle_position = copy.deepcopy(self.paddle_latch_handle_min)
        elif self.paddle_latch_handle_max <= msg.data:
            with self.lock:
                self.paddle_latch_handle_position = copy.deepcopy(self.paddle_latch_handle_max)
        else:
            raise Exception("paddle latch value out of range")

    def drawer_1_cb(self, msg: Float64):
        """callback for drawer 1 position

        Args:
            msg (Float64): This message transmits the revolute joint angle (in radians)
        """
        self.get_logger().debug(f"This is the value of the drawer 1 joint: {msg}")
        if self.drawer_1_min <= msg.data and msg.data <= self.drawer_1_max:
            with self.lock:
                self.drawer_1_position = msg.data
        elif self.drawer_1_min >= msg.data:
            with self.lock:
                self.drawer_1_position = copy.deepcopy(self.drawer_1_min)
        elif self.drawer_1_max <= msg.data:
            with self.lock:
                self.drawer_1_position = copy.deepcopy(self.drawer_1_max)
        else:
            raise Exception("drawer 1 value out of range")

    def drawer_2_cb(self, msg: Float64):
        """callback for drawer 2 position

        Args:
            msg (Float64): This message transmits the revolute joint angle (in radians)
        """
        self.get_logger().debug(f"This is the value of the drawer 2 joint: {msg}")
        if self.drawer_2_min <= msg.data and msg.data <= self.drawer_2_max:
            with self.lock:
                self.drawer_2_position = msg.data
        elif self.drawer_2_min >= msg.data:
            with self.lock:
                self.drawer_1_position = copy.deepcopy(self.drawer_2_min)
        elif self.drawer_2_max <= msg.data:
            with self.lock:
                self.drawer_1_position = copy.deepcopy(self.drawer_2_max)
        else:
            raise Exception("drawer 2 value out of range")

    def recessed_latch_handle_cb(self, msg: Float64):
        """callback for recessed latch handle position

        Args:
            msg (Float64): This message transmits the revolute joint angle (in radians)
        """
        self.get_logger().debug(f"This is the value of the recessed latch handle joint: {msg}")
        if self.recessed_latch_handle_min <= msg.data and msg.data <= self.recessed_latch_handle_max:
            with self.lock:
                self.recessed_latch_handle_position = msg.data
        elif self.recessed_latch_handle_min >= msg.data:
            with self.lock:
                self.recessed_latch_handle_position = copy.deepcopy(self.recessed_latch_handle_min)
        elif self.recessed_latch_handle_max <= msg.data:
            with self.lock:
                self.recessed_latch_handle_position = copy.deepcopy(self.recessed_latch_handle_max)
        else:
            raise Exception("recessed latch handle value out of range")


def main(args=None):
    rclpy.init(args=args)

    trainer_manager = TrainerManager()

    try:
        rclpy.spin(trainer_manager)
    except KeyboardInterrupt:
        pass
    finally:
        # Destroy the node explicitly
        # (optional - otherwise it will be done automatically
        # when the garbage collector destroys the node object)
        trainer_manager.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
