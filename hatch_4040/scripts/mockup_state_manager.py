#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
import copy
import threading


# Class that stores the information for each mockup config
class MockupConfig:
    def __init__(self, topic_name, min_position, max_position, initial_position, joint_name):
        self.topic_name = topic_name
        self.joint_name = joint_name
        self.min_position = min_position
        self.max_position = max_position
        self.position = initial_position


class MockupStateManager(Node):
    def __init__(self):
        super().__init__("mockup_state_manager")

        # set default parameter of prefix to empty
        self.declare_parameter("prefix", "")
        self.prefix = self.get_parameter("prefix").get_parameter_value().string_value

        self.load_mockup_configs()

        # create the joint state publisher
        self.publisher_ = self.create_publisher(JointState, "joint_states", 10)

        subscriptions = []
        for index, mockup_config in enumerate(self.mockup_configs):
            # this was a bit funky for copying in index
            # see https://github.com/ros2/rclpy/issues/629#issuecomment-1542151499 for reference
            subscriptions.append(
                self.create_subscription(
                    Float64,
                    self.prefix + mockup_config.topic_name,
                    lambda msg, idx=index: self.position_cb(msg, idx),
                    10,
                )
            )

        # create the timer for joint state publisher callback
        timer_period_sec = 0.5  # unit: seconds
        self.timer = self.create_timer(timer_period_sec, self.joint_state_cb)

        self.lock = threading.Lock()

        # update the joint states
        self._joint_states = dict()
        for mockup_config in self.mockup_configs:
            self._joint_states[self.prefix + mockup_config.joint_name] = copy.deepcopy(mockup_config.position)

        self.joint_state_cb()  # going ahead and starting

    def load_mockup_configs(self):
        """loads the parameters provided with each of the relevaant joints and populates self.mockup_configs"""
        # get the list of topic names first
        self.declare_parameter("topic_names", [""])
        self.topic_names = self.get_parameter("topic_names").get_parameter_value().string_array_value

        # if we didn't get any topic names, don't try anything else
        if self.topic_names == [""]:
            self.get_logger().warning("Did not find any topics, is your parameter file set up properly?")
            return

        # puopulate self.mockup_configs based on loaded parameters
        self.mockup_configs = []
        for topic in self.topic_names:
            self.get_logger().info(f"Loading: {self.prefix + topic}")
            self.declare_parameter("topics." + topic + ".joint_name", "")
            self.declare_parameter("topics." + topic + ".min_position", 0.0)
            self.declare_parameter("topics." + topic + ".max_position", 0.0)
            self.declare_parameter("topics." + topic + ".initial_position", 0.0)

            joint_name = self.get_parameter("topics." + topic + ".joint_name").get_parameter_value().string_value
            min_position = self.get_parameter("topics." + topic + ".min_position").get_parameter_value().double_value
            max_position = self.get_parameter("topics." + topic + ".max_position").get_parameter_value().double_value
            initial_position = (
                self.get_parameter("topics." + topic + ".initial_position").get_parameter_value().double_value
            )

            # add mockups to the member variable
            self.mockup_configs.append(MockupConfig(topic, min_position, max_position, initial_position, joint_name))

    def joint_state_cb(self):
        """Publisher for the manager which publishes the joint state info."""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        # with mutex locking, add all of the joint states
        with self.lock:
            for mockup_config in self.mockup_configs:
                msg.name.append(self.prefix + mockup_config.joint_name)
                msg.position.append(mockup_config.position)
                msg.velocity.append(0.0)
                msg.effort.append(0.0)

        self.publisher_.publish(msg)

        self.get_logger().debug(
            f"This is the hatch joint state message: {msg}"
        )  # this will fill in the string with formatted msg data

    def position_cb(self, msg: Float64, index):
        """callback for the position setting

        Args:
            msg (Float64): This message transmits the joint position (in radians or meters)
            index (int): the index that we are accessing (to ineracte with self.mockup_configs
        """
        self.get_logger().debug(
            f"This is the new position of {self.prefix + self.mockup_configs[index].joint_name}: {msg}"
        )
        if self.mockup_configs[index].min_position <= msg.data and msg.data <= self.mockup_configs[index].max_position:
            with self.lock:
                self.mockup_configs[index].position = msg.data
        elif self.mockup_configs[index].min_position >= msg.data:
            with self.lock:
                self.mockup_configs[index].position = copy.deepcopy(self.mockup_configs[index].min_position)
        elif self.mockup_configs[index].max_position <= msg.data:
            with self.lock:
                self.mockup_configs[index].position = copy.deepcopy(self.mockup_configs[index].max_position)
        else:  # I don't think this will ever happen 0.o
            raise Exception(self.mockup_configs[index].joint_name + " joint angle out of range")


def main(args=None):
    rclpy.init(args=args)

    mockup_state_manager = MockupStateManager()

    rclpy.spin(mockup_state_manager)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    mockup_state_manager.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
