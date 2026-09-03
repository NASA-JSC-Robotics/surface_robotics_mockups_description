#!/usr/bin/env python3

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState
from mockup_msgs.srv import SetJointState


# Class that stores the information for each mockup config
class MockupConfig:
    def __init__(self, min_position, max_position, initial_position, joint_name):
        self.joint_name = joint_name
        self.min_position = min_position
        self.max_position = max_position
        self.position = initial_position
        self.velocity = 0.0
        self.effort = 0.0

    def set_position(self, desired_position):
        self.position = max(self.min_position, min(desired_position, self.max_position))


class MockupStateManager(Node):
    def __init__(self):
        super().__init__(
            "mockup_state_manager",
            allow_undeclared_parameters=True,
            automatically_declare_parameters_from_overrides=True,
        )

        # set default parameter of prefix to empty
        self.prefix = self.get_parameter("prefix").get_parameter_value().string_value

        self.load_mockup_configs()

        # create the joint state publisher
        self.publisher_ = self.create_publisher(JointState, "joint_states", 10)

        self.set_joint_state_service = self.create_service(SetJointState, "~/set_joint_state", self.set_joint_state_cb)

        # create the timer for joint state publisher callback
        timer_period_sec = 0.5  # unit: seconds
        self.timer = self.create_timer(timer_period_sec, self.joint_state_cb)

    def load_mockup_configs(self):
        """loads the parameters provided with each of the relevaant joints and populates self.mockup_configs"""
        # get the list of topic names first

        joints_params = self.get_parameters_by_prefix("joints")

        joints = {key.split(".")[0] for key in joints_params.keys()}

        # puopulate self.mockup_configs based on loaded parameters
        self.mockup_configs = dict()
        for joint in joints:
            self.get_logger().info(f"Loading: {self.prefix + joint}")

            joint_name = joint
            min_position = joints_params[f"{joint}.min_position"].value
            max_position = joints_params[f"{joint}.max_position"].value
            initial_position = joints_params[f"{joint}.initial_position"].value

            # add mockups to the member variable
            self.mockup_configs[joint_name] = MockupConfig(min_position, max_position, initial_position, joint_name)

    def joint_state_cb(self):
        """Publisher for the manager which publishes the joint state info."""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        # add all of the joint states
        for mockup_config in self.mockup_configs.values():
            msg.name.append(self.prefix + mockup_config.joint_name)
            msg.position.append(mockup_config.position)
            msg.velocity.append(mockup_config.velocity)
            msg.effort.append(mockup_config.effort)

        self.publisher_.publish(msg)

    def set_joint_state_cb(self, req: SetJointState.Request, res: SetJointState.Response):
        """
        Callback for the setting internal joint states which will then be published.
        """

        # Check first to make sure that data is the right size.
        # Any non-empty lists must be of the same size of names
        error_msg = ""
        msg_size = len(req.joint_state.name)
        valid = True

        for field in ("position", "velocity", "effort"):
            values = getattr(req.joint_state, field)
            if values and (len(values) != msg_size):
                error_msg += (
                    f"The size of `{field}` ({len(values)}) does not match the size of"
                    "`name` ({msg_size}) in SetJointState. "
                )
                valid = False

        # return early if not valid
        if not valid:
            res.message = error_msg
            res.success = False
            return res

        for i, name in enumerate(req.joint_state.name):
            mockup_config = self.mockup_configs[name]
            # position is special because we have to clamp it, so it uses a member function
            if req.joint_state.position:
                mockup_config.set_position(req.joint_state.position[i])
            if req.joint_state.velocity:
                mockup_config.velocity = req.joint_state.velocity[i]
            if req.joint_state.effort:
                mockup_config.effort = req.joint_state.effort[i]

        res.success = True
        return res


def main(args=None):
    rclpy.init(args=args)

    mockup_state_manager = MockupStateManager()

    try:
        rclpy.spin(mockup_state_manager)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        mockup_state_manager.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
