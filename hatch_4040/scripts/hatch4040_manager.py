#!/usr/bin/env python3

import sys
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from geometry_msgs.msg import PoseStamped, TransformStamped, Quaternion
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
import copy, math, threading

class Hatch4040Manager(Node):
    def __init__(self):
        super().__init__('hatch4040_manager')

        # set deafult parameter of prefix to empty
        self.declare_parameter('prefix', '')
        self.prefix = self.get_parameter('prefix').get_parameter_value().string_value

        # create the joint state publisher
        self.publisher_ = self.create_publisher(JointState, 'joint_states', 10)
        
        # create subscriptions's to revolute/prismatic joints
        self.hatch_sub = self.create_subscription(Float64, self.prefix + 'hatch_position', self.hatch_pos_cb, 10)
        self.ext_rot_sub = self.create_subscription(Float64, self.prefix + 'ext_wheel_position', self.ext_wheel_pos_cb, 10)
        self.ext_rot_h_sub = self.create_subscription(Float64, self.prefix + 'ext_handle_position', self.ext_handle_pos_cb, 10)
        self.int_rot_sub = self.create_subscription(Float64, self.prefix + 'int_wheel_position', self.int_wheel_pos_cb, 10)
        self.int_rot_h_sub = self.create_subscription(Float64, self.prefix + 'int_handle_position', self.int_handle_pos_cb, 10)

        # create the timer for joint state publisher callback
        timer_period_sec = 0.5 # unit: seconds
        self.timer = self.create_timer(timer_period_sec, self.hatch4040_joint_state_cb)

        # initialization of all the latch component positions
        self.hatch_position = 0.0
        self.ext_wheel_position = 0.0
        self.ext_handle_position = 0.0
        self.int_wheel_position = 0.0
        self.int_handle_position = 0.0

        self.lock = threading.Lock() 

        # ranges for all joints
        self.hatch_min = 0
        self.hatch_max = math.radians(95.0)
        self.ext_wheel_min = 0
        self.ext_wheel_max = 2*math.pi
        self.ext_handle_min = 0
        self.ext_handle_max = math.pi/2
        self.int_wheel_min = 0
        self.int_wheel_max = 2*math.pi
        self.int_handle_min = 0
        self.int_handle_max = math.pi/2

        # update the joint states
        self._hatch_joint_states = {self.prefix + 'hatch_outer_frame_face_joint': copy.deepcopy(self.hatch_position), self.prefix + 'external_rotary_joint': copy.deepcopy(self.ext_wheel_position), \
            self.prefix + 'external_rotary_handle_joint': copy.deepcopy(self.ext_handle_position), self.prefix + 'internal_rotary_joint': copy.deepcopy(self.int_wheel_position), self.prefix + 'internal_rotary_handle_joint': copy.deepcopy(self.int_handle_position)}           

        self.hatch4040_joint_state_cb() # going ahead and starting


    def hatch4040_joint_state_cb(self):
        """Publisher for the manager which publishes the joint state info. 
        """
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = [self.prefix + 'hatch_outer_frame_face_joint', self.prefix + 'external_rotary_joint', self.prefix + 'external_rotary_handle_joint', self.prefix + 'internal_rotary_joint', self.prefix + 'internal_rotary_handle_joint']
        with self.lock: 
            msg.position = [self.hatch_position, self.ext_wheel_position, self.ext_handle_position, self.int_wheel_position, self.int_handle_position] # fill with float64 list
            msg.velocity = [0.0, 0.0, 0.0, 0.0, 0.0]
            msg.effort = [0.0, 0.0, 0.0, 0.0, 0.0]
            for x, joint_name in enumerate(msg.name):
                if joint_name in self._hatch_joint_states.keys():
                    self._hatch_joint_states[joint_name] = copy.deepcopy(msg.position[x]) # go ahead and update

        self.publisher_.publish(msg)

        self.get_logger().debug('This is the hatch joint state message: {}'.format(msg)) # this will fill in the string with formated msg data

    def hatch_pos_cb(self, msg: Float64):
        """ callback for the hatch door position

        Args:
            msg (Float64): This message transmits the revolute joint angle (in radians) 
        """
        self.get_logger().debug('This is the angle of the hatch joint: {}'.format(msg))
        if self.hatch_min <= msg.data and msg.data  <= self.hatch_max:
            with self.lock: 
                self.hatch_position = msg.data
        elif self.hatch_min >= msg.data:
            with self.lock:
                self.hatch_position = copy.deepcopy(self.hatch_min)
        elif self.hatch_max <= msg.data:
            with self.lock:
                self.hatch_position = copy.deepcopy(self.hatch_max)
        else:
            raise Exception("hatch joint angle out of range")
    
    def ext_wheel_pos_cb(self, msg: Float64):
        """ callback for internal handle position

        Args:
            msg (Float64): This message transmits the revolute joint angle (in radians) 
        """
        self.get_logger().debug('This is the value of the external wheel joint: {}'.format(msg))
        if self.ext_wheel_min <= msg.data and msg.data <= self.ext_wheel_max:
            with self.lock: 
                self.ext_wheel_position = msg.data
        elif self.ext_wheel_min >= msg.data:
            with self.lock:
                self.ext_wheel_position = copy.deepcopy(self.ext_wheel_min)
        elif self.ext_wheel_max <= msg.data:
            with self.lock:
                self.ext_wheel_position = copy.deepcopy(self.ext_wheel_max)                
        else:
            raise Exception("external wheel value out of range")
    
    def ext_handle_pos_cb(self, msg: Float64):
        """ callback for external handle position

        Args:
            msg (Float64): This message transmits the revolute joint angle (in radians) 
        """
        self.get_logger().debug('This is the value of the external handle joint: {}'.format(msg))
        if self.ext_handle_min <= msg.data and msg.data <= self.ext_handle_max:
            with self.lock: 
                self.ext_handle_position = msg.data
        elif self.ext_handle_min >= msg.data:
            with self.lock:
                self.ext_handle_position = copy.deepcopy(self.ext_handle_min)
        elif self.ext_handle_max <= msg.data:
            with self.lock:
                self.ext_handle_position = copy.deepcopy(self.ext_handle_max)                 
        else:
            raise Exception("external handle value out of range")

    def int_wheel_pos_cb(self, msg: Float64):
        """ callback for internal wheel position

        Args:
            msg (Float64): This message transmits the revolute joint angle (in radians) 
        """
        self.get_logger().debug('This is the angle of the internal wheel joint: {}'.format(msg))
        if self.int_wheel_min <= msg.data and msg.data <= self.int_wheel_max:
            with self.lock: 
                self.int_wheel_position = msg.data
        elif self.int_wheel_min >= msg.data:
            with self.lock:
                self.int_wheel_position = copy.deepcopy(self.int_wheel_min)
        elif self.int_wheel_max <= msg.data:
            with self.lock:
                self.int_wheel_position = copy.deepcopy(self.int_wheel_max)                  
        else:
            raise Exception("internal wheel joint angle out of range")
        
    def int_handle_pos_cb(self, msg: Float64):
        """ callback for internal handle position

        Args:
            msg (Float64): This message transmits the revolute joint angle (in radians) 
        """
        self.get_logger().debug('This is the angle of the inernal handle joint: {}'.format(msg))
        if self.int_handle_min <= msg.data and msg.data <= self.int_handle_max:
            with self.lock: 
                self.int_handle_position = msg.data
        elif self.int_handle_min >= msg.data:
            with self.lock:
                self.int_handle_position = copy.deepcopy(self.int_handle_min)
        elif self.int_handle_max <= msg.data:
            with self.lock:
                self.int_handle_position = copy.deepcopy(self.int_handle_max)                     
        else:
            raise Exception("internal handle joint angle out of range")
        
def main(args=None):
    rclpy.init(args=args)

    hatch_manager = Hatch4040Manager()

    rclpy.spin(hatch_manager)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    hatch_manager.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()        