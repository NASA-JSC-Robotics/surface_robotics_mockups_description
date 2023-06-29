import sys
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.callback_groups import ReentrantCallbackGroup
from geometry_msgs.msg import PoseStamped, TransformStamped, Quaternion
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
import copy, math, threading

class Hatch4040Manager(Node):
    def __init__(self):
        super().__init__('hatch4040_manager')

        self.declare_parameter('robot_description', '') # putting None in as a default causes deprecation warning
        
        robot_description = self.get_parameter('robot_description').value
        if not robot_description:
            self.get_logger().error('Passing in the robot description is required for this node ({name}) to operator'.format(name=self.get_name())) 
            return

        # initialize the transform broadcaster
        self._tf_publisher = StaticTransformBroadcaster(self)
        self.tf_name = 'TODO INTIAL FRAME'

        # intialize tf timer and service callback
        tf_timer_period_sec = 0.5 # unit: seconds
        self.tf_timer = self.create_timer(tf_timer_period_sec, self.make_transform)

        self.pose_stamped = PoseStamped() # will store timestamped data about the source frame
        # initialize the header
        self.pose_stamped.header.stamp = self.get_clock().now().to_msg() # initialize to the current time
        self.pose_stamped.header.frame_id = 'world' 

        self.pose_stamped.pose.position.x = -0.002
        self.pose_stamped.pose.position.y = 0.868
        self.pose_stamped.pose.position.z = 0.257

        self.pose_stamped.pose.orientation.x = -0.00
        self.pose_stamped.pose.orientation.y = -0.022
        self.pose_stamped.pose.orientation.z = 1.0
        self.pose_stamped.pose.orientation.w = -0.004

        # callback groups
        self.position_cb_group = MutuallyExclusiveCallbackGroup() # the group for all of the position callbacks, which has them all go one at a time

        # create the joint state publisher
        self.publisher_ = self.create_publisher(JointState, 'hatch4040_joint_states', 10, callback_group = self.pub_cb_group)
        
        # create subscriptions's to revolute/prismatic joints
        self.hatch_sub = self.create_subscription(Float64, 'hatch_position', self.door_pos_cb, 10, callback_group = self.position_cb_group)
        self.ext_rot_sub = self.create_subscription(Float64, 'ext_wheel_position', self.left_latch__pos_cb, 10, callback_group = self.position_cb_group)
        self.ext_rot_h_sub = self.create_subscription(Float64, 'ext_handle_position', self.right_latch_pos_cb, 10, callback_group = self.position_cb_group)
        self.int_rot_sub = self.create_subscription(Float64, 'int_wheel_position', self.latch_lock_pos_cb, 10, callback_group = self.position_cb_group)
        self.int_rot_h_sub = self.create_subscription(Float64, 'int_handle_position', self.right_latch_pos_cb, 10, callback_group = self.position_cb_group)

        # initialize the thread lock
        self.lock = threading.Lock() # the lock is used to freeze data inputs while the joint state pubisher callback is operating

        # create the timer for joint state publisher callback
        timer_period_sec = 0.5 # unit: seconds
        self.timer = self.create_timer(timer_period_sec, self.hatch4040_joint_state_cb, callback_group = self.pub_cb_group)

        # initialization of all the latch component positions
        self.hatch_starting_position = 0.0

        # ranges for all joints
        self.hatch_min = 0
        self.hatch_max = math.pi
        self.ext_wheel_min = 0
        self.ext_wheel_max = 2*math.pi
        self.ext_handle_min = 0
        self.ext_handle_max = math.pi/2
        self.int_wheel_min = 0
        self.int_wheel_max = 2*math.pi
        self.int_handle_min = 0
        self.int_handle_max = math.pi/2
        
        # set two conditions for the initialization of the door (open vs closed)
        door_initialized_open = self.declare_parameter("door_initialized_open", True) # "door_initialized_open" is the parameter a user can type into the terminal to initialize the door open or closed
        if self.get_parameter("door_initialized_open").value: # open
            self.door_position = -math.pi/2
        else: # closed
            self.door_position = 0.0

        # update the joint states
        self._hatch_joint_states = {'door_to_frame_joint': copy.deepcopy(self.door_position), 'housing_to_right_latch_pull': copy.deepcopy(self.right_latch_position), \
            'housing_to_left_latch_pull': copy.deepcopy(self.left_latch_position), 'housing_to_latch_lock': copy.deepcopy(self.latch_lock_position)}
        self._co_change_position_tolerance_radians = 0.05             

        # go ahead and publish first joint_state. Get TF working (these are just in case since we don't know the executor's timing trigger for these things)
        self.hatch4040_joint_state_cb()
        # go ahead and publish first transform
        self.make_transform()


    def make_transform(self):
        """ function to make the tf transform stamped message and pass it to the broadcaster 
        """
        transform = TransformStamped()

        # set the parameters of the transform to those updated globally
        transform.header.stamp = self.pose_stamped.header.stamp 
        transform.header.frame_id = self.pose_stamped.header.frame_id 
        transform.child_frame_id = self.tf_name
        transform.transform.translation.x = self.pose_stamped.pose.position.x # note: had to break down the xyz because pose stamped uses a Point while transform stamped uses a vector3
        transform.transform.translation.y = self.pose_stamped.pose.position.y
        transform.transform.translation.z = self.pose_stamped.pose.position.z
        transform.transform.rotation = self.pose_stamped.pose.orientation # this is a quaternion

        # publish the transformation to the static transform broadcaster
        self._tf_publisher.sendTransform(transform)
       

    def hatch4040_joint_state_cb(self):
        """Publisher for the manager which publishes the joint state info. 
        """
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['hatch_4040_frame_face_joint', 'housing_to_left_latch_pull', 'housing_to_right_latch_pull', 'housing_to_latch_lock']
        co_update_required = False # update set to false until there is an established diff with the prev joint state
        with self.lock: 
            msg.position = [self.door_position, self.left_latch_position, self.right_latch_position, self.latch_lock_position] # fill with float64 list
            msg.velocity = [] # fill with float64 list
            msg.effort = [] # fill with float64 list
            for x, joint_name in enumerate(msg.name):
                if joint_name in self._hatch_joint_states.keys():
                    # TODO: consider reducing to only the links with collision objects
                    # TODO: consider ops. Is it possible we could be moving the door and the update rate is so fast that this diff is not met? 
                    if abs(msg.position[x] - self._hatch_joint_states[joint_name]) > self._co_change_position_tolerance_radians: # if things have moved, update the collision objects. Note: the > __ : indicates that the 
                        co_update_required = True
                    self._hatch_joint_states[joint_name] = copy.deepcopy(msg.position[x]) # go ahead and update

        self.publisher_.publish(msg)


        self.get_logger().debug('This is the hatch joint state message: {}'.format(msg)) # this will fill in the string with formated msg data

    def door_pos_cb(self, msg: Float64):
        """ callback for the hatch door position

        Args:
            msg (Float64): This message transmits the revolute joint angle (in radians) 
        """
        self.get_logger().debug('This is the angle of the door joint: {}'.format(msg))
        if self.door_min <= msg.data and msg.data  <= self.door_max:
            with self.lock: 
                self.door_position = msg.data
        else:
            raise Exception("door joint angle out of range")
    
    def left_latch__pos_cb(self, msg: Float64):
        """ callback for merlin left latch position

        Args:
            msg (Float64): This message transmists the prismatic joint value (in meters) for the connection between the left latch pull and housing
        """
        self.get_logger().debug('This is the value of the left latch pull joint: {}'.format(msg))
        if self.l_latch_min <= msg.data and msg.data <= self.l_latch_max:
            with self.lock: 
                self.left_latch_position = msg.data
        else:
            raise Exception("left latch pull value out of range")
    
    def right_latch_pos_cb(self, msg: Float64):
        """ callback for merlin right latch position

        Args:
            msg (Float64): This message transmists the prismatic joint value (in meters) for the connection between the right latch pull and housing
        """
        self.get_logger().debug('This is the value of the right latch pull joint: {}'.format(msg))
        if self.r_latch_min <= msg.data and msg.data <= self.r_latch_max:
            with self.lock: 
                self.right_latch_position = msg.data
        else:
            raise Exception("right latch pull value out of range")

    def latch_lock_pos_cb(self, msg: Float64):
        """ callback for latch lock position

        Args:
            msg (Float64): This message transmits the revolute joint angle (in radians) between the latch lock and housing
        """
        self.get_logger().debug('This is the angle of the latch lock joint: {}'.format(msg))
        if self.lock_min <= msg.data and msg.data <= self.lock_max:
            with self.lock: 
                self.latch_lock_position = msg.data
        else:
            raise Exception("latch lock joint angle out of range")
        
    def latch_lock_pos_cb(self, msg: Float64):
        """ callback for latch lock position

        Args:
            msg (Float64): This message transmits the revolute joint angle (in radians) between the latch lock and housing
        """
        self.get_logger().debug('This is the angle of the latch lock joint: {}'.format(msg))
        if self.lock_min <= msg.data and msg.data <= self.lock_max:
            with self.lock: 
                self.latch_lock_position = msg.data
        else:
            raise Exception("latch lock joint angle out of range")