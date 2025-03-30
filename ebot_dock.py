#!/usr/bin/env python3
'''
# Team ID:          < LB#1136 >
# Theme:            < Cosmo Logistic >
# Author List:      < Krishnapranav >
# Filename:         < ebot_dock.py >
# Functions:        < dock_control_callback, orientation_callback, ultra_sonic_callback,
#                     dock_the_bot, undock_the_bot, main >
# Global variables: < None>

PURPOSE OF THIS NODE : Handles the linear and angular docking and undocking of the ebot with different stations.

'''
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from tf_transformations import euler_from_quaternion
from std_msgs.msg import Float32MultiArray
from ebot_docking.srv import DockSw  # Import custom service message
import time

# Define a class for your ROS2 node
class MyRobotDockingController(Node):

    def __init__(self):
        # Initialize the ROS2 node with a unique name
        super().__init__('dock control service')

        # Create a callback group for managing callbacks
        self.callback_group = ReentrantCallbackGroup()

        # Subscribe to odometry data for robot pose information
        self.orientation_sub = self.create_subscription(Odometry, '/odometry/filtered', self.orientation_callback, 10)

        # Subscribe to ultrasonic sensor data for distance measurements
        self.ultra_sub = self.create_subscription(Float32MultiArray, 'ultrasonic_sensor_std_float', self.ultra_sonic_callback, 10)

        # Create aservice for controlling docking behavior
        self.dock_control_srv = self.create_service(DockSw, 'dock_control', self.dock_control_callback, callback_group=self.callback_group)

        # Create a publisher for sending velocity commands to the robot
        self.cmd_vel_pub_ = self.create_publisher(Twist, '/cmd_vel', 10)

        # Initialize all  flags and parameters 
        self.angular_dock_complete = False # set to true when angle docking is complete
        self.dock_aligned = False # set to true when docking operation is complete
        self.undocking = False # set to true when undocking is going on

        # initialize the robot positions
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        self.dock_angle = 0.0
        self.dock_distance = 0.0

        self.get_logger().info("DOCKING SERVICE STARTED")

    '''
    Purpose:
    ---
    function which handles service requests to start docking. Sets robot states based on input parameters.

    Input Arguments:
    ---
    request : [DockSw.Request]
    Contains docking request parameters: linear_dock, orientation_dock, and orientation.

    response : [DockSw.Response]
    Will contain the result of the docking operation: success and message.

    Returns:
    ---
    response : [DockSw.Response]
    Indicates the success or failure of the docking operation.

    Example call:
    ---
    Called automatically when a service request is received by the 'dock_control' service.
    '''
    def dock_control_callback(self, request : DockSw.Request, response : DockSw.Response):

        # catch the request from the client
        linear_dock = request.linear_dock
        angular_dock = request.orientation_dock
        undocking = request.undocking
        self.dock_angle = request.orientation
        self.dock_distance = request.distance
        
        # in case of docking
        if linear_dock == True and angular_dock == True:
            self.dock_aligned = False
        
        # in case of un-docking
        if undocking == True:
            self.undocking = True
        
        # Wait until the robot is undocking
        while self.undocking:
            self.undock_the_bot()

        # Wait until the robot is aligned for docking
        while not self.dock_aligned:
            self.dock_the_bot()
            time.sleep(0.1)

        # Set the service response indicating success
        response.success = True
        response.message = "Docking control finished"

        return response

    '''
    Purpose:
    ---
    callback function which gets the odometry data and updates the robot's pose (position and orientation).

    Input Arguments:
    ---
    msg : [Odometry]
    Odometry message containing position and orientation of the robot.

    Returns:
    ---
    None

    Example call:
    ---
    This function is automatically called whenever an odometry message is received.
    '''
    def orientation_callback(self, msg : Odometry):
        # Extract and update robot pose information from odometry message
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        quaternion_array = msg.pose.pose.orientation

        orientation_list = [quaternion_array.x, quaternion_array.y, quaternion_array.z, quaternion_array.w]
        _, _, yaw = euler_from_quaternion(orientation_list)

        self.robot_yaw = yaw
       
    '''
    Purpose:
    ---
    functions get range data from the left and right ultrasonic sensor.

    Input Arguments:
    ---
    msg : [Range]
    Range message containing the distance measured by the sensor.

    Returns:
    ---
    None

    Example call:
    ---
    Automatically called when a message from the '/ultrasonic_rl/scan' topic is received.
    '''
  
    # callback for ultrasonic subscription
    def ultra_sonic_callback(self,msg : Float32MultiArray):
        self.usrleft_value= msg.data[4]
        self.usrright_value = msg.data[5]

    '''
    Purpose:
    ---
    The main control loop for aligning the robot's orientation and distance during docking. Executes in a timed loop.

    Input Arguments:
    ---
    None

    Returns:
    ---
    None

    Example call:
    ---
    This function is automatically called at the specified interval set by create_timer().
    '''
    def dock_the_bot(self):
        vel_msg = Twist()

        # Proportional gain for linear and angular control
        Kp_angular = 3.0
        Kp_linear = -0.3

        # Target orientation
        target_angle = self.dock_angle

        # Align the orientation of the robot
        if self.angular_dock_complete == False:

            # compute the angle error
            angle_error = target_angle - self.robot_yaw
            vel_msg.angular.z = Kp_angular * angle_error

            # limit the angular velocity.
            if abs(vel_msg.angular.z) > 0.5:
                vel_msg.angular.z = 0.5

            # if the orientation is less than the threshold angle error, angular docking is complete
            if abs(angle_error) < 0.05:
                self.angular_dock_complete = True

        # Once orientation is aligned, align its distance from the conveyor belt
        else:

            # get the range value as the mean of right and left ultrasonic sensor
            dis = (self.usrleft_value + self.usrright_value) / 2
            vel_msg.linear.x = Kp_linear * dis

            # When docking is complete, stop the robot and reset the variables
            if dis < self.dock_distance:
                vel_msg.linear.x = 0.0
                vel_msg.angular.z = 0.0

                # set the flags as True/False
                self.dock_aligned = True
                self.angular_dock_complete = False

        self.cmd_vel_pub_.publish(vel_msg)
    '''
    Purpose:
    ---
    The main control loop for undocking. The robot needs to come out of the docked state in order to navigate to the other pose. Executes in a timed loop.

    Input Arguments:
    ---
    None

    Returns:
    ---
    None

    Example call:
    ---
    This function is automatically called at the specified interval set by create_timer().
    '''
    def undock_the_bot(self):
        vel_msg = Twist()

        # Proportional gain for linear and angular control
        Kp_linear = 0.3

        # get the range value as the mean of right and left ultrasonic sensor
        dis = (self.usrleft_value + self.usrright_value) / 2

        vel_msg.linear.x = Kp_linear * dis

        # Loops till the distance is more than the specified dock distance in request
        if dis > self.dock_distance:
            vel_msg.linear.x = 0.0
            vel_msg.angular.z = 0.0

            # end undocking process
            self.undocking = False

        self.cmd_vel_pub_.publish(vel_msg)

'''
Purpose:
---
Spins the node in seperate threads.
'''
def main(args=None):
    rclpy.init(args=args)
    my_robot_docking_controller = MyRobotDockingController()
    executor = MultiThreadedExecutor()
    executor.add_node(my_robot_docking_controller)
    executor.spin()
    my_robot_docking_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
