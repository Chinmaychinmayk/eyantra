#! /usr/bin/env python3

'''
# Team ID:          < LB#1136 >
# Theme:            < Cosmo Logistic >
# Author List:      < Krishnapranav >
# Filename:         < ebot_nav.py >
# Functions:        < dock, receive_payload, drop_payload, result_callback, goal_checker, main >
# Global variables: < None>

PURPOSE OF THIS NODE : Handles the entire operation. Navigates the ebot to drop pose, requests the payload, navigates to conveyor
                        and drops the payload there.
'''

from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.node import Node
from tf_transformations import quaternion_from_euler, euler_from_quaternion
# from payload_service.srv import PayloadSW
from ebot_docking.srv import PayloadSW
from ebot_docking.srv import DockSw  
import time
from usb_servo.srv import ServoSw
from std_srvs.srv import Trigger

class EbotNav(Node):
    def __init__(self):
        super().__init__('transform_lookup_node')
        self.navigator = BasicNavigator()

        # clients initialization
        self.drop_payload_client = self.create_client(ServoSw, '/toggle_usb_servo')
        self.payload_client = self.create_client(PayloadSW, '/pass_payload')
        self.dock_client = self.create_client(DockSw, '/dock_control')

        # CONVEYOR 1 POSITION
        self.conv1_pose = PoseStamped()
        self.conv1_pose.header.frame_id = 'map'
        self.conv1_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        self.conv1_pose.pose.position.x = 3.13
        self.conv1_pose.pose.position.y = 1.9
        quat = quaternion_from_euler(0.0, 0.0, 0.0)  # 90 degrees in radians

        self.conv1_pose.pose.orientation.x = quat[0]
        self.conv1_pose.pose.orientation.y = quat[1]
        self.conv1_pose.pose.orientation.z = quat[2]
        self.conv1_pose.pose.orientation.w = quat[3]

        # CONVEYOR 2 POSITION
        self.conv2_pose = PoseStamped()
        self.conv2_pose.header.frame_id = 'map'
        self.conv2_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        self.conv2_pose.pose.position.x = 3.0 # 2
        self.conv2_pose.pose.position.y = -1.2 #1.25
        quat = quaternion_from_euler(0.0, 0.0, 0.0)  # Convert orientation from Euler to Quaternion

        self.conv2_pose.pose.orientation.x = quat[0]
        self.conv2_pose.pose.orientation.y = quat[1]
        self.conv2_pose.pose.orientation.z = quat[2]
        self.conv2_pose.pose.orientation.w = quat[3]

        # ARM POSITION
        self.arm_pose = PoseStamped()
        self.arm_pose.header.frame_id = 'map'
        self.arm_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        self.arm_pose.pose.position.x = 2.6
        self.arm_pose.pose.position.y = -2.65
        quat = quaternion_from_euler(0.0, 0.0, 0.0)  # Convert orientation from Euler to Quaternion

        self.arm_pose.pose.orientation.x = quat[0]
        self.arm_pose.pose.orientation.y = quat[1]
        self.arm_pose.pose.orientation.z = quat[2]
        self.arm_pose.pose.orientation.w = quat[3]

        self.dock_angle_ = 2.85

        # wait for the arm manipulation service
        while not self.payload_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for service...')

        self.get_logger().info("NAVIGATING EBOT !")

    '''
    Purpose:
    ---
    Function to send a request to the DOCKING service (/dock_control) asynchronously.
    It is called when the robot needs to align itself perfectly, e.g., dock to the conveyor belt station
    to drop the box.

    Input Arguments:
    ---
    dock_angle : float
    The target docking angle in radians.

    Returns:
    ---
    None

    Example call:
    ---
    dock(dock_angle=1.57)
    '''
    def dock(self, dock_angle):
        self.get_logger().info("REQUESTING TO DOCK")

        req = DockSw.Request()

        # Send the request to dock both position and orientation of the robot
        req.orientation_dock = True
        req.linear_dock = True
        req.undocking = False
        req.orientation = dock_angle

        # set distance close to the wall
        req.distance = 30.0

        future = self.dock_client.call_async(req)
        
        # Spin until the service is completed
        while not future.done():
            rclpy.spin_once(self)
    
    '''
    Purpose:
    ---
    Function to send a request to the DOCKING service (/dock_control) asynchronously.
    It is called when the robot needs to un-dock or come out of the docked position to be able to navigate
    to the next pose

    Input Arguments:
    ---
    None

    Returns:
    ---
    None

    Example call:
    undock()
    '''
    def undock(self):
        self.get_logger().info("REQUESTING TO DOCK")

        req = DockSw.Request()

        # Send the request to un-dock the robot
        req.orientation_dock = False
        req.linear_dock = False
        req.undocking = True

        # set distance, this means dock till the distance is 200
        req.distance = 200.0

        future = self.dock_client.call_async(req)
        
        # Spin until the service is completed
        while not future.done():
            rclpy.spin_once(self)
    '''
    Purpose:
    ---
    Function to send a request to the PASSING service (/pass_payload) asynchronously.
    It is called when the robot has reached the drop position and is ready to receive the payload which needs
    to be transported to the conveyor belt.

    Input Arguments:
    ---
    None
    
    Returns:
    ---
    None
    
    Example call:
    ---
    receive_payload()
    '''
    def receive_payload(self):
        self.get_logger().info("REQUESTING PAYLOAD")

        req = PayloadSW.Request()
        req.drop = True  # Specify that the robot is receiving the payload
        req.pick = False
        
        future = self.payload_client.call_async(req)
        future.add_done_callback(self.result_callback)
        
        while not future.done():
            rclpy.spin_once(self)
    '''
    Purpose:
    ---
    Function to send a request to the PASSING service (/pass_payload) asynchronously.
    It is called when the robot has reached the drop position and is ready to receive the payload which needs
    to be transported to the conveyor belt.

    Input Arguments:
    ---
    None
    
    Returns:
    ---
    None
    
    Example call:
    ---
    receive_payload()
    '''
    def pick_payload(self):
        self.get_logger().info("INITIALIZE PASS")

        req = PayloadSW.Request()
        req.drop = False  # Specify that the robot is receiving the payload
        req.pick = True

        future = self.payload_client.call_async(req)
        future.add_done_callback(self.result_callback)
        
    '''
    Purpose:
    ---
    Function to send a request to the DROP PAYLOAD service (/payload_sw) asynchronously.
    It is called when the robot has reached the conveyor belt station and is ready to drop the payload in the
    conveyor belt.

    Input Arguments:
    ---
    None

    Returns:
    ---
    None

    Example call:
    ---
    drop_payload()
    '''
    def drop_payload(self, state):
        self.get_logger().info("DROPPING PAYLOAD")

        request = ServoSw.Request()
        request.servostate = state

        future = self.drop_payload_client.call_async(req)
        future.add_done_callback(self.result_callback)
        
        while not future.done():
            rclpy.spin_once(self)
    '''
    Purpose:
    ---
    Function which is called whenever a server (e.g., dock server) returns a result. This result is stored
    in a variable and used at required places.

    Input Arguments:
    ---
    future : rclpy Future
    The future object returned by the service call, containing the result.

    Returns:
    ---
    None

    Example call:
    ---
    Added as a done callback, the client keeps expecting a future object once the request has been sent. When it receives a
    result this callback function is called.
    '''
    def result_callback(self, future):
        try:
            self.res = future.result()
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')
    '''
    Purpose:
    ---
    Function to check if the goal position has been reached or not. Runs a loop until the nav2 navigation server confirms that the robot
    has reached the goal position.

    Input Arguments:
    ---
    destination : str
    The name of the target location for logging purposes.

    Returns:
    ---
    None

    Example call:
    ---
    node.goal_checker("conveyor 2")
    '''
    def goal_checker(self, destination):

        # Loop until the navigation task is not succeeded.
        while not self.navigator.isTaskComplete():
            self.get_logger().info(f'NAVIGATING TO POSE {destination}')

        # Check the status of the navigation
        result = self.navigator.getResult()

        if result == TaskResult.SUCCEEDED:
            self.get_logger().info('Pose reached successfully!')
        print(self.navigator.getFeedback())

'''
Purpose:
---
The whole mission is carried out by this function: it navigates the robot and requests services to dock the robot, receive and drop the payload.
It calls the nav2 navigator to carry out navigation and spins the node when required.
'''
def main():
    rclpy.init()
    node = EbotNav()
    
    for i in range(3):
        # request the arm to pick a payload and wait at the pick position
        node.pick_payload()

        # navigate to the arm pose
        node.navigator.goToPose(node.arm_pose)
        node.goal_checker("drop_pose")

        # Dock to the station 
        node.dock(dock_angle= node.dock_angle_)
        rclpy.spin_once(node)

        # wait for the arm to reach the pick position if the ebot reaches before that
        while not node.res.success:
            node.get_logger().info("WAITING FOR ARM TO REACH PICK POSITION")
        
        # request the payload to be dropped
        node.receive_payload()

        box_name = node.res.message

        # if the box is numbered even : go to conveyor 2 eg. box1, box3, we are checking the last element of the string
        if int(list(box_name)[-1]) % 2:
            # the box is odd numbered so conveyor 2
            node.navigator.goToPose(node.conv2_pose)
            node.goal_checker("conveyor 2")

        else:
            # the box is even numbered so conveyor 1
            node.navigator.goToPose(node.conv1_pose)
            node.goal_checker("conveyor 1")

        # dock to the conveyor
        node.dock(dock_angle= node.dock_angle_)
        rclpy.spin_once(node)

        # request the servo service to drop the payload
        node.drop_payload(True)
        rclpy.spin_once(node)

    node.get_logger().info("MISSION SUCCESS !")

    # Shutdown the node once process completed
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
