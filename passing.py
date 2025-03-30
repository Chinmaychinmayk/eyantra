#!/usr/bin/env python3

'''
# Team ID:          < LB#1136 >
# Theme:            < Cosmo Logistic >
# Author List:      < Krishnapranav >
# Filename:         < passing.py >
# Functions:        < pass_payload, wrench_callback, attach_payload, detach_payload, 
#                     get_payload_pose, result_callback, move_to_or,
#                     move_to_pose, counter_delay, move_arm, lookup_transform,
#                     compute_distance, main  >
# Global variables: < None>

PURPOSE OF THIS NODE : It handles the passing operation by manipulating th ur5 robotic arm. It is supposed to pick up the payload
                       when the robot is navigating to the drop position. It places the payload on the ebot when it is requested to do so.
'''

import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from tf_transformations import euler_from_quaternion
from geometry_msgs.msg import TwistStamped
from pymoveit2.robots import ur5
from scipy.spatial.transform import Rotation as R
from std_msgs.msg import Float64
import numpy as np
from ebot_docking.srv import ArucoSW ,PayloadSW

from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
import time
from ur_msgs.srv import SetIO

class TransformLookupNode(Node):
    def __init__(self):
        super().__init__('transform_lookup_node')

        '''
        TRANSFORM
        '''
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_timer = self.create_timer(0.02, self.lookup_transform)

        '''
        PICK AND DROP INITIALIZATION
        '''
        # publisher to publish velocities for the ee of arm
        self.ee_vel_pub = self.create_publisher(TwistStamped, "/ServoCmdVel", 10)

        # clients linked to services to detach, attach and get pose of payload
        self.get_pickndrop_position = self.create_client(ArucoSW,'/payload_pos')
        self.arm_gripper_control = self.create_client(SetIO, '/io_and_status_controller/set_io')

        # subscribe from net wrench topic
        self.wrench_sub = self.create_subscription(Float64, '/net_wrench', self.wrench_callback, 10)

        # initialize end effector position and orientation as a random arbitary val ue
        self.ee_x = 0.1
        self.ee_y = 0.1
        self.ee_z = 0.1
        self.ee_orx, self.ee_ory, self.ee_orz = 0.0,0.0,0.0

        # initialization for the state machine
        self.state = 0 # the passing operation is conducted sequentially as per the state
        self.prev_state = 0
        self.net_wrench = 0.0
        self.cnt = 0.0
        self.payload_dimension = 0.22 # dimension of the payload. Set by user.

        '''
        PASSING SERVICE
        '''
        # initialize the passing server and specify callback group
        self.callback_group = ReentrantCallbackGroup()
        self.payload_server = self.create_service(PayloadSW, '/pass_payload', self.pass_payload, callback_group=self.callback_group)
        self.drop_payload_ = False
        self.pick_n_hold = False
        self.get_logger().info("PASSING SERVICE STARTED !")
    
    '''
    Purpose:
    ---
    functions get force data from net wrench topic

    Input Arguments:
    ---
    msg : [Float64]
    Float64 message containing the force experienced by the gripper

    Returns:
    ---
    None

    Example call:
    ---
    Automatically called when a message from the '/net_wrench' topic is received.
    '''
    def wrench_callback(self, msg :Float64):
        self.get_logger().info(f"{self.net_wrench}")
        self.net_wrench = msg.data

    '''
    Purpose:
    ---
    Function called whenever a request for /pass_payload service is requested. It runs a loop which moves the robotic
    arm to pick and drop the payload.

    Input Arguments:
    ---
    request : [PayloadSW.Request]
    Request containing the receive and drop flags.

    response : [PayloadSW.Response]
    Response containing success status and message.

    Returns:
    ---
    response : [PayloadSW.Response]
    Indicates success or failure of the payload passing operation.

    Example Call:
    ---
    This function is a ROS2 service callback. It is triggered when a service request is made to the "pass_payload" service.
    '''

    def pass_payload(self, request : PayloadSW.Request, response : PayloadSW.Response):

        pick = request.pick
        drop = request.drop

        self.get_logger().info(f"PASSING REQUEST RECEIVED {pick} {drop}")

        if pick == False and drop == True:
            self.drop_payload_ = True

        if pick == True and drop == False:
            self.pick_n_hold = True

        while self.pick_n_hold:

            # function move_arm is called to publish the appropriate velocities to ee, grip and drop the payload
            self.pick_n_hold_payload()
            self.get_logger().info(f"PICK AND HOLD CALLED {self.state}")
            time.sleep(0.02)

        # Run the loop until the passing operation is complete
        while self.drop_payload_:
            # function move_arm is called to publish the appropriate velocities to ee, grip and drop the payload
            self.get_logger().info(f"DROP CALLED {self.state}")
            self.drop_payload()
            time.sleep(0.02)

        # Once the loop is exited the passing operation is complete, and response is returned
        response.success = True
        response.message = self.payload_id[0]

        return response
    '''
    Purpose:
    ---
    Function to send a request to the arm gripper to activate/deactivate the electro-magnet
    It is called when the arm needs to grip the payload.

    Input Arguments:
    ---
    state : bool
    Whether to activatte or deactivate the arm gripper

    Returns:
    ---
    None

    Example Call:
    ---
    To grip the payload : attach_payload(True)
    '''
    def arm_gripper(self, state):
        req = SetIO.Request()
        req.fun = 1
        req.pin = 16
        req.state = float(state)
        self.arm_gripper_control.call_async(req)

        # Call the service asynchronously
        future = self.arm_gripper_control.call_async(req)
        future.add_done_callback(self.result_callback)

    '''
    Purpose:
    ---
    Function to send a request to the Detect Aruco service (/payload_pos) asynchronously.
    It is called when the arm needs the pick and drop position of the payload.

    (This is a custom service created in order to make the function discrete and robust. The client requests the pick and drop position
    for the payload, which is computed by the detect_aruco server and returns the response.)

    Input Arguments:
    ---
    None

    Returns:
    ---
    None

    Example Call:
    ---
    get_payload_pose()
    '''
    def get_payload_pose(self):
        # Create a request for the ArucoSW service, to get the pick and drop coordinates.
        self.get_logger().info("REQESTING PAYLOAD DROP POSE !")

        req = ArucoSW.Request()
        req.get_position = True

        # Call the service asynchronously
        future = self.get_pickndrop_position.call_async(req)
        future.add_done_callback(self.result_callback)
    
    '''
    Purpose:
    ---
    Handles the result of a service call and processes the response.

    Input Arguments:
    ---
    future : rclpy.Future
    Future object containing the result of the service call.

    Returns:
    ---
    None

    Example Call:
    ---
    Used as a callback function for asynchronous service calls. 
    '''

    def result_callback(self, future):
        try:
            self.response = future.result()
            # self.get_logger().info(f'Service response: {self.response}')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')
 
    '''
    Purpose:
    ---
    Calculates and applies the required angular velocities for the end effector to reach a target orientation.

    Input Arguments:
    ---
    goal_or : tuple
    Target orientation as (roll, pitch, yaw).

    Returns:
    ---
    tuple
    Angular velocities (wx, wy, wz).

    Example Call:
    ---
    wx, wy, wz  = compute_angular_velocity((1.57, 0, 0))
    '''
    def move_to_or(self, goal_or):
        
        # compute orientation error about x, y and z axes
        wx = (goal_or[0] - self.ee_orx)
        wy = (goal_or[1] - self.ee_ory)
        wz = (goal_or[2] - self.ee_orz)

        # if the error is less than a threshold return zero velocities
        if abs(wx)< 0.04 and abs(wy)< 0.04 and abs(wz)< 0.04:
            return 0.0,0.0,0.0
        
        return 3*wx, 3*wy, 3*wz
    
    '''
    Purpose:
    ---
    Calculates and applies the required linear velocities for the end effector to reach a target position.

    Input Arguments:
    ---
    goal_pose : tuple
    Target position as (x, y, z).

    Returns:
    ---
    tuple
    Linear velocities (vx, vy, vz).

    Example Call:
    ---
    vx, vy, vz = compute_linear_velocity((0.5, 0.5, 0.2))
    '''
    def move_to_pose(self, goal_pose):

        dist = self.compute_distance(goal_pose[0],goal_pose[1],goal_pose[2],self.ee_x,self.ee_y,self.ee_z)

        # compute position error about x, y and z axes and divide it by the distance to get its unit vector.
        vel_x = (goal_pose[0] - self.ee_x)/dist
        vel_y = (goal_pose[1] - self.ee_y)/dist
        vel_z = (goal_pose[2] - self.ee_z)/dist

        # if the error is less than a threshold return zero velocities
        if abs(vel_x)*dist < 0.04 and abs(vel_y)*dist < 0.04 and abs(vel_z)*dist < 0.04:
            self.state += 1
            return 0.0,0.0,0.0
        
        return vel_x, vel_y, vel_z

    '''
    Purpose:
    ---
    Implements a delay counter for executing tasks after a specified number of cycles.

    Input Arguments:
    ---
    cnt : int
    Number of cycles to count before returning True.

    Returns:
    ---
    bool
    True if the counter has reached the specified value, otherwise False.

    Example Call:
    ---
    if delay_counter(100):
        print("Delay complete, proceed to next task")
    '''

    def counter_delay(self, cnt):
        self.cnt += 1

        if self.cnt == cnt:
            return True
        else:
            return False

    '''
    Purpose:
    ---
    Implements a state machine to control the arm to pick the payload from the conveyor.
    Coordinates the entire process of identifying the picking positions of the payloads and attaching it to the gripper.
    Input Arguments:
    ---
    None

    Returns:
    ---
    None

    Example Call:
    ---
    Called periodically in a loop to manage the arm's movements.
    '''
    def pick_n_hold_payload(self):
        
        # initialize the twist message for the publisher
        ee_vel_msg = TwistStamped()
        ee_vel_msg.header.stamp = self.get_clock().now().to_msg()
        ee_vel_msg.header.frame_id = ur5.base_link_name()

        # STATE 0 : Move the end effector arm to an orientation which points it downwards.
        if self.state == 0:
            or_goal = [3.14,0.0,1.57]
            ori = self.move_to_or(or_goal)

            # constant velocity provided to the end effector until the orientation has been reached. It is stored in TwistStamped msg ee_vel_msg.
            ee_vel_msg.twist.angular.x = 0.0
            ee_vel_msg.twist.angular.y = 3.5
            ee_vel_msg.twist.angular.z = 0.0

            if ori[0] == 0.0 and ori[1] == 0.0:
                ee_vel_msg.twist.angular.y = 0.0

                # state is increamented when the move_to_or function returns zero velocity indicating the process is completed
                self.state+=1

        # STATE 1 : Get the pick position of the payload, along with their marker id.
        if self.state == 1:

            # function is called and the client request is sent.
            self.get_payload_pose()
            
            # wait for a breif moment to receive the response from the server
            if self.counter_delay(3):

                # response stored in class objects : pick, drop, payload_id
                self.pick = self.response.payload_pick # 1-D array which stores the pick positions as : [x1, y1, z1, x2, y2, z2......]
                self.payload_id = self.response.payload_id # 1-D array which stores the payload ids as : ["box_1", "box_2"......]
                self.cnt = 0
                
                # state is increamented on receiving the response from server
                self.state+=1
        
        # STATE 2 : Move to an intermediate position just above the payload to avoid collisions with the payload
        if self.state == 2:
            
            # call the move_to_pose function and pass the appropriate goal coordinates.
            vx, vy, vz = self.move_to_pose((self.pick[0],self.pick[1],self.pick[2]+0.4))

            # the velocities returned by move_to_pose function are stored in ee_vel_msg as linear velocities.
            ee_vel_msg.twist.linear.x = vx
            ee_vel_msg.twist.linear.y = vy
            ee_vel_msg.twist.linear.z = vz

        # STATE 3 : Move to the pick position that is, the position of a payload.
        if self.state == 3:
            
            # call the move_to_pose function and pass the goal coordinates stored in the pick array.
            vx, vy, vz = self.move_to_pose((self.pick[0],self.pick[1],self.pick[2]-0.06))
            ee_vel_msg.twist.linear.x = vx
            ee_vel_msg.twist.linear.y = vy
            ee_vel_msg.twist.linear.z = vz

            # if the arm is pressing on the box hard enough to attach, change state
            if self.net_wrench > 70:
                self.arm_gripper(1)
                time.sleep(2.0)
                self.state+=1

        # STATE 4 : Move to an intermediate point located just above the pick position.
        if self.state == 4:
            
            # call the move_to_pose function and pass the goal coordinates stored in the drop array.
            vx, vy, vz = self.move_to_pose((self.pick[0],self.pick[1],self.pick[2]+0.4))

            ee_vel_msg.twist.linear.x = vx
            ee_vel_msg.twist.linear.y = vy
            ee_vel_msg.twist.linear.z = vz
            # self.get_logger().info("PICKING UP THE PACKAGE !")

        # STATE 5 : Set the pick_n_hold variable as True indicating that the payload has been picked up and ready to be dropped.
        if self.state == 5:
            self.pick_n_hold = False # end of pick n hold operation

            self.state = 1
            self.get_logger().info(f'BOX NAME {self.payload_id}')
        
        # publish the velocities as in ee_vel_msg.
        self.ee_vel_pub.publish(ee_vel_msg)
    
    '''
    Purpose:
    ---
    Implements a state machine to control the arm for placing the payload on the ebot.
    Coordinates the entire process of identifying drop position and detaching the payload.

    Input Arguments:
    ---
    None

    Returns:
    ---
    None

    Example Call:
    ---
    Called periodically in a loop to manage the arm's movements.
    '''

    def drop_payload(self):
        
        # initialize the twist message for the publisher
        ee_vel_msg = TwistStamped()
        ee_vel_msg.header.stamp = self.get_clock().now().to_msg()
        ee_vel_msg.header.frame_id = ur5.base_link_name()
        
        # STATE 1 : Get the drop position of the payload, along with their marker id.
        if self.state == 1:

            # function is called and the client request is sent.
            self.get_payload_pose()
            
            # wait for a breif moment to receive the response from the server
            if self.counter_delay(3):
                self.cnt = 0

                # response stored in class objects : pick, drop, payload_id
                self.drop = self.response.payload_drop # 1-D array which stores the drop positions as : [x1, y1, z1, x2, y2, z2......]
            
                # state is increamented on receiving the response from server
                self.state+=1

        # STATE 2 : Move to an intermediate position to avoid collision with conveyor belt or self.
        if self.state == 2:

            vx, vy, vz = self.move_to_pose((self.drop[0],self.drop[1],self.drop[2]+0.5))

            ee_vel_msg.twist.linear.x = vx
            ee_vel_msg.twist.linear.y = vy
            ee_vel_msg.twist.linear.z = vz

        # STATE 3 : Move to the drop position, maintaining a clearence of 20cm to account for the payload dimension.
        if self.state == 3:

            vx, vy, vz = self.move_to_pose((self.drop[0],self.drop[1],self.drop[2]+self.payload_dimension))

            ee_vel_msg.twist.linear.x = vx
            ee_vel_msg.twist.linear.y = vy
            ee_vel_msg.twist.linear.z = vz
        
        # STATE 4 :  Detach the payload from the end effector.
        if self.state == 4:

            # function detach_payload sends the request to detach the payload from the gripper
            self.arm_gripper(0)
            self.state+=1
        
        # STATE 5 :  Move the end effector slightly above the drop position, to avoid collision when the ebot moves.
        if self.state == 5:

            vx, vy, vz = self.move_to_pose((self.drop[0],self.drop[1],0.15))

            ee_vel_msg.twist.linear.x = vx
            ee_vel_msg.twist.linear.y = vy
            ee_vel_msg.twist.linear.z = vz
        
        # STATE 6 : Set the drop_payload_ variable as True indicating the completion of the passing operation.
        if self.state == 6:

            self.drop_payload_ = False # dropping complete
            self.state = 1
        
            self.get_logger().info(f'BOX NAME {self.payload_id}')
        
        # publish the velocities as in ee_vel_msg.
        self.ee_vel_pub.publish(ee_vel_msg)
    
    '''
    Purpose:
    ---
    Periodically retrieves the transform between the base of the robot arm and the end effector, 
    essential for motion planning and state updates.

    Input Arguments:
    ---
    None

    Returns:
    ---
    None

    Example Call:
    ---
    This function is periodically called as a timer callback.
    '''

    def lookup_transform(self):
        try:
            # Lookup the transform from 'base_link' to 'map' frame
            transform = self.tf_buffer.lookup_transform('base_link','wrist_3_link', rclpy.time.Time()) 

            self.ee_x = transform.transform.translation.x
            self.ee_y = transform.transform.translation.y
            self.ee_z = transform.transform.translation.z

            ox = transform.transform.rotation.x
            oy = transform.transform.rotation.y
            oz = transform.transform.rotation.z
            ow = transform.transform.rotation.w

            # convert the orientation to euler angles.
            self.ee_orx, self.ee_ory, self.ee_orz = euler_from_quaternion([ox,oy,oz,ow])

        except Exception as e:
            self.get_logger().warn(f"Could not lookup transform: {e}")
    
    '''
    Purpose:
    ---
    Function that computes euclidian distance between given two points.

    Input Arguments:
    ---
    x1,y1,z1,x2,y2,z2 : float
    Coordinates of the points

    Returns:
    ---
    euclidian distance

    Example call:
    ---
    dist = self.compute_distance(1.0,1.1,2.2,0.0,1.2,1.0) : distance between (1,1.1,2.2) and (0.0,1.2,1.0)
    
    '''
    def compute_distance(self,x1,y1,z1,x2,y2,z2):
        return np.sqrt((x2 - x1)**2 + (y2 - y1)**2 + (z2 - z1)**2)
'''
Purpose:
---
Main function spins the node in separate threads to make sure the functions occur asynchronously.
'''
def main(args=None):
    rclpy.init(args=args)

    node = TransformLookupNode()
    executor = MultiThreadedExecutor(3)
    executor.add_node(node)
    executor.spin()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

# ros2 service call /servo_node/start_servo std_srvs/srv/Trigger {}