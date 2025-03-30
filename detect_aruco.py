#!/usr/bin/env python3

'''
# Team ID:          < LB#1136 >
# Theme:            < Cosmo Logistic >
# Author List:      < Krishnapranav >
# Filename:         < detect_aruco >
# Functions:        < depthimagecb, colorimagecb, detect_aruco, process_image, aruco_pose_callback, main>
# Global variables: < None>

PURPOSE OF THIS NODE : It handles processing the camera data, to detect the Aruco markers and computes their position and orientation in 3D space.
(This is a custom server ie. not mentioned in the theme, created to make the operation modular and robust.)
'''
import rclpy
import cv2
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener, TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from tf_transformations import quaternion_from_euler,quaternion_multiply
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2.aruco as aruco
import numpy as np
from payload_service.srv import ArucoSW
from rclpy.callback_groups import ReentrantCallbackGroup

class TransformLookupNode(Node):
    def __init__(self):
        super().__init__('transform_lookup_node')

        '''
        TRANSFORM
        '''
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.br = TransformBroadcaster(self)

        '''
        CAMERA
        '''

        # create the subscribers to get the data from the camera
        self.color_cam_sub = self.create_subscription(Image, '/camera/camera/color/image_raw', self.colorimagecb, 10)
        self.depth_cam_sub = self.create_subscription(Image, '/camera/camera/aligned_depth_to_color/image_raw', self.depthimagecb, 10)
        self.callback_group = ReentrantCallbackGroup()

        # create the service /payload_pose and initialize the detect_aruco server.
        self.detect_aruco_service = self.create_service(ArucoSW, '/payload_pos', self.aruco_pose_callback, callback_group=self.callback_group)

        
        image_processing_rate = 0.5  # rate of time to process image (seconds).
        self.bridge = CvBridge() # variable to bridge cv2 library with ROS2, Gazebo.                                                       

        # timer which runs constantly to process the image.                              
        self.timer = self.create_timer(image_processing_rate, self.process_image)       

        self.image = None # RGB image.
        self.depth_image = None # RGBD image.
        self.payload = {} # a dictionary which stores the marker id as keys and the corresponding position as values.

        self.get_logger().info("GET PAYLOAD POSE SERVICE STARTED !")

    '''
    Purpose:
    ---
    Callback function that gets the depth image from the RGBD camera. It converts it into a numpy array,
    and stores it in an object.

    Input Arguments:
    ---
    None

    Returns:
    ---
    None

    Example call:
    ---
    This function is periodically called at the specified interval set by create_timer().
    '''
    def depthimagecb(self, img_msg):

        try:
            # convert ROS Image into cv2 image
            self.depth_image = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding='passthrough')
        except CvBridgeError as e:
            self.get_logger().error(f"Failed to convert depth image: {str(e)}")
            return

    '''
    Purpose:
    ---
    Callback function that gets the RGB image from the camera. It converts it into a numpy array,
    and stores it in an object.

    Input Arguments:
    ---
    None

    Returns:
    ---
    None

    Example call:
    ---
    This function is periodically called at the specified interval set by create_timer().
    '''
    def colorimagecb(self, img_msg):
        try:
            # Convert ROS2 Image message to OpenCV image
            self.image = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding='bgr8')

        except Exception as e:
            self.get_logger().error(f"Error converting image: {e}")
    '''
    Purpose:
    ---
    Function which detects the Aruco markers on the payloads and on the ebot. It computes the 
    centre, orientation, and ID of the Aruco marker. It shows the image if required.

    Input Arguments:
    ---
    image: OpenCV image (numpy array) containing the RGB camera feed.

    Returns:
    ---
    center_x: list of x-coordinates of Aruco marker centers.
    center_y: list of y-coordinates of Aruco marker centers.
    angle_aruco: list of orientation angles of the detected markers.
    marker_id: list of IDs of detected Aruco markers.

    Example call:
    ---
    detect_aruco(image)
    '''
    def detect_aruco(self, image):

        # convert the image into Gray image
        gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        # initialize the Aruco Library to enable Aruco marker detection
        aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
        aruco_params = aruco.DetectorParameters()
        aruco_detector = aruco.ArucoDetector(aruco_dict, aruco_params)
        corners, ids, _ = aruco_detector.detectMarkers(gray_image)

        if ids is not None:
            
            # camera specifications
            cam_mat = np.array([[931.1829833984375, 0.0, 640.0], [0.0, 931.1829833984375, 360.0], [0.0, 0.0, 1.0]])
            dist_mat = np.array([0.0,0.0,0.0,0.0,0.0])

            # size of the aruco marker we are using
            size_of_aruco_m = 0.15

            obj_points = np.array([
                [-size_of_aruco_m / 2, size_of_aruco_m / 2, 0],
                [size_of_aruco_m / 2, size_of_aruco_m / 2, 0],
                [size_of_aruco_m / 2, -size_of_aruco_m / 2, 0],
                [-size_of_aruco_m / 2, -size_of_aruco_m / 2, 0]
            ], dtype=np.float32)

            # intialize the centre, marker_id and aruco orientation as empty arrays
            center_x = []
            center_y = []
            marker_id = []
            angle_aruco = []

            # loop through the corners of all the markers
            for i, corner in enumerate(corners):

                img_points = corner[0].astype(np.float32)  # Detected 2D points in the image

                # get the translation and rotation of each aruco marker in space with solvePnP library. (Mostly inaccurate due to lack of depth information.)
                success, rvec, tvec = cv2.solvePnP(obj_points, img_points, cam_mat, dist_mat)
                corners_array = corner[0]
                angle_aruco.append(rvec[1])

                # Find the center of the marker (mean of the four corner points)
                center_x.append(int(np.mean(corners_array[:, 0])))
                center_y.append(int(np.mean(corners_array[:, 1])))

                center = (center_x[i], center_y[i])
                marker_id.append(ids[i][0])

                # draw the centre of the marker in the cv2 window
                cv2.circle(image, center, 5, (0, 255, 0), -1)

                if success:
                    # draw axes on the centre of the marker in the cv2 window
                    cv2.drawFrameAxes(image, cam_mat, dist_mat, rvec, tvec, size_of_aruco_m * 2.0)
                else:
                    print(f"Failed to estimate pose for marker {i}.")

                # put text and add the marker id on the markers in the cv2 window
                cv2.putText(image, f"centre: {marker_id[i]}", (center_x[i] - 10, center_y[i] - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 125, 122), 2)
                cv2.polylines(image, [corners_array.astype(np.int32)], isClosed=True, color=(0, 125, 255), thickness=2)
                cv2.circle(image, center, 5, (0, 255, 255), -1)

            # return the centre, aruco orientation and ids
            return center_x, center_y, angle_aruco, marker_id
            
        else:
            print("No ArUco markers detected.")
        
        # show the cv2 window for debugging purposes
        # cv2.imshow("Image Window", image)
        # cv2.waitKey(1)

    '''
    Purpose:
    ---
    Function which processes the Aruco marker information to compute its position (x, y, z) and orientation (roll, pitch, yaw)
    in space, with respect to the camera frame. It also gets the base to object transformation and publishes the same.

    Input Arguments:
    ---
    None

    Returns:
    ---
    None

    Example call:
    ---
    This function is periodically called at the specified interval set by create_timer().
    '''
    def process_image(self):
        # self.get_logger().info("PROCESSING IMAGE")
        # Camera specifications.
        sizeCamX = 1280
        sizeCamY = 720
        centerCamX = 640
        centerCamY = 360
        focalX = 931.1829833984375
        focalY = 931.1829833984375

        if self.image is not None:
            # Detect the ArUco markers and extract their centers and orientations.
            aruco_data = self.detect_aruco(self.image)

            if aruco_data is None:
                self.get_logger().warn("ArUco detection failed: No data returned.")
                return

            try:
                center_x, center_y, angle_aruco, marker_id = aruco_data
            except ValueError as e:
                self.get_logger().warn(f"Unexpected format from detect_aruco: {str(e)}")
                return

            self.payload = {}

            for i in range(len(center_x)):
                try:
                    # Get the distance of the pixels in space using the depth image information.
                    dis = self.depth_image[center_y[i], center_x[i]] / 1000

                    # Calculate x, y, z coordinates from center pixel coordinates and depth image info.
                    x = dis * (sizeCamX - center_x[i] - centerCamX) / focalX
                    y = dis * (sizeCamY - center_y[i] - centerCamY) / focalY
                    z = dis

                    # Initialize the transform message.
                    tf_msg = TransformStamped()
                    tf_msg.header.stamp = self.get_clock().now().to_msg()

                    # Define the parent and child frames.
                    tf_msg.header.frame_id = "camera_link"  # Parent frame
                    tf_msg.child_frame_id = f"obj_{marker_id[i]}"  # Child frame

                    # Set translation (x, y, z) in meters.
                    tf_msg.transform.translation.x = z
                    tf_msg.transform.translation.y = x
                    tf_msg.transform.translation.z = y

                    # Set the orientation of the ArUco marker.
                    q1 = quaternion_from_euler(0.0, -0.83, 0.0)
                    q2 = quaternion_from_euler(3.14, 0.0, 0.0)
                    q3 = quaternion_multiply(q1, q2)
                    q4 = quaternion_from_euler(0.0, 0.0, angle_aruco[i] - 1.57)  # Roll, Pitch, Yaw
                    q_cam_to_box = quaternion_multiply(q3, q4)

                    tf_msg.transform.rotation.x = q_cam_to_box[0]
                    tf_msg.transform.rotation.y = q_cam_to_box[1]
                    tf_msg.transform.rotation.z = q_cam_to_box[2]
                    tf_msg.transform.rotation.w = q_cam_to_box[3]

                    # Publish the transform.
                    self.br.sendTransform(tf_msg)

                    # Look up the transform from base link to the payload.
                    base_to_box = self.tf_buffer.lookup_transform('base_link', f'obj_{marker_id[i]}', rclpy.time.Time())

                    tf_msg2 = TransformStamped()
                    tf_msg2.header.stamp = self.get_clock().now().to_msg()

                    tf_msg2.header.frame_id = "base_link"  # Parent frame
                    tf_msg2.child_frame_id = f"obj_{marker_id[i]}"  # Child frame

                    tf_msg2.transform.translation.x = base_to_box.transform.translation.x
                    tf_msg2.transform.translation.y = base_to_box.transform.translation.y
                    tf_msg2.transform.translation.z = base_to_box.transform.translation.z

                    tf_msg2.transform.rotation.x = base_to_box.transform.rotation.x
                    tf_msg2.transform.rotation.y = base_to_box.transform.rotation.y
                    tf_msg2.transform.rotation.z = base_to_box.transform.rotation.z
                    tf_msg2.transform.rotation.w = base_to_box.transform.rotation.w

                    # Publish the transform from base to box.
                    # self.br.sendTransform(tf_msg2)

                    # Store the ArUco position in the dictionary payload with their IDs.
                    self.payload[f'box{marker_id[i]}'] = (
                        base_to_box.transform.translation.x,
                        base_to_box.transform.translation.y,
                        base_to_box.transform.translation.z,
                    )

                    self.get_logger().info(f'{self.payload}')

                except IndexError as e:
                    self.get_logger().warn(f"Index error at marker {i}: {str(e)}")
                except Exception as e:
                    self.get_logger().warn(f"Error during processing marker {i}: {str(e)}")

    '''
    Purpose:
    ---
    Function called whenever a request for /payload_pos service is requested. It stores the pick and drop positions in 
    an array and returns the response. It also returns an array with the marker IDs.

    Input Arguments:
    ---
    request: ArucoSW.Request object containing the service request data.
    response: ArucoSW.Response object used to return the response data.

    Returns:
    ---
    response: Service response containing payload pick positions, drop positions, and IDs.

    Example call:
    ---
    Triggered when the /payload_pos service is invoked.
    '''          
    def aruco_pose_callback(self, request : ArucoSW.Request, response : ArucoSW.Response):
        
        # catch the request
        get_position = request.get_position

        self.get_logger().info(f"PICK AND DROP POSITION REQUESTED !")

        if get_position == True:

            # initialize the response : pick position, drop position and payload ids.
            pick = []
            drop = []
            payload_id = []
            box_pose = list(self.payload.values())
            box_id = list(self.payload.keys())

            # loop through the dictionary values
            for i in range(len(box_pose)):

                # based on their position in space divide the coordinates into drop and pick positions.

                if box_pose[i][2] < -0.1:
                    drop.append(box_pose[i][0])
                    drop.append(box_pose[i][1])
                    drop.append(box_pose[i][2])
                
                else:
                    pick.append(box_pose[i][0])
                    pick.append(box_pose[i][1])
                    pick.append(box_pose[i][2])
                    payload_id.append(box_id[i])

            # Store the response and return
            response.payload_pick = pick
            response.payload_drop = drop
            response.payload_id = payload_id

        return response
       
'''
Purpose:
---
main function which spins the node.
'''
def main(args=None):
    rclpy.init(args=args)

    node = TransformLookupNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    # Shutdown the ROS client library
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()   