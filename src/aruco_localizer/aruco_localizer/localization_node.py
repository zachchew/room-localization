import rclpy
from rclpy.node import Node
import cv2
import numpy as np
import json
from autolab_core import RigidTransform
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from builtin_interfaces.msg import Time
from .utils import ARUCO_DICT
from geometry_msgs.msg import PoseStamped

class ArucoLocalizer(Node):
    def __init__(self):
        super().__init__('aruco_localizer')
        self.get_logger().info("Aruco Localizer Node Started")

        # Load camera parameters
        self.K = np.load("calibration_matrix.npy")
        self.D = np.load("distortion_coefficients.npy")

        # Load tag map
        with open("tag_map.json", "r") as f:
            self.tag_map = json.load(f)

        self.tag_length = 0.1  # meters
        self.bridge = CvBridge()
        self.aruco_dict = cv2.aruco.Dictionary_get(ARUCO_DICT["DICT_5X5_50"])
        self.detector_params = cv2.aruco.DetectorParameters_create()

        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )
        
        self.pose_pub = self.create_publisher(PoseStamped, "/robot_pose", 10)

        self.tf_broadcaster = TransformBroadcaster(self)

    def image_callback(self, msg):        
        # Convert ROS Image message to OpenCV format
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        corners, ids, _ = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.detector_params)

        if ids is not None:
            for i in range(len(ids)):
                tag_id = int(ids[i])
                if str(tag_id) not in self.tag_map:
                    self.get_logger().warn(f"Tag ID {tag_id} not in tag_map")
                    continue

                rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(
                    corners[i], self.tag_length, self.K, self.D)

                R = cv2.Rodrigues(rvec[0])[0]
                t = tvec[0][0]

                T_tag_cam = RigidTransform(R, t, from_frame="tag", to_frame="camera")
                T_tag_world = self.get_tag_pose(tag_id)
                T_cam_world = T_tag_world * T_tag_cam.inverse()

                # Broadcast transform
                self.publish_transform(T_cam_world)
                
                pose_msg = PoseStamped()
                pose_msg.header.stamp = self.get_clock().now().to_msg()
                pose_msg.header.frame_id = "world"
                pose_msg.pose.position.x = T_cam_world.translation[0]
                pose_msg.pose.position.y = T_cam_world.translation[1]
                pose_msg.pose.position.z = T_cam_world.translation[2]
                pose_msg.pose.orientation.x = T_cam_world.quaternion[0]
                pose_msg.pose.orientation.y = T_cam_world.quaternion[1]
                pose_msg.pose.orientation.z = T_cam_world.quaternion[2]
                pose_msg.pose.orientation.w = T_cam_world.quaternion[3]

                self.pose_pub.publish(pose_msg)
                
                self.get_logger().info(
                    f"📍 Robot Pose [world frame]:\n"
                    f"Position (x,y,z): {T_cam_world.translation}\n"
                    f"Orientation (quaternion): {T_cam_world.quaternion}\n"
                )


    def get_tag_pose(self, tag_id):
        tag = self.tag_map[str(tag_id)]
        pos = np.array(tag["position"])
        rot = np.array(tag["rotation"])
        return RigidTransform(rot, pos, from_frame="tag", to_frame="world")

    def publish_transform(self, T_cam_world):
        tf_msg = TransformStamped()
        tf_msg.header.stamp = self.get_clock().now().to_msg()
        tf_msg.header.frame_id = 'world'
        tf_msg.child_frame_id = 'camera'

        tf_msg.transform.translation.x = T_cam_world.translation[0]
        tf_msg.transform.translation.y = T_cam_world.translation[1]
        tf_msg.transform.translation.z = T_cam_world.translation[2]

        quat = T_cam_world.quaternion
        tf_msg.transform.rotation.x = quat[0]
        tf_msg.transform.rotation.y = quat[1]
        tf_msg.transform.rotation.z = quat[2]
        tf_msg.transform.rotation.w = quat[3]

        self.tf_broadcaster.sendTransform(tf_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ArucoLocalizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
