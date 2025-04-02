import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class RTSPImagePublisher(Node):
    def __init__(self):
        super().__init__('rtsp_image_publisher')
        self.publisher_ = self.create_publisher(Image, '/camera/image_raw', 10)
        self.bridge = CvBridge()

        # RTSP stream
        self.rtsp_url = "rtsp://10.8.0.2:8080/h264_ulaw.sdp"
        self.cap = cv2.VideoCapture(self.rtsp_url)

        if not self.cap.isOpened():
            self.get_logger().error(f"❌ Failed to open RTSP stream at {self.rtsp_url}")
            rclpy.shutdown()
            return

        self.get_logger().info(f"✅ Connected to RTSP stream at {self.rtsp_url}")

        # Timer callback at ~30 FPS
        timer_period = 1.0 / 5.0
        self.timer = self.create_timer(timer_period, self.publish_frame)

    def publish_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn("⚠️ Failed to grab frame")
            return

        msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera'
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = RTSPImagePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
