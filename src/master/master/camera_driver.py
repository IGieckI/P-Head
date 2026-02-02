import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import time

class NetworkCameraDriver(Node):
    def __init__(self):
        super().__init__('network_camera_driver')

        # Declare parameters so we can set IPs in the launch file
        self.declare_parameter('camera_ip', '0.0.0.0')
        self.declare_parameter('camera_name', 'camera')

        self.ip = self.get_parameter('camera_ip').get_parameter_value().string_value
        self.name = self.get_parameter('camera_name').get_parameter_value().string_value
        
        # The standard ESP32-CAM stream URL
        self.stream_url = f"http://{self.ip}:81/stream"
        
        self.publisher_ = self.create_publisher(Image, f'/{self.name}/image_raw', 10)
        
        # 30 FPS timer
        self.timer = self.create_timer(0.033, self.timer_callback)
        self.bridge = CvBridge()
        
        self.get_logger().info(f'Attempting to connect to {self.name} at {self.stream_url}...')
        self.cap = cv2.VideoCapture(self.stream_url)

    def timer_callback(self):
        if not self.cap.isOpened():
            # Try to reconnect if connection lost
            self.cap.open(self.stream_url)
            return

        ret, frame = self.cap.read()
        if ret:
            # Convert OpenCV image to ROS2 message
            msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self.name
            self.publisher_.publish(msg)
        else:
            self.get_logger().warn(f'Failed to retrieve frame from {self.name}')

def main(args=None):
    rclpy.init(args=args)
    node = NetworkCameraDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cap.release()
        node.destroy_node()
        rclpy.shutdown()