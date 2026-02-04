import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from std_msgs.msg import String, Int16MultiArray
from cv_bridge import CvBridge

import cv2
import numpy as np

class RobotMasterController(Node):
    def __init__(self):
        super().__init__('robot_master_controller')

        # Publisher topic the ESP32 will listen to for movement commands
        self.cmd_vel_pub = self.create_publisher(Twist, '/movement_out', 10)
        
        # Publisher for speech output (Text-to-Speech)
        self.speech_pub = self.create_publisher(String, '/robot/sound_out', 10)

        # Listener to the camera drivers
        self.create_subscription(Image, '/cam1/image_raw', self.cam1_callback, 10)
        self.create_subscription(Image, '/cam2/image_raw', self.cam2_callback, 10)
        
        # Listener for incoming voice text
        self.create_subscription(String, '/robot/sound_in', self.voice_callback, 10)
        # Audio Bridge listener
        self.create_subscription(Int16MultiArray, '/robot/sound_in', self.audio_callback, 10)
        # Internal state
        self.bridge = CvBridge()
        self.latest_cam1_frame = None
        self.latest_cam2_frame = None
        
        # Timer for the main control loop (10Hz)
        self.timer = self.create_timer(0.1, self.control_loop)
        
        self.get_logger().info("Robot Master Controller Started")

    def cam1_callback(self, msg):
        try:
            self.latest_cam1_frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"Error decoding cam1: {e}")

    def cam2_callback(self, msg):
        try:
            self.latest_cam2_frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"Error decoding cam2: {e}")
    
    def voice_callback(self, msg):
        self.get_logger().info(f"Heard voice command: {msg.data}")
        # TODO

    def audio_callback(self, msg):
        # raw Int16 audio chunks from ESP32
        data_len = len(msg.data)
        if data_len > 0:
            # TODO: Elaborate audio
            self.get_logger().info(f"Received audio bridge data: {data_len} samples")

    def control_loop(self):
        if self.latest_cam1_frame is not None:
            # Could perform some processing like face detection
            pass

        # Movement commands
        move_cmd = Twist()
                
        # For now, it just stop
        move_cmd.linear.x = 0.0
        move_cmd.angular.z = 0.0
        
        # Publish the command to the ESP32
        self.cmd_vel_pub.publish(move_cmd)

def main(args=None):
    rclpy.init(args=args)
    node = RobotMasterController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()