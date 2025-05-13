# Camera Capture node for ROB499 HW5

#This node watches an image stream and listens for a capture message,
#when a message is recieved, the node saves the last seen image.
#This should probably be implemented as a service rather than a subscriber? meh.

#Import core ROS stuff
import rclpy
from rclpy.node import Node

#Import message types
from sensor_msgs.msg import Image
from std_msgs.msg import Bool

#OpenCV and cv_bridge for handling images:
import cv2
from cv_bridge import CvBridge

class CameraCapture(Node):
	def __init__(self):
		super().__init__('camera_capture')
		
		#subscribe to both the camera and capture topics
		self.cam_sub = self.create_subscription(Image, 'camera_feed', self.cam_callback, 10)
		self.cap_sub = self.create_subscription(Bool, 'capture', self.cap_callback, 10)
		
		#Cv Bridge for handling and saving images:
		self.bridge = CvBridge()
		
		#Previous Image in openCV form:
		self.prev_image = None

		#Counter for image saving:
		self.image_num = 0

	#unpack the image message and save it to the node
	def cam_callback(self, msg):
		self.get_logger().info('Frame Recieved!')
		self.prev_image = self.bridge.imgmsg_to_cv2(msg)
		#self.get_logger().info(f'{self.prev_image}')
	
	#Save the last seen image when called
	def cap_callback(self, msg):
		self.get_logger().info('Saving Previous Image!')
		#Save image with opencv
		success = cv2.imwrite(f'Intruder_{self.image_num}.jpg', self.prev_image)
		self.get_logger().info(f'Image Save Success: {success}')
		#Increment the counter so we don't overwrite images
		self.image_num = self.image_num + 1

def main(args = None):
	rclpy.init()

	camera_capture = CameraCapture()

	rclpy.spin(camera_capture)

	rclpy.shutdown()

#End of File
