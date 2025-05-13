#!usr/env/bin python3

#Personal Space node for ROB499 HW5
#This node takes in a point cloud identifying people, and determines 
#if they are within the robot's personal space bubble

#rclpy imports
import rclpy
from rclpy.node import Node

#message type imports
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool

#useful functions for working with PointCloud2 messages:
from sensor_msgs_py import point_cloud2

#import numpy
import numpy as np

class PersonalSpace(Node):
	def __init__(self):
		super().__init__('personal_space')
		
		#This should be a parameter ideally
		self.personal_diameter = 2.0

		self.subscriber = self.create_subscription(PointCloud2, 'people_points', self.sub_callback, 10)

		self.marker_pub = self.create_publisher(Marker, 'vizualization_marker', 10)

		self.capture_pub = self.create_publisher(Bool, 'capture', 10)

		self.in_space_last_loop = False
		
	def sub_callback(self, msg):
		#(re)publish a marker showing our territory
		self.pub_ps()

		#unpack the points from the message:
		points = point_cloud2.read_points(msg)		
		self.get_logger().info(f'Header: {msg.header}')
		self.get_logger().info(f'people points: {points}')
		

		#iterate over each point in the point cloud, checking if they're within our personal space
		in_space = False
		for point in points:
			dist = np.sqrt(point[0]**2 + point[1]**2 + point[2]**2)
			if dist < self.personal_diameter/2:
				in_space = True

			#do stuff if there's someone in our space
			if (not self.in_space_last_loop and in_space):

				#Ensure we don't publish multiple times
				self.in_space_last_loop = True

				#publish a marker to Rviz at the place we saw the intruder:
				point_msg = Point(x=float(point[0]),y=float(point[1]),z=float(point[2]))
				self.pub_intruder(point_msg)

				#publish a message to our image capture node asking it to save a photo for us:
				cap_msg = Bool()
				cap_msg.data = True
				self.capture_pub.publish(cap_msg)
		
		#After the loop, reset things if there's not someone in our space:
		if not in_space:
			self.in_space_last_loop = False

	def pub_intruder(self, point):
		
		#publish a marker which shows an intruder in the robot's personal space:
		ps_marker = Marker()
		ps_marker.header.frame_id = 'ramsis/base_laser_scanner';
		ps_marker.header.stamp = self.get_clock().now().to_msg()
		ps_marker.ns = "personal_space"
		ps_marker.id = 1
		ps_marker.type = 3 #visualization_msgs.msg.Marker.CYLINDER
		ps_marker.action = 0 #visualization_msgs.msg.Marker.ADD
		ps_marker.pose.position = point
		ps_marker.pose.orientation.x = 0.0
		ps_marker.pose.orientation.y = 0.0
		ps_marker.pose.orientation.z = 0.0
		ps_marker.pose.orientation.w = 1.0
		ps_marker.scale.x = 0.2
		ps_marker.scale.y = 0.2
		ps_marker.scale.z = 0.75
		ps_marker.color.r = 0.0
		ps_marker.color.g = 0.0
		ps_marker.color.b = 1.0
		ps_marker.color.a = 0.75
		ps_marker.lifetime = rclpy.duration.Duration(seconds=1.0).to_msg()

		self.marker_pub.publish(ps_marker)
		self.get_logger().info('personal space marker published')



	def pub_ps(self):
		
		#publish a marker which shows the robot's personal space:
		ps_marker = Marker()
		ps_marker.header.frame_id = 'ramsis/base_laser_scanner';
		ps_marker.header.stamp = self.get_clock().now().to_msg()
		ps_marker.ns = "personal_space"
		ps_marker.id = 0
		ps_marker.type = 3 #visualization_msgs.msg.Marker.CYLINDER
		ps_marker.action = 0 #visualization_msgs.msg.Marker.ADD
		ps_marker.pose.position = Point(x=0.0,y=0.0,z=0.0)
		ps_marker.pose.orientation.x = 0.0
		ps_marker.pose.orientation.y = 0.0
		ps_marker.pose.orientation.z = 0.0
		ps_marker.pose.orientation.w = 1.0
		ps_marker.scale.x = self.personal_diameter
		ps_marker.scale.y = self.personal_diameter
		ps_marker.scale.z = 0.1
		ps_marker.color.r = 0.0
		ps_marker.color.g = 1.0
		ps_marker.color.b = 0.0
		ps_marker.color.a = 0.5
		ps_marker.lifetime = rclpy.duration.Duration(seconds=1.0).to_msg()

		self.marker_pub.publish(ps_marker)
		self.get_logger().info('personal space marker published')

def main(args = None):
	rclpy.init()

	personal_space = PersonalSpace()

	rclpy.spin(personal_space)

	rclpy.shutdown()

#End of File
