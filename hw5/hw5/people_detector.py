#!usr/env/bin python3

#People detector for ROB499 homework 5

#Uses Scikit's DBScan and some math to find the centers of each potential 
#person from a filtered laser scan (cluster of points)

#ROS2 imports:
import rclpy
from rclpy.node import Node

#message types for input and output
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import PointCloud2

#useful functions for working with PointCloud2 messages
from sensor_msgs_py import point_cloud2

#include numpy and scikit-learn for it's DBSCAN algorithm
import numpy as np
from sklearn.cluster import DBSCAN

#Node definition
class PeopleDetector(Node):
	def __init__(self):
		super().__init__('people_detector')

		#setup subscriber and publisher
		self.sub = self.create_subscription(LaserScan, 'input_scan', self.sub_callback, 10)
		self.pub = self.create_publisher(PointCloud2, 'output_points', 10)

		#Setup parameters for DBSCAN

	def sub_callback(self, msg):
		#First, convert the laserscan to cartesian from polar coords
		xyz_pts = []
		for i,r in enumerate(msg.ranges):
			#skip points that are marked as inf (Not there or background)
			if not (msg.range_min < r < msg.range_max):
				continue;
			
			#Calculate the angle of rotation
			theta = msg.angle_min + i * msg.angle_increment

			#convert to cartesian points:
			point = [r*np.cos(theta), r*np.sin(theta),0]
			xyz_pts.append(point)
	
		#Hacky, but I don't know the initial list size and need numpy functionality after
		xyz_pts = np.array(xyz_pts)

		#Second, pass the points to Scikit DBSCAN, which will classify them for us
		db = DBSCAN(eps = 0.5, min_samples = 6).fit(xyz_pts)
		
		#Group IDs of each point detected
		labels = db.labels_
		unique_labels = set(labels)

		#Third, since DBSCAN doesn't seem to give us the center of each group, we find that manually via averaging
		#Get the center of mass of each group:
		people_locations = []
		for l in unique_labels:
			#Ignore noise points
			if l == -1:
				continue
			#get all the indicies of points in the group:
			indices = np.where(labels == l)
			#self.get_logger().info(f'indices: {indices}')
			self.get_logger().info(f'xyz_pts: {xyz_pts[indices]}')
			#get the average location
			location = np.sum(xyz_pts[indices], 0) / len(indices[0]) #indicies is a single entry inside a tuple
			self.get_logger().info(f'avg loc: {location}')

			people_locations.append(location)

		#Finally, We repackage and rebroadcast this as a PointCloud2 object for later use
		new_msg = point_cloud2.create_cloud_xyz32(msg.header, people_locations)
		self.pub.publish(new_msg)

#Nothing special, init rclpy and run the node
def main(args = None):
	rclpy.init(args = args)

	people_detector = PeopleDetector()

	rclpy.spin(people_detector)

	rclpy.shutdown()

#End of File
