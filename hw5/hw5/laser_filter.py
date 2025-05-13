#!usr/bin/env/ python3

# Node which 'filters' a LaserScan message by removing a snapshot of the background.
# Osian Leahy

#Import rclpy stuffs:
import rclpy
from rclpy.node import Node
from rclpy.parameter import parameter_value_to_python
from rclpy.parameter_event_handler import ParameterEventHandler

#Import the laser scan message which we will recieve and rebroadcast
from sensor_msgs.msg import LaserScan

#Lets try using Trigger.srv from the standard services:
from std_srvs.srv import Trigger

#Import numpy for array subtraction:
import numpy as np

class LaserFilter(Node):
	def __init__(self):
		#Initialize the parant class
		super().__init__('laser_filter')
		
		#Setup a subscriber
		self.sub = self.create_subscription(LaserScan, 'input_scan', self.subscriber_callback, 10)
		
		#Setup a publisher 
		self.pub = self.create_publisher(LaserScan, 'output_scan', 10)

		#Setup the service server
		self.service = self.create_service(Trigger,'scan_snapshot',self.service_callback)

		#Setup the parameter handler and default parameters
		self.declare_parameter('epsilon', 0.2) #Distance below which we consider points to be the same as background
		self.param_handler = ParameterEventHandler(self)

		self.param_callback_handle = self.param_handler.add_parameter_callback(
			parameter_name = 'epsilon',
			node_name = 'laser_filter',
			callback = self.param_callback
		)

		#setup some data objects we'll need:
		self.subtract_background = False #This is set to true once we start subtracting the background

	#This callback modifies and republishes the LaserScan when run:
	def subscriber_callback(self, msg):
		#Remove the background if we have chosen a background snapshot:
		if self.subtract_background:
			new_msg = msg
			
			new_ranges = np.array(msg.ranges)
			diff_ranges = new_ranges - np.array(self.background_scan.ranges)
			
			#self.get_logger().info(f'{np.array(msg.ranges)}')

			for i, v in np.ndenumerate(diff_ranges):

				#Three edgecases:
					#nan: result of inf-inf, nothing there in current or background
					#inf: result of inf-num, nothing there currently, previously in background
					#-inf, result of num-inf, something there that wasn't in background
				if v == float('nan') or v == float('inf'):
					new_ranges[i] = float('inf')
				elif v == float('-inf'):
					pass #leave the current value since there used to be nothing here
				
				#Two possible threshold results:
					#Abs value is < epsilon; point is too close to be considered new, set to inf
					#Abs value is > epsilon; point is far enough to be new, leave it
				elif abs(v) < self.get_parameter('epsilon').get_parameter_value().double_value:
					new_ranges[i] = float('inf') #
				elif abs(v) > self.get_parameter('epsilon').get_parameter_value().double_value:
					pass #leave the current value since something is significantly closer
			
			#Rebroadcast new ranges
			new_msg.ranges = new_ranges.tolist()
		
		#otherwise just pass the LaserScan message through:
		else:
			self.background_scan = msg
			new_msg = msg

		#Do these irrsepective of what happens above:
		self.get_logger().info(f'Recieved: {msg.header}, Sending: {new_msg.header}')
		self.pub.publish(new_msg)

	#This callback instructs us to use the last scan as a baseline 'background' scan
	def service_callback(self, request, response):
		#Start subtracting the last see background:
		self.subtract_background = True

		#return a successful response:
		response.success = True
		response.message = f'Subtracting background scan: {self.background_scan.header}'
		self.get_logger().info(response.message)

		return response
	
	#This callback logs the epsilon parameter update
	def param_callback(self,parameter):
		self.get_logger().info(f'{parameter.name} updated to {parameter.value}')

#Entry point for the node
def main(args=None):
	#init rclpy first:
	rclpy.init(args=args)

	#create an instance of the class:
	filt = LaserFilter()

	#rclpy speeen:
	rclpy.spin(filt)

	#shutdown rclpy after done:
	rclpy.shutdown()

#End of File
