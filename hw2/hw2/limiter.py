#!/usr/bin/env python3


#A ROS2 transformer node which limits/truncates a Float32 value to within a specific window

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32

class Limiter(Node):
	'''
	This node should subscribe to a topic publishing Float32 values, shrink them to within specified bounds,
	and republish on a specified outbound topic.
	'''

	def __init__(self,lower,upper):
		#Initialize the parent class:
		super().__init__('limiter')

		self.lower_bound = lower
		self.upper_bound = upper
		
		#Create the publisher first so it's buffer is available once we start the subscriber
		self.pub = self.create_publisher(Float32, 'new_number', 10)

		#Create the subscriber
		self.sub = self.create_subscription(Float32, 'number', self.callback, 10)

	#Define a function which truncates the input given the defined lower and upper bounds.
	def truncate(self, input):
		if (input < self.lower_bound):
			return self.lower_bound
		elif (input > self.upper_bound):
			return self.upper_bound
		else: return input
			
	#Define a callback function which runs every time we see a new message on the subbed topic
	def callback(self,msg):
		#make a new message to be sent:
		new_msg = Float32()

		#Set the value
		new_msg.data = self.truncate(msg.data)

		#Publish the new message:
		self.pub.publish(new_msg)

		#log the new message so we see something in the terminal:
		self.get_logger().info(f'Published {new_msg.data}')

#Define an entry point with the specific limits we want defined.
def limit(args = None):
	run_node(-0.5,0.5)

#Abstract away starting and running the node in this function
def run_node(lower,upper, args=None):
	
	#Initialize rclpy
	rclpy.init(args=args)

	#Make an instance of the Limiter Node:
	limiter = Limiter(lower,upper)

	#Hand node control over to ROS with the spin command
	rclpy.spin(limiter)

	#Shut things down if we magically get to this line
	rclpy.shutdown()

#setup a main function in case we want to run this as a standalone script
if __name__ == '__main__':
	limit()
