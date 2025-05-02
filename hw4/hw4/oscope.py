#!/usr/bin/env python3

# Oscope 2 assignment for ROB 499 HW4
#
# oscope.py
#
# Osian Leahy
#

#Import the ros python Node class & infrastructure
import rclpy
from rclpy.node import Node

#import the required parameter stuffs:
from rclpy.parameter import parameter_value_to_python
from rclpy.parameter_event_handler import ParameterEventHandler

#Import the python time module rather than trust the timer function for accurate time?
import time

#import math for the sine function
import numpy as np

#import the SendData service from last homework:
from hw_interfaces.srv import SendData

# We're going to publish a Float32, which is a standard ROS message type
from std_msgs.msg import Float32

#Create a class instance inheriting from node
class OscopePublisher(Node):
	def __init__(self):
		# Initialize the parent class, giving it a name.
		super().__init__('oscope')

		# Create a publisher, and assign it to a member variable. The call takes a type, topic name, and queue size.
		self.pub = self.create_publisher(Float32, 'oscope', 100)

		#Timer to sample the sine wave
		self.timer = self.create_timer(0.01, self.callback)

		#Setup a parameter event handler for parameter change callbacks:
		self.handler = ParameterEventHandler(self)

		# Define node parameters for the sinusoidal frequency and the output value
		self.declare_parameter('frequency', 1.0) #hz
		self.freq_callback_handle = self.handler.add_parameter_callback(
			parameter_name = 'frequency',
			node_name = 'oscope',
			callback = self.param_cb
		)

		self.declare_parameter('clamp', 1.0) #Symmetric +/- limit
		self.clamp_callback_handle = self.handler.add_parameter_callback(
			parameter_name = 'clamp',
			node_name = 'oscope',
			callback = self.param_cb
		)

		#This variable stores the sine wave output
		self.outputValue = 0;

		#quick service for starting and stopping data send
		self.service = self.create_service(SendData, 'send_data', self.service_callback)

		self.sending_data = False

	#This callback prints the updated parameters when they're updated
	def param_cb(self, parameter):
		value = parameter_value_to_python(parameter.value)
		self.get_logger().info(f'{parameter.name} updated to: {value}')

	#Callback for data send:
	def service_callback(self, request, response):
		#Set and log the request
		self.sending_data = request.send
		self.get_logger().info(f'Recieved a request for sending_data = {request.send}')
		#Send a response
		response.acknowledge = True
		return response

	# This callback will be called every time the timer fires and we sample the sine wave.
	def callback(self):
		#Get the most recent parameters:
		freq = self.get_parameter('frequency').get_parameter_value().double_value
		clamp = self.get_parameter('clamp').get_parameter_value().double_value

		# Make an Float32 message, and fill in the information.
		msg = Float32()
		self.outputValue = np.clip(np.sin(freq * 2 * np.pi * time.time()),-1*clamp, clamp)
		msg.data = self.outputValue

		#Only publish if we have been told to do so:
		if (self.sending_data):
			# Publish the message, just like we do in ROS.
			self.pub.publish(msg)
			# Log that we published something.
			self.get_logger().info(f'Published {self.outputValue}')

#define a function to run the node. This will be our entry point
def run_node(args=None):
	# Initialize rclpy.
	rclpy.init(args=args)

	# Make an instance of the node class defined above.
	sinePublisher = OscopePublisher()

	# The spin() call gives control over to ROS2, and it now takes a Node-derived
	# class as a parameter.
	rclpy.spin(sinePublisher)

	# Make sure we shutdown everything cleanly.  This should happen, even if we don't
	# include this line, but you should do it anyway.
	rclpy.shutdown()

#Additional entry points can be defined which allow for different parameters (sine frequencies in this case):

# If we run the node as a script, then we're going to start here.
if __name__ == '__main__':
	# The idiom in ROS2 is to set up a main() function and to call it from the entry
	# point of the script.
	normal_wave()
