#!usr/bin/env python3

#Data Reciever node for HW3 of ROB499

#This node recieves packets from data sender, calculates the latenecy, and rebroadcasts it.

#import the things that every node needs
import rclpy
from rclpy.node import Node

#import our custom message and service definitions
from hw_interfaces.msg import TestPacket
from hw_interfaces.srv import EnableLogging

#rebroadcast as a Int32 of nanoseconds
from std_msgs.msg import Int32

#manually log to a csv file when enabled via services
import csv

#Define the class that inherits from node:
class DataReceiver(Node):
	def __init__(self):
		#initialize the node
		super().__init__('data_receiver')

		#Create a subscriber which subs to the 'data' topic:
		self.sub = self.create_subscription(TestPacket,'data',self.callback,10)

		#Create a service for enabling/disabling logging to a csv file:
		self.service = self.create_service(EnableLogging,'enable_logging',self.service_callback)
		self.log_enable = False

		#Create some publishers to rebroadcast the average and instantaneous latency
		self.pub_avg = self.create_publisher(Int32,'latency',10)
		self.pub_instant = self.create_publisher(Int32,'raw_latency',10)

		#Create a 10 sample circular buffer for calculating avg latency:
		self.buffer_length = 10
		self.latency_buffer = [0]*self.buffer_length
		self.buffer_index = 0

		self.avg_latency = 0

	def callback(self,msg):
		#log the message we recieved:
		self.get_logger().info(f'Received TestPacket with sent_time: {msg.send_time}')
	
		#Define two new messages to send
		avg_msg = Int32()
		inst_msg = Int32()

		#Calculate the latency
		self.latency_buffer[self.buffer_index] = self.get_clock().now().nanoseconds - rclpy.time.Time.from_msg(msg.send_time).nanoseconds

		#Calculate the average latency
		#list comprehension accounts for the starting edgecase where the buffer isn't full (probably slow)
		self.avg_latency = int(sum(self.latency_buffer) / len([v for v in self.latency_buffer if v != 0]))

		#Send and log messages
		avg_msg.data = self.avg_latency
		inst_msg.data = self.latency_buffer[self.buffer_index]

		self.pub_avg.publish(avg_msg)
		self.pub_instant.publish(inst_msg)

		self.get_logger().info(f'published inst lat: {self.latency_buffer[self.buffer_index]}, avg lat: {self.avg_latency}')

		if self.log_enable:
			self.local_log_data()

		#increment the circular buffer
		self.buffer_index = (self.buffer_index + 1) % (self.buffer_length - 1)

	#callback function which is run when a service client submits a request:
	def service_callback(self, request, response):
		#Check if we are to start or stop file logging
		if (request.enable and not self.log_enable):
			#Enable logging with the filename we were provided and open the file to log to
			self.log_enable = True
			self.filename = request.filename
			self.get_logger().info(f'Received request to start local logging to {self.filename}')
		
		elif (self.log_enable and not request.enable):
			#Disable logging with the filename we were provided and close the file
			self.log_enable = False
			self.filename = None
			self.get_logger().info(f'Received request to stop local logging')
		
		else:
			#Note that we got a redundant request:
			self.get_logger().info(f'Received a redundant request for local logging = {request.enable}')
			
		#acknowledge the service call when done
		response.acknowledge = True
		return response

	#Function which is run when self.log_enable is true, logging to a local csv file.
	def local_log_data(self):
		#Write to the file (Probably the excruciatingly slow way by opening and closing it each time)
		with open(self.filename, 'a', newline='') as csvfile:
			local_logger = csv.writer(csvfile)
			local_logger.writerow([self.get_clock().now().nanoseconds,self.latency_buffer[self.buffer_index],self.avg_latency])

#Node entry point
def main(args=None):
	#init rclpy
	rclpy.init(args=args)

	#Create an instance of our node
	data_receiver = DataReceiver()

	#Speeeeeeeeeeeeeeeeeeeeeeen forever
	rclpy.spin(data_receiver)

	#shutdown just in case
	rclpy.shutdown()

if __name__== '__main__':
	main()

