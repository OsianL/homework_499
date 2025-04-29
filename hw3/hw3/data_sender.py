#!/usr/bin/env python3

#A node to test the TestPacket custom message type for hw3.

import rclpy
from rclpy.node import Node

#Custom message and service import
from hw_interfaces.msg import TestPacket
from hw_interfaces.srv import SendData

#Define our Node class
class DataSender(Node):
	def __init__(self):
		#Initialize the parent class
		super().__init__('data_sender')

		#Create a publisher, publish at 1hz (last field is buffer size not frequency genius)
		self.pub = self.create_publisher(TestPacket,'data',10)
		
		#Setup a timer to repeatedly run the callback function (which publishes data) (period is set here)
		self.timer = self.create_timer(1, self.callback)
		
		#Define a service server for starting and stopping data send.
		self.service = self.create_service(SendData,'send_data',self.service_callback)

		#Define any other class variables we want here.
		self.sendingData = False

	def callback(self):
		#Only send data if sendingData is toggled to True:
		if self.sendingData:

			#make an instance of our message:
			msg = TestPacket()

			#Assign it values
			msg.send_time = self.get_clock().now().to_msg()
			msg.payload = [0,1,2,3]

			#publish the message and log it:
			self.pub.publish(msg)
			self.get_logger().info(f'Published time {msg.send_time}')
			self.get_logger().info(f'Published payload {msg.payload}')

	def service_callback(self,request,response):
		#Update whether we send data or not
		self.sendingData = request.send
		
		#Log the request:
		self.get_logger().info(f'Recieved a request for sendingData = {request.send}')

		#return true acknowledging the request:
		response.acknowledge = True

		#Return the response because thats the pattern.
		return response

#Don't forget the node entrypoint genius
def main(args=None):
	#initialize rclpy
	rclpy.init(args=args)
	
	#create an instance of the data sender:
	dataSender = DataSender()

	#Speeeeeeeeeen forever
	rclpy.spin(dataSender)

	#Shut down in case we need to:
	rclpy.shutdown()

#In case this ever gets run as a script (it won't):
if __name__ == '__main__':
	main()
