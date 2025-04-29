#!/usr/bin/env python3

#Service clients for HW3, which interace with the SendData and EnableLogging services 
#implemented in send_data.py and receive_data.py

#Import rclpy and Node as is tradition:
import rclpy
from rclpy.node import Node

#We will be implementing these two services:
from hw_interfaces.srv import SendData

class SendServiceClient(Node):
	def __init__(self):
		super().__init__('client')

		#setup service clients with types and names:
		self.client = self.create_client(SendData,'send_data')

		#Wait until connected to the server, log timeout messages ever ~second until connected:
		while not self.client.wait_for_service(timeout_sec=1):
			self.get_logger().info('waiting for server to connect')

	def send_data_request(self,send):
		#create the request to send based on the user input:
		request = SendData.Request()
		request.send = send

		#service calls do be async for some reason 
		#(which is why the response should be a class variable)
		self.response = self.client.call_async(request)

#Define the main client behaviors:
def send_main(send, args=None):
	#init rclpy
	rclpy.init(args=args)

	#create the client node
	client = SendServiceClient()

	#send the request
	client.send_data_request(send)

	#Delay while we wait for the request response
	#This forces the service call to be synchronous
	while rclpy.ok():
		#allow rclpy to do it's stuff:
		rclpy.spin_once(client)

		#Check if we have a response
		if client.response.done():
			#Do a try, except, else in case the response is garbled or we're slighty early:
			try:
				#get the response:
				ret = client.response.result()
			except Exception as e:
				#log the exception if things fail:
				client.get_logger().error(f'Service Call failed: {e}')
			else:
				#if everything's fine, log the result:
				client.get_logger().info(f'Requested send_data={send}, got {ret.acknowledge} acknowledging')
				break #this breaks the while loop
	#Shut the node down:
	rclpy.shutdown()

#Define some entry points based on what we want to do:
def send_data_enable(args=None):
	send_main(True,args)

def send_data_disable(args=None):
	send_main(False,args)

