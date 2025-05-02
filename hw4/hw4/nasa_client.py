#!/usr/bin/env python3

#Action client for Launching the rocket.

#pull in rclpy and Action/Node pkgs
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

#Pull in the action definition from hw_interfaces:
from hw_interfaces.action import LaunchRocket

#Create a node class to serve as the client:
class NasaClient(Node):
	#init the node, we will have seperate entry points for showing cancellation
	def __init__(self, with_cancel=False):
		super().__init__('nasa_client')

		#Add an action client:
		self.client = ActionClient(self, LaunchRocket, 'launch_rocket')
		
		#save cancel setting
		self.with_cancel = with_cancel
	
	#function we call to send the goal to the server, starting the action
	def send_goal(self,t):
		#setup the action goal object and length of countdown
		goal = LaunchRocket.Goal()
		goal.countdown = t

		#wait for the server to be ready:
		self.client.wait_for_server()

		#once the server is ready, make the request to launch the rocket. Save the result handle:
		self.result = self.client.send_goal_async(goal, feedback_callback = self.feedback_cb)

		#assign a callback function to the result handle. 
		#this callback is run once the action is accepted or rejected (NOT when the callback is done)
		self.result.add_done_callback(self.response)

	#callback function which processes feedback as it is relayed:
	def feedback_cb(self, feedback_msg):
		#Just Log the feedback to the terminal, and cancel the goal 
		self.get_logger().info(f'T Minus {feedback_msg.feedback.t_remaining} Seconds!')
		
		#If we are demonstrating the action cancellation functionality, cancel at T minus 2:
		if self.with_cancel and feedback_msg.feedback.t_remaining == 5:
			self.get_logger().info('Abort Launch!')
			future = self.goal_handle.cancel_goal_async()
			future.add_done_callback(self.cancel_cb)
	
	#callback function which is assigned when the goal is canceled
	def cancel_cb(self, future):
		#Get the result from the goal handle passed into here?
		response = future.result()

		#Log if the goal successfully canceled
		if len(response.goals_canceling) > 0:
			self.get_logger().info('Launch safely aborted.')
		else:
			self.get_logger().info('Launch failed to abort. :(')

	
	def response(self, future):
		#Get the result of requesting the action. 
		#This is an object whic tells us if the request has begun to be processed
		self.goal_handle = future.result()

		#If it's not accepted, log the failure:
		if not self.goal_handle.accepted:
			self.get_logger().info('Launch Delayed')
			return

		#If the action was acceppted, get a handle and add a callback to process the results once finished:
		self.result_handle = self.goal_handle.get_result_async()
		self.result_handle.add_done_callback(self.process_result)

	#This callback is assigned to the result handle once the goal is accepted. 
	#It processes the results once they're ready
	def process_result(self, future):
		#Get the result by going back to the future:
		result = future.result().result

		#Log it (Result is a string):
		self.get_logger().info(result.launch)

def without_cancel(args=None):
	#init rclpy
	rclpy.init(args=args)

	#setup a node:
	client = NasaClient(with_cancel=False)

	#Make Ze action call for a 10s countdown.
	client.send_goal(10)	

	#give control to ros
	rclpy.spin(client)

	#make sure everything shuts down neatly.
	rclpy.shutdown()

def with_cancel(args=None):

	#init rclpy
	rclpy.init(args=args)

	#setup a node:
	client = NasaClient(with_cancel=True)

	#Make Ze action call for a 10s countdown.
	client.send_goal(10)	

	#give control to ros
	rclpy.spin(client)

	#make sure everything shuts down neatly.
	rclpy.shutdown()
