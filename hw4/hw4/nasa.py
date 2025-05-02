#!/usr/bin/env python3

# Action server for ROB499 HW4

#rclpy and specific Node/Action packages
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse

#Needed for action cancellation
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

#get our action definition
from hw_interfaces.action import LaunchRocket

#use a sleep because we're lazy
from time import sleep

class Nasa(Node):
	def __init__(self):
		super().__init__('nasa')

		#Setup the action server
		#arguments are self, Action Type, action name, action callback, callback group and cancel callback
		self.server = ActionServer(self, LaunchRocket, 'launch_rocket', self.launch_callback,
								callback_group = ReentrantCallbackGroup(), cancel_callback = self.cancel_callback)
		

	#Callback for the action server
	def launch_callback(self, goal):
		#first, log that we've recieved an action call
		self.get_logger().info(f'Beginning Countdown. Launch in T-{goal.request.countdown}')

		#Build a result that we can send back
		result = LaunchRocket.Result()
		result.launch = "LAUNCHING ZE ROCKET!"

		#delay while periodically sending countdown feedback
		counter = goal.request.countdown
		#Check if goal is canceled, return an abort string if it is.
		for i in range(goal.request.countdown):
			if goal.is_cancel_requested:
				goal.canceled()
				self.get_logger().info('LAUNCH ABORTED!')
				result.launch = "ABORTING ZE LAUNCH!"
				return result
			
			#otherwise, sleep for a second and then return the countdown as feedback
			sleep(1.0)
			counter -= 1
			self.get_logger().info(f'T-{counter}')
			goal.publish_feedback(LaunchRocket.Feedback(t_remaining = counter))

		#once we're out of the for loop, send the result back:
		goal.succeed()
		self.get_logger().info(result.launch)
		sleep(1.0)
		self.print_rocket()
		return result

	#Callback for canceling the action
	def cancel_callback(self, goal_handle):
		self.get_logger().info('ABORTING ZE LAUNCH!')
		return CancelResponse.ACCEPT

	def print_rocket(self):
		ascii_art = r"""
		       _________
		      (=========)
		      |=========|
		      |====_====|
		      |== / \ ==|
		      |= / _ \ =|
		   _  |=| ( ) |=|
		  /=\ |=|     |=| /=\
		  |=| |=| USA |=| |=|
		  |=| |=|  _  |=| |=|
		  |=| |=| | | |=| |=|
		  |=| |=| | | |=| |=|
		  |=| |=| | | |=| |=|
		  |=| |/  | |  \| |=|
		  |=|/    | |    \|=|
		  |=/NASA |_| NASA\=|
		  |(_______________)|
		  |=| |_|__|__|_| |=|
		  |=|   ( ) ( )   |=|
		 /===\           /===\
		|||||||         |||||||
		-------         -------
		 (~~~)           (~~~)
		"""

		self.get_logger().info(ascii_art)


#entrypoint for the node
def main(args=None):
	#Init rclpy
	rclpy.init(args=args)

	#Setup an instance of the node:
	server = Nasa()

	rclpy.spin(server, MultiThreadedExecutor())

	#Shut down afterwards:
	rclpy.shutdown()

