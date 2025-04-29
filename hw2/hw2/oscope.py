#!/usr/bin/env python3

#Copyright 2025 Osian Leahy

#Licensed under the Apache License, Version 2.0 (the "License");
#you may not use this file except in compliance with the License.
#You may obtain a copy of the License at

#    http://www.apache.org/licenses/LICENSE-2.0

#Unless required by applicable law or agreed to in writing, software
#distributed under the License is distributed on an "AS IS" BASIS,
#WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#See the License for the specific language governing permissions and
#limitations under the License.

# Oscope assignment for ROB 499
#
# oscope.py
#
# Osian Leahy
#

#Import the ros python Node class & infrastructure
import rclpy
from rclpy.node import Node

#Import the python time module rather than trust the timer function for accurate time?
import time

#import math for the sine function
import math

# We're going to publish a Float32, which is a standard ROS message type
from std_msgs.msg import Float32

#Create a class instance inheriting from node
class OscopePublisher(Node):
	def __init__(self):
		# Initialize the parent class, giving it a name.
		super().__init__('oscope')

		# Create a publisher, and assign it to a member variable. The call takes a type, topic name, and queue size.
		self.pub = self.create_publisher(Float32, 'oscope', 200)

		# Rather than setting up a Rate-controller loop, the idiom in ROS2 is to use timers.
		# Timers are available in the Node interface, and take a period (in seconds), and a
		# callback.  Timers are repeating by default.
		self.timer = self.create_timer(0.01, self.callback)

		# Define class variables for the sinusoidal frequency and the output value
		self.frequency = 1 #hz
		self.outputValue = 0

	# This callback will be called every time the timer fires.
	def callback(self):
		# Make an Float32 message, and fill in the information.
		msg = Float32()
		self.outputValue = math.sin(self.frequency * 2 * math.pi * time.time())
		msg.data = self.outputValue

		# Publish the message, just like we do in ROS.
		self.pub.publish(msg)

		# Log that we published something.
		self.get_logger().info(f'Published {self.outputValue}')

# The idiom in ROS2 is to use a function to do all of the setup and work.  This
# function is referenced in the setup.py file as the entry point of the node when
# we're running the node with ros2 run.  The function should have one argument, for
# passing command line arguments, and it should default to None.
def run_node(freq, args=None):
	# Initialize rclpy.
	rclpy.init(args=args)

	# Make an instance of the node class defined above.
	sinePublisher = OscopePublisher()

	#kinda messy to do this here, should probably be in the class init function
	sinePublisher.frequency = freq

	# The spin() call gives control over to ROS2, and it now takes a Node-derived
	# class as a parameter.
	rclpy.spin(sinePublisher)

	# Make sure we shutdown everything cleanly.  This should happen, even if we don't
	# include this line, but you should do it anyway.
	rclpy.shutdown()

#Additional entry points can be defined which allow for different parameters (sine frequencies in this case):

#run the node at 1hz for part 6 of the assignment
def normal_wave(args=None):
	run_node(1.0)

def slow_wave(args=None):
	run_node(0.5)

def fast_wave(args=None):
	run_node(2.0)


# If we run the node as a script, then we're going to start here.
if __name__ == '__main__':
	# The idiom in ROS2 is to set up a main() function and to call it from the entry
	# point of the script.
	normal_wave()
