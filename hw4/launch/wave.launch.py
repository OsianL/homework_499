#Python launch file for HW4. 
#This launches 3 instances of oscope node which publish at different frequencies, 
#as well as an instance of plot Juggler for viewing purposes.

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
	return LaunchDescription([
		Node(
			package = 'hw4',
			executable = 'oscope', #This is the node name in setup.py
			name = 'one_hz_scope',
			output = 'screen',
			emulate_tty = True,
			parameters = [
				{'frequency' : 1.0}
			],
			remappings = [
				('/oscope', '/one_hz'),
				('/send_data', '/send_data_one')
			]
		),

		Node(
			package = 'hw4',
			executable = 'oscope',
			name = 'five_hz_scope',
			output = 'screen',
			emulate_tty = True,
			parameters = [
				{'frequency' : 5.0}
			],
			remappings = [
				('/oscope', '/five_hz'),
				('/send_data', '/send_data_five')
			]
		),

		Node(
			package = 'hw4',
			executable = 'oscope',
			name = 'ten_hz_scope',
			output = 'screen',
			emulate_tty = True,
			parameters = [
				{'frequency' : 10.0},
				{'clamp' : 0.7}
			],
			remappings = [
				('/oscope', '/ten_hz'),
				('/send_data', '/send_data_ten')
			]
		),

		Node(
			package = 'plotjuggler',
			executable = 'plotjuggler',
			name = 'plotjuggler'
		)
		
	])
