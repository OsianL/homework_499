#!/usr/env/bin python3

#Launch file for hw5, opens the relevant nodes and rviz config. 

from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
import os

def generate_launch_description():
	
	return LaunchDescription([
		#Launch the filter node
		Node(
			package = 'hw5',
			executable = 'laser_filter',
			name = 'laser_filter',
			parameters = [
				{'epsilon' : 0.1}
			],
			remappings = [
				('/input_scan', '/scan'),
				('/output_scan', '/filtered_scan')
			]
		),
		#Launch the person detector node
		Node(
			package = 'hw5',
			executable = 'people_detector',
			name = 'people_detector',
			remappings = [
				('/input_scan','/filtered_scan'),
				('/output_points', '/people_points')
			]
		),
		#Launch the personal space node
		Node(
			package = 'hw5',
			executable = 'personal_space',
			name = 'personal_space'
		),
		#Launch the camera capture Node
		Node(
			package = 'hw5',
			executable = 'camera_capture',
			name = 'camera_capture',
			remappings = [
				('/camera_feed', '/astra_ros/devices/default/color/image_color'),
			]
		),
		#Launch Rviz2:
		Node(
			package = 'rviz2',
			executable = 'rviz2',
			name = 'rviz2',
			arguments = ['-d' +  os.path.join(get_package_share_directory('hw5'),'rviz_hw5.rviz')]
		)

	])
