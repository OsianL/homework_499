# Osian leahy's Homework submission for ROB499 HW5.

To test this code:
1. Install Scikit-learn using the instructions for ubuntu/debian here (sudo apt-get rather than pip):
> https://scikit-learn.org/stable/install.html

2. Unzip and Build the package like normal

3. launch all required nodes and rviz using the following:
> ros2 launch hw5 people_detection.launch.py
	
	Note: Rviz should automatically launch with the relevant config file for this assignment. If it doesn't, please load it from the hw5 package root directory: "rviz_hw5.rviz"

4. Start the bag file using the followign command, be sure to edit the path to the bag file:
> ros2 bag play --playback-duration 43 <PATH-TO-BAG>/hw5_good_good/

5. Relatively quickly after starting the bag file, call the service to grab a background snapshot of the laserscan. Ideally this should be done before anyone appears on the scan:

> ros2 service call /scan_snapshot std_srvs/srv/Trigger

6. Observe the results in Rviz2:
	- Small red points are the unfiltered laser scan
	- Small purple points are the filterd laser scan
	- White points following clusters of purple points are the detected people
	- The flat centered green cylinder represents the robot's "personal space"
	- Tall Blue cylinders which monetarily appear sticking out of the robot's personal space are "intruders"

7. Confirm the saved pictures of intruders, some of which may be empty if intruders walk to the side of the robot. These will be saved in the directory you ran the launch file from.


