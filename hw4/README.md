#Readme file for homework 4 of ROB 499 at Oregon State Univeristy

###Instructions for Use/Testing/Grading:

First, unzip the provided code into a ros2 workspace src folder and run colcon build to compile/install the hw4 and hw_interfaces packages.

Second, run the wave.launch.py launch file, which will open 3 oscope nodes of 1,5, and 10 hz frequencies, along with plotjuggler.

> ros2 launch hw4 wave.launch.py

Third, test the nasa action server and clients. The server can be launched via:

> ros2 run hw4 nasa

Two action clients are available which demonstrate the cancelability of actions. To launch the rocket, run:

> ros2 run hw4 nasa_client_launch

To abort the launch, run:

> ros2 run hw4 nasa_client_launch_cancel


