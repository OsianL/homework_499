#Readme file for homework 4 of ROB 499 at Oregon State Univeristy

###Instructions for Use/Testing/Grading:

First, unzip the provided code into a ros2 workspace src folder and run colcon build to compile/install the hw4 and hw_interfaces packages.

Second, run the wave.launch.py launch file, which will open 3 oscope nodes of 1,5, and 10 hz frequencies, along with plotjuggler.

> ros2 launch hw4 wave.launch.py

Third, subscribe to the /one_hz, /five_hz, and /ten_hz topics in plot juggler to see the output. 
By default, none of the nodes will be publishing data. After executing the required service call for 
publishing, you will have to go back and start plotting the topics in plotjuggler

Fourth, enable publishing of the oscope nodes by running the following service calls:

> ros2 service call /send_data_one hw_interfaces/srv/SendData "{send: True}"
> ros2 service call /send_data_five hw_interfaces/srv/SendData "{send: True}"
> ros2 service call /send_data_ten hw_interfaces/srv/SendData "{send: True}"

Fifth, enable publishing of the oscope nodes by running the following service calls:
Third, test the nasa action server and clients. The server can be launched via:

> ros2 run hw4 nasa

Two action clients are available which demonstrate the cancelability of actions. To launch the rocket, run:

> ros2 run hw4 nasa_client_launch

To attempt and abort the launch, run:

> ros2 run hw4 nasa_client_launch_cancel


