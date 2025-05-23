#Code for ROB499 HW6, a recreation of homeworks 2 and 3 in cpp.

##To Test/Grade this code, do the following:

1. unzip the hw6.zip file into a ros2 workspace src directory.

2. Build the code using colcon build

3. To test the oscope and limiter nodes, run the oscope_launch.xml launch file using the command below:
> ros2 launch hw6 oscope_launch.xml
4. Confirm that everything is running as epected via topic list/listen

5. To test the data sender nodes, run the data_launch.xml launch file using:
> ros2 launch hw6 data_launch.xml
6. Click through the plotjuggler prompts, and confirm that everything is running as expected by viewing the average/instantaneous latencies in plot juggler. Note the lower latency of the cpp nodes.

7. Confirm that the SendData service works by stopping one of the data_sender nodes. (the SendData service call is also used in the launch file):
> ros2 service call /python/send_data hw_interfaces/srv/SendData.srv "{send: True}"

All done. Yay!
