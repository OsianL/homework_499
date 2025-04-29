#Code for ROB 499 Homework 3: Services by Osian Leahy

##To Test/Grade this assignment's code:

First, unzip the provided code into a ros2 workspace source file and run colcon build to compile/install the two packages, hw3 and hw_interfaces (These will be nested inside the unzipped folder).

Second, run the data_sender and data_reciever nodes in seperate terminals using the following commands:
> ros2 run hw3 data_sender
> ros2 run hw3 data_receiver
Note that these nodes will show no log output when initially started.

Next, start data sending data from the data_sender node by running the following node which calls the DataSender service.
> ros2 run hw3 send_enable

At this point, one can echo the /latency and /raw_latency topics to confirm their output:
> ros2 topic echo /latency
> ros2 topic echo /raw_latency

Sending data can be disabled by running:
> ros2 run hw3 send_disable

While sending data, we can also enable local logging in the data_receiver node. This will output the current time, instantaneous latency, and average latency to a .csv file named "local_log.csv" in the present working directory of the termianl the node was started in. 

Local logging can be started by:
> ros2 run hw3 log_enable

And it can be stopped by:
> ros2 run hw3 log_disable
