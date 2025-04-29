# Readme fie for homework 2 of ROB 499 at Oregon State University.

## Following are instructions on how to use the ROS2 Nodes contained in this package to generate a sine wave cropped to the range of -0.5 to 0.5. This wave can then be viewed using a tool like PlotJuggler.

**First,** the package should be unzipped into a ROS2 Jazzy workspace and built using colcon build. Then, the ros workspace install should be sourced.

**Second,** a node can be started to generate a 1hz sine wave. This should be done with the following command:
> ros2 run hw2 oscope

**Third,** a node can be started to recieve and limit the outgoing 1hz sinewave. This can be done with the limiter node, with the input topic (/number) remapped to the output topic of the sine wave (/oscope):
> ros2 run hw2 limiter --ros-args --remap /number:=/oscope

The result will be logged by the limiter node and published to a topic called /new_number. It can then be plotted using PlotJuggler.
