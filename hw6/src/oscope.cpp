//oscope.cpp
//
// Osian Leahy
//
//Reacreation of the oscope assignment for ROB 499 in Cpp

//Include rclcpp:
#include <rclcpp/rclcpp.hpp>

//include standard message:
#include <std_msgs/msg/float32.hpp>

//include MATH, which is for nerds:
#include <cmath>

//include chrono literals for timing:
#include <chrono>

//namespace gaming:
using namespace std::chrono_literals;

//Create a class for the node:
class OscopePublisher : public rclcpp::Node {

//Public class data can be seen by anyone
public:
	//Constructor function which initializes the class and adds any required data:
	OscopePublisher() : Node("oscope"), frequency_(1) {
		
		//Create a publisher to otuput messages to:
		publisher_ = this->create_publisher<std_msgs::msg::Float32>("oscope", 10); 

		//Create a timer to control the publication frequency.
		//Publish at 100hz using chrono literals & arithmetic.
		//Don't forget std bind, this basically makes a lambda function it can call
		//(std::bind fills in "this" as the first arg of timer_callback, 
		//like doing timer_callback() vs self.timer_callback() in python )
		timer_ = this->create_wall_timer(1s / 100, std::bind(&OscopePublisher::timer_callback, this));

	}

//private class data should only be used/seen within the class
private:
	//We can define our variables down here because we have a 2 pass compiler?
	//I guess the constructor is not called until after the compiler understands the class
	float frequency_;

	//This includes defining the publisher and timer
	rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher_;
	rclcpp::TimerBase::SharedPtr timer_; //Didn't realize this was part of ros/rcl, makes sense.

	//define a rclcpp time object so we can make ze sine waves:
	rclcpp::Time current_time_;

	//The callback that timer uses to publish messages:
	void timer_callback() {
		//Make a new message of the Float32 variety:
		//little unclear to me what I would do without auto here, 
		//I guess Float32() is a function and I could look for it's return type...
		auto message = std_msgs::msg::Float32();
		
		//Get the current time.
		current_time_ = this->get_clock()->now();

		message.data = std::sin(2*M_PI*frequency_*current_time_.seconds());

		//Publish the message:
		publisher_->publish(message);

		//log the published value:
		RCLCPP_INFO(this->get_logger(), "published %f",message.data);
	}
};

//Single point of entry for the node:
int main(int argc, char **argv){
	//init ROS:
	rclcpp::init(argc, argv);

	//Create an instance of our node and save a pointer of it:
	auto oscope = std::make_shared<OscopePublisher>();

	//Pass the node to the ROS event handler via spin:
	rclcpp::spin(oscope);
	
	//shutdown in case we need to:
	rclcpp::shutdown();

	//return 0 because main finished with no errors.
	return 0;

}
