
//cpp implementation of a ROS2 transformer node which limits/truncates 
//a Float32 value to within a specified window

//Include rclcpp, standard messages, and chrono literals
#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/float32.hpp>

#include <chrono>

//namespace gaming
using namespace std::chrono_literals;
using std::placeholders::_1;

//define the node class:
class Limiter : public rclcpp::Node {
public:
	//Constructor for the class:
	Limiter() : Node("limiter") {
		
		//Create publisher and subscriber:
		publisher_ = this->create_publisher<std_msgs::msg::Float32>("limited", 10);
		subscription_ = this->create_subscription<std_msgs::msg::Float32>("oscope", 10, std::bind(&Limiter::sub_callback, this, _1));
		
		//add a paramter for the limiter threshold:
		this->declare_parameter("limit", 0.5);
	}

private:
	//define our variables, including publisher/subscription objects
	rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher_;
	rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr subscription_;

	//write our subscriber callback:
	void sub_callback(const std_msgs::msg::Float32::SharedPtr msg) {
		//make a new message to save our output to:
		std_msgs::msg::Float32 new_msg = std_msgs::msg::Float32();

		//Get the current value of the parameter:
		double limit = this->get_parameter("limit").as_double();

		//Truncate the output based on the limit parameter:
		if (limit < msg->data){
			new_msg.data = limit;
		} else if (-limit > msg->data) {
			new_msg.data = -limit;
		} else {
			new_msg.data = msg->data;
		}

		//Republish the truncated value:
		publisher_->publish(new_msg);

		//Log whatever we published:
		RCLCPP_INFO(this->get_logger(), "Recieved %f, Published %f",msg->data, new_msg.data);
	}
};

//Single entry point via main:
int main(int argc, char **argv){
	//initialize rcl
	rclcpp::init(argc, argv);

	//create an instance of the class:
	auto limiter = std::make_shared<Limiter>();

	//Speeeeeen
	rclcpp::spin(limiter);

	//just in case:
	rclcpp::shutdown();

	return 0;
}
