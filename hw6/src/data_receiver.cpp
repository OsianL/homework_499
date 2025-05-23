//data_receiver.cpp
//Node which recieves testpackets and calculates the latency

//Include RCL:
#include <rclcpp/rclcpp.hpp>

//Include msg definitionss (different from .msg name because evil):
#include <hw_interfaces/msg/test_packet.hpp>
#include <std_msgs/msg/int64.hpp>

using std::placeholders::_1;

//Node class definition
class DataReceiver : public rclcpp::Node {
public:
	//alias custom message for cleanliness
	using TestPacket = hw_interfaces::msg::TestPacket;
	using Int64 = std_msgs::msg::Int64;

	//Class constructor:
	DataReceiver() : Node("data_receiver") {
		//Create subscriber and publishers
		sub_ = this->create_subscription<TestPacket>("data", 10, std::bind(&DataReceiver::callback, this, _1));
		pub_inst_ = this->create_publisher<Int64>("raw_latency", 10);
		pub_avg_ = this->create_publisher<Int64>("latency", 10);
	
		//initialize ring buffer:
		buf_idx = 0;
	}
	
private:
	//Declarations of sub/pub variables:
	rclcpp::Subscription<TestPacket>::SharedPtr sub_;
	rclcpp::Publisher<Int64>::SharedPtr pub_inst_;
	rclcpp::Publisher<Int64>::SharedPtr pub_avg_;

	//Declarations of latency ring buffer variables:
	const static int buf_len = 10;
	int lat_buf[buf_len] = {-1, -1, -1, -1, -1, -1, -1, -1, -1, -1};
	int buf_idx;

	//Subscriber callback:
	void callback(const TestPacket::SharedPtr msg) {
		
		rclcpp::Time msg_time = rclcpp::Time(msg->send_time);

		//Log that we've recieved a message:
		RCLCPP_INFO(this->get_logger(),"Recieved TestPacket with sent time: %li", msg_time.nanoseconds());
		
		//Create two new messages to send:
		Int64 avg_msg = Int64();
		Int64 inst_msg = Int64();

		//Calculate the instantaneous latency from that message:
		rclcpp::Time current_time = this->get_clock()->now(); 
		lat_buf[buf_idx] = current_time.nanoseconds() - msg_time.nanoseconds();

		//Calculate the average latency from the buffer:
		int loop_sum = 0;
		int loop_samples = 0;

		for (int i = 0; i < buf_len; i++){
			//Gaurding if statement to handle startup
			if (lat_buf[i] == -1){continue;}
		
			//Sum elements and count number of filled in samples
			loop_samples++;
			loop_sum = loop_sum + lat_buf[i];
		}

		int lat_avg = loop_sum / loop_samples;

		//Fill in, Publish, and log new messages:
		avg_msg.data = lat_avg;
		inst_msg.data = lat_buf[buf_idx];

		pub_avg_->publish(avg_msg);
		pub_inst_->publish(inst_msg);

		RCLCPP_INFO(this->get_logger(), "Published inst lat: %li, avg lat: %li", inst_msg.data, avg_msg.data);

		//Increment the ring buffer index:
		buf_idx = (buf_idx + 1) % (buf_len - 1);
	}

};

int main(int argc, char **argv){
	//initialize ROS
	rclcpp::init(argc, argv);

	//Create class instances:
	auto data_receiver = std::make_shared<DataReceiver>();

	//ros go speeeeen;
	rclcpp::spin(data_receiver);

	//shutdown
	rclcpp::shutdown();

	//Return 0 for success:
	return 0;
}
