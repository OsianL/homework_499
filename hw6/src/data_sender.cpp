//Recreation of hw3 data_sender.py in cpp

//include rcl and chrono literals 
#include <rclcpp/rclcpp.hpp>
#include <chrono>

//chrono literal namespae:
using namespace std::chrono_literals;

//include our custom message/service
#include <hw_interfaces/msg/test_packet.hpp> //I guess this gets generated by colcon build
#include <hw_interfaces/srv/send_data.hpp>

//Placeholders needed for setting up the service callback:
using std::placeholders::_1;
using std::placeholders::_2;

//Node which inherits from Node (Woah)
class DataSender : public rclcpp::Node {
public:
	//alias the message and service types:
	using SendData = hw_interfaces::srv::SendData;
	using TestPacket = hw_interfaces::msg::TestPacket;

	//Node constructor:
	DataSender() : Node("data_sender"), sending_(false) {
		
		//Create the service
		service_ = this->create_service<SendData>("send_data", std::bind(&DataSender::service_callback, this, _1, _2));
		
		//Create the publisher:
		publisher_ = this->create_publisher<TestPacket>("data", 10);

		//Create a timer for the publisher, publish at 1hz:
		timer_ = this-> create_wall_timer(1s, std::bind(&DataSender::timer_callback, this));
	}

	//runs when called by external service
	void service_callback(
		const std::shared_ptr<SendData::Request> request,
		const std::shared_ptr<SendData::Response> response){

		//Update the sending data variable:
		sending_ = request->send;
		
		//Log the result:
		RCLCPP_INFO(this->get_logger(), "Got service request, sending packets now: %d", sending_);

		//Update the response:
		response->acknowledge = true;
	}

	//Runs periodically, sending dummy packets
	void timer_callback(){
		if (sending_){	
			rclcpp::Time current_time = this->get_clock()->now();

			//Generate the test packet with timestamp
			TestPacket packet = TestPacket();
			packet.send_time = current_time;
			packet.payload = {0, 1, 2, 3};

			//publish the test packet
			publisher_->publish(packet);

			//log that it was published
			RCLCPP_INFO(this->get_logger(), "published at time %li",current_time.nanoseconds());
		}
	}

private:
	//service and publisher pointers:
	rclcpp::Service<SendData>::SharedPtr service_;
	rclcpp::Publisher<TestPacket>::SharedPtr publisher_;
	
	//timer pointer:
	rclcpp::TimerBase::SharedPtr timer_;

	//Class variables:
	bool sending_;
};

int main(int argc, char **argv) {
	//Initialize rclcpp:
	rclcpp::init(argc, argv);

	//create shared pointer to instance of class 
	auto data_sender = std::make_shared<DataSender>();

	//speeeeen
	rclcpp::spin(data_sender);

	//die
	rclcpp::shutdown();

}
