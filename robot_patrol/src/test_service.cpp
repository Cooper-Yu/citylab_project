#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "custom_interfaces/srv/get_direction.hpp"

using namespace std::chrono_literals;

class DirectionServiceClient : public rclcpp::Node 
{
public:
    using GetDirection = custom_interfaces::srv::GetDirection;

    DirectionServiceClient() 
    : Node("direction_service_client")
    {
        // Create the Service Client object
        std::string name_service = "/direction_service";
        client_= this->create_client<GetDirection>(name_service);
        //Create a subscriber that will receive the current laser data
        auto qos = rclcpp::QoS(10).reliability(rclcpp::ReliabilityPolicy::Reliable);
        this->laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan",
            qos,
            std::bind(&DirectionServiceClient::laserscan_callback, this, std::placeholders::_1)
        );

        // Wait for the service to be available (checks every second)
        while (!client_->wait_for_service(1s)) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "Service %s not available, waiting again...", name_service.c_str());
        }
        RCLCPP_INFO(this->get_logger(), "Service Client Ready");
    }
private:
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
    sensor_msgs::msg::LaserScan::SharedPtr latest_laser_data_;
    rclcpp::Client<GetDirection>::SharedPtr client_;

    void laserscan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        latest_laser_data_ = msg;
        send_request();
    }
public:
    void send_request()
    {
        // Create an empty GetDistance request
        auto request = std::make_shared<GetDirection::Request>();
        
        request->laser_data  = *latest_laser_data_;

        RCLCPP_INFO(this->get_logger(), "Service Request");


        // Send the request asynchronously
        auto result_future = client_->async_send_request(request);

        //Asynchronous request sending
        client_->async_send_request(
            request,
            [this](rclcpp::Client<GetDirection>::SharedFuture future)
            {
                auto response = future.get();

                RCLCPP_INFO(
                    this->get_logger(),
                    "Service Response: %s",
                    response->direction.c_str()
                );
            }
        ); 
    }
};

int main(int argc, char** argv)
{
    // Initialize the ROS communication
    rclcpp::init(argc, argv);
    
    // Declare the node constructor
    auto client = std::make_shared<DirectionServiceClient>();
    
    rclcpp::spin(client);
    
    
    // Shutdown the ROS communication
    rclcpp::shutdown();
    return 0;
}