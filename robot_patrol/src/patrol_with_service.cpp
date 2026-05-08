#include <rclcpp/rclcpp.hpp>
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/qos.hpp"
#include "custom_interfaces/srv/get_direction.hpp"

#include <cmath>
#include <functional>
#include <chrono>
#include <string>

class Patrol : public rclcpp::Node 
{
public: 
    // Alias for the service type
    using GetDirection = custom_interfaces::srv::GetDirection;

    Patrol() 
    : Node("patrol_with_service_node"),
      obstacle_ahead_(false),
      current_direction_("forward")
    {   // Create a client to connect to /direction_service
        const std::string service_name = "/direction_service";

        direction_service_client_ = this->create_client<GetDirection>(service_name);
        // QoS configuration for reliable communication
        auto qos = rclcpp::QoS(10).reliability(rclcpp::ReliabilityPolicy::Reliable);  
        // Subscribe to LaserScan topic
        laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan",
            qos,
            std::bind(&Patrol::laserscan_callback, this, std::placeholders::_1)
        );
        // Publisher for robot velocity
        twist_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "/cmd_vel", 
            10
        );
        // Timer to continuously send velocity commands
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&Patrol::control_loop, this)
        );

        RCLCPP_INFO(this->get_logger(), "Patrol node started.");
    }

private:
    // ROS interfaces
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Client<GetDirection>::SharedPtr direction_service_client_;
    // State variables
    bool obstacle_ahead_;
    std::string current_direction_;
    // Check if a ray indicates an obstacle (< 0.35m)
    bool is_obstacle_range(float r) const
    {
        return std::isfinite(r) && r < 0.35f;   
    }
    // Normalize angle to range [-pi, pi]
    double normalize_angle(double angle)
    {
        while (angle > M_PI) {
            angle -= 2.0 * M_PI;
        }
        while (angle < -M_PI) {
            angle += 2.0 * M_PI;
        }
        return angle;
    }
    // Callback for LaserScan data
    void laserscan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        obstacle_ahead_ = false;
        // Check front window for obstacles
        for (size_t i = 0; i < msg->ranges.size(); ++i) {
            float r = msg->ranges[i];

            if (!std::isfinite(r)) {
                continue;
            }

            double raw_angle = msg->angle_min + static_cast<double>(i) * msg->angle_increment;
            double angle = normalize_angle(raw_angle);

            // front window: [-20°, +20°]
            if (angle >= -M_PI / 9.0 && angle <= M_PI / 9.0) {
                if (is_obstacle_range(r)) {
                    obstacle_ahead_ = true;
                    break;
                }
            }
        }
        // If no obstacle, move forward directly
        if (!obstacle_ahead_) {
            current_direction_ = "forward";
            return;
        }
        // Check if service is ready
        if (!direction_service_client_->service_is_ready()) {
            RCLCPP_WARN(this->get_logger(), "Direction service not ready.");
            return;
        }
        // Prepare service request with laser data
        auto request = std::make_shared<GetDirection::Request>();
        request->laser_data = *msg;
        // Call service asynchronously
        direction_service_client_->async_send_request(
            request,
            [this](rclcpp::Client<GetDirection>::SharedFuture future)
            {
                auto response = future.get();
                current_direction_ = response->direction;

                RCLCPP_INFO(
                    this->get_logger(),
                    "Direction service response: %s",
                    current_direction_.c_str()
                );
            }
        );
    }
    // Control loop that publishes velocity commands
    void control_loop()
    {
        geometry_msgs::msg::Twist cmd;

        if (current_direction_ == "forward") {
            cmd.linear.x = 0.1;
            cmd.angular.z = 0.0;
        }
        else if (current_direction_ == "left") {
            cmd.linear.x = 0.1;
            cmd.angular.z = 0.5;
        }
        else if (current_direction_ == "right") {
            cmd.linear.x = 0.1;
            cmd.angular.z = -0.5;
        }
        else {
            cmd.linear.x = 0.0;
            cmd.angular.z = 0.0;
        }

        twist_pub_->publish(cmd);
    }
};

int main(int argc, char** argv) 
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Patrol>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}