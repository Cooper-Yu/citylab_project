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
    using GetDirection = custom_interfaces::srv::GetDirection;

    Patrol() 
    : Node("patrol_node"),
      obstacle_ahead_(false),
      current_direction_("forward")
    {
        const std::string service_name = "/direction_service";

        direction_service_client_ = this->create_client<GetDirection>(service_name);

        auto qos = rclcpp::QoS(10).reliability(rclcpp::ReliabilityPolicy::Reliable);  

        laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "fastbot_1/scan",
            qos,
            std::bind(&Patrol::laserscan_callback, this, std::placeholders::_1)
        );

        twist_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "fastbot_1/cmd_vel", 
            10
        );

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&Patrol::control_loop, this)
        );

        RCLCPP_INFO(this->get_logger(), "Patrol node started.");
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Client<GetDirection>::SharedPtr direction_service_client_;

    bool obstacle_ahead_;
    std::string current_direction_;

    bool is_obstacle_range(float r) const
    {
        return std::isfinite(r) && r < 0.35f;   
    }

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

    void laserscan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        obstacle_ahead_ = false;

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

        if (!obstacle_ahead_) {
            current_direction_ = "forward";
            return;
        }

        if (!direction_service_client_->service_is_ready()) {
            RCLCPP_WARN(this->get_logger(), "Direction service not ready.");
            return;
        }

        auto request = std::make_shared<GetDirection::Request>();
        request->laser_data = *msg;

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