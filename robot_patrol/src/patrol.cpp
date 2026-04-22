#include <rclcpp/rclcpp.hpp>
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/qos.hpp"
#include <cmath>
#include <limits>
#include <vector>
#include <algorithm>
#include <functional>
#include <chrono>

class Patrol : public rclcpp::Node 
{
public:   
    Patrol() 
    : Node("patrol_node"),
      direction_(0.0), 
      best_index_(-1),
      obstacle_ahead_(false)
    {
        auto qos = rclcpp::QoS(10).reliability(rclcpp::ReliabilityPolicy::Reliable);  
        laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "fastbot_1/scan",
            qos,
            std::bind(&Patrol::laserscan_callback, this, std::placeholders::_1)
        );
        twist_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("fastbot_1/cmd_vel", 10);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&Patrol::control_loop, this)
        );

        RCLCPP_INFO(this->get_logger(), "Patrol node started.");
    
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr  twist_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    double direction_;
    int best_index_;
    bool obstacle_ahead_;


    bool is_finite_range(float r) const
    {
        return std::isfinite(r);
    }

    bool is_obstacle_range(float r) const
    {
        return std::isfinite(r) && r < 0.35f;
    }

    void laserscan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        const auto & laser_ranges = msg->ranges;

        int start_index = static_cast<int>((-M_PI / 2.0 - msg->angle_min) / msg->angle_increment);
        int end_index   = static_cast<int>(( M_PI / 2.0 - msg->angle_min) / msg->angle_increment);

        start_index = std::max(0, start_index);
        end_index   = std::min(static_cast<int>(laser_ranges.size()) - 1, end_index);

        int obstacle_start_index = static_cast<int>((-M_PI / 9 - msg->angle_min) / msg->angle_increment);
        int obstacle_end_index = static_cast<int>((M_PI / 9 - msg->angle_min) / msg->angle_increment);
        for (int i = obstacle_start_index; i <= obstacle_end_index; ++i) {
            float r = laser_ranges[i];
            if (is_obstacle_range(r)) {
                obstacle_ahead_ = true;
                break;
            } 
            else {
                obstacle_ahead_ = false;
            }
        }
        if (obstacle_ahead_) {
            float max_distance = -1.0f;

            for (int i = start_index; i <= end_index; ++i) {
                float r = laser_ranges[i];

                if (!is_finite_range(r)) {
                    continue;
                }

                if (r > max_distance) {
                    max_distance = r;
                    best_index_ = i;
                }
            }

            if (best_index_ != -1) {
                direction_ = msg->angle_min + best_index_ * msg->angle_increment;
            }
        }

        RCLCPP_INFO_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            1000,
            "obstacle_ahead=%s, direction=%.3f rad",
            obstacle_ahead_ ? "true" : "false",
            direction_
        );
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


