#include <rclcpp/rclcpp.hpp>
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/qos.hpp"
#include <cmath>
#include <algorithm>
#include <functional>
#include <chrono>

class Patrol : public rclcpp::Node 
{
public:   
    Patrol() 
    : Node("patrol_node"),
      direction_(0.0), 
      obstacle_ahead_(false)
    {
        auto qos = rclcpp::QoS(10).reliability(rclcpp::ReliabilityPolicy::Reliable);  

        laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan",
            qos,
            std::bind(&Patrol::laserscan_callback, this, std::placeholders::_1)
        );

        twist_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

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

    double direction_;
    bool obstacle_ahead_;
    // Filter out rays with a value of inf
    bool is_finite_range(float r) const
    {
        return std::isfinite(r);
    }
    // Detecting obstacles
    bool is_obstacle_range(float r) const
    {
        return std::isfinite(r) && r < 0.35f;   
    }
    // Convert the angle range to -pi ~ pi
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
        const auto& ranges = msg->ranges;

        obstacle_ahead_ = false;

        double best_angle = 0.0;
        float max_distance = -1.0f;

        for (size_t i = 0; i < ranges.size(); ++i) {
            float r = ranges[i];

            if (!is_finite_range(r)) {
                continue;
            }

            double raw_angle = msg->angle_min + static_cast<double>(i) * msg->angle_increment;

            // Real-world robots might be 0 ~ 2pi, but here we'll convert them to -pi ~ pi.
            double angle = normalize_angle(raw_angle);

            // 1. Directly in front (window): [-20°, +20°]
            if (angle >= -M_PI / 9.0 && angle <= M_PI / 9.0) {
                if (is_obstacle_range(r)) {
                    obstacle_ahead_ = true;
                }
            }

            // 2. Only find the farthest ray within a 180° range in front.
            if (angle >= -M_PI / 2.0 && angle <= M_PI / 2.0) {
                if (r > max_distance) {
                    max_distance = r;
                    best_angle = angle;
                }
            }
        }

        direction_ = best_angle;
        // Print basic information such as the direction of obstacles.
        RCLCPP_INFO_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            1000,
            "obstacle_ahead=%s, direction=%.3f rad",
            obstacle_ahead_ ? "true" : "false",
            direction_
        );
    }

    // The motion logic of the car
    void control_loop()
    {
        geometry_msgs::msg::Twist cmd;

        if (!obstacle_ahead_) {
            // no obstacle aheard, move forward
            cmd.linear.x = 0.1;
            cmd.angular.z = 0.0;
        }
        else {
            // There is an obstacle ahead; turn towards the direction of the furthest ray
            cmd.linear.x = 0.1;
            cmd.angular.z = direction_ / 2;
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