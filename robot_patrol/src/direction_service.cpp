#include <functional>
#include <cmath>
#include <limits>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "custom_interfaces/srv/get_direction.hpp"
class DirectionService : public rclcpp::Node 
{
public:
    using GetDirection = custom_interfaces::srv::GetDirection;
    DirectionService() 
    : Node("direction_service")
    {
        // Create a service that will handle distance queries
        const std::string service_name = "/direction_service";
        this->service_ = this->create_service<GetDirection>(
            service_name, 
            std::bind(&DirectionService::get_direction_callback, this,
                    std::placeholders::_1, std::placeholders::_2));
        RCLCPP_INFO(this->get_logger(), "%s Service Server Ready...", service_name.c_str());
        // //Create a subscriber that will receive the current laser data
        // auto qos = rclcpp::QoS(10).reliability(rclcpp::ReliabilityPolicy::Reliable);
        // this->laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        //     "fastbot_1/scan",
        //     qos,
        //     std::bind(&Patrol::laserscan_callback, this, std::placeholders::_1)
        // );
    }

private:
    rclcpp::Service<GetDirection>::SharedPtr service_;
    // rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
    // sensor_msgs::msg::LaserScan::SharedPtr latest_laser_data_;
    void get_direction_callback(
        const std::shared_ptr<GetDirection::Request> request,
        std::shared_ptr<GetDirection::Response> response)
    {
        RCLCPP_INFO(this->get_logger(), "Service Requested");
        float total_dist_sec_right = 0;
        float total_dist_sec_front = 0;
        float total_dist_sec_left = 0;
        float front_min = std::numeric_limits<double>::infinity();

        const auto & laser = request->laser_data;
        for (size_t i = 0; i < laser.ranges.size(); i++)
        {
            float angle = laser.angle_min + i * laser.angle_increment;
            float dist = laser.ranges[i];

            if (!std::isfinite(dist)) {
                continue;
            }

            // right section: -90° to -30°
            if (angle >= -M_PI / 2 && angle < -M_PI / 6) {
                total_dist_sec_right += dist;
            }

            // front section: -30° to 30°
            else if (angle >= -M_PI / 6 && angle <= M_PI / 6) {
                total_dist_sec_front += dist;

                if (dist < front_min) {
                    front_min = dist;
                }
            }

            // left section: 30° to 90°
            else if (angle > M_PI / 6 && angle <= M_PI / 2) {
                total_dist_sec_left += dist;
            }
        }

        // front is safe
        if (front_min > 0.35) {
            response->direction = "forward";
        }
        // front is less than 35 cm, choose safer side
        else {
            if (total_dist_sec_left > total_dist_sec_right && total_dist_sec_left > total_dist_sec_front) 
            {
                response->direction = "left";
            } else if(total_dist_sec_right > total_dist_sec_front){
                response->direction = "right";
            } else {
                response->direction = "forward";
            }
        }

        RCLCPP_INFO(
            this->get_logger(),
            "right: %.2f, front: %.2f, left: %.2f, front_min: %.2f -> %s",
            total_dist_sec_right,
            total_dist_sec_front,
            total_dist_sec_left,
            front_min,
            response->direction.c_str()
        );
        RCLCPP_INFO(this->get_logger(), "Service Completed");
    }
    

    // void laserscan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    // {
    //     latest_laser_data = msg;
    // }
};


int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DirectionService>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}