#include <functional>
#include <cmath>
#include <limits>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "custom_interfaces/srv/get_direction.hpp"
class DirectionService : public rclcpp::Node 
{
public:
    // Alias for service type
    using GetDirection = custom_interfaces::srv::GetDirection;
    DirectionService() 
    : Node("direction_service")
    {
        // Create a service server that listens on /direction_service
        const std::string service_name = "/direction_service";
        this->service_ = this->create_service<GetDirection>(
            service_name, 
            std::bind(&DirectionService::get_direction_callback, this,
                    std::placeholders::_1, std::placeholders::_2));
        // Log that the server is ready
        RCLCPP_INFO(this->get_logger(), "%s Service Server Ready...", service_name.c_str());
        
    }

private:
    // Service server object
    rclcpp::Service<GetDirection>::SharedPtr service_;

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

    // Callback executed every time a request is received
    void get_direction_callback(
        const std::shared_ptr<GetDirection::Request> request,
        std::shared_ptr<GetDirection::Response> response)
    {
        RCLCPP_INFO(this->get_logger(), "Service Requested");
        // Variables to accumulate distances in each section
        float total_dist_sec_right = 0;
        float total_dist_sec_front = 0;
        float total_dist_sec_left = 0;

        // Extract LaserScan data from request
        const auto & laser = request->laser_data;

        // Loop through all laser rays
        for (size_t i = 0; i < laser.ranges.size(); i++)
        {
            float raw_angle = laser.angle_min + i * laser.angle_increment;
            double angle = normalize_angle(raw_angle);
            float dist = laser.ranges[i];
            // Ignore invalid readings (inf or NaN)
            if (!std::isfinite(dist)) {
                continue;
            }
            // Divide the space into 3 sections of 60° each
            // right section: -90° to -30°
            if (angle >= -M_PI / 2 && angle < -M_PI / 6) {
                total_dist_sec_right += dist;
            }

            // front section: -30° to 30°
            else if (angle >= -M_PI / 6 && angle <= M_PI / 6) {
                total_dist_sec_front += dist;

            }

            // left section: 30° to 90°
            else if (angle > M_PI / 6 && angle <= M_PI / 2) {
                total_dist_sec_left += dist;
            }
        }
        // Decision logic:
        // Choose the direction with the largest accumulated distance
        if (total_dist_sec_front >= total_dist_sec_left &&
            total_dist_sec_front >= total_dist_sec_right)
        {
            response->direction = "forward";
        }
        else if (total_dist_sec_left > total_dist_sec_right)
        {
            response->direction = "left";
        }
        else
        {
            response->direction = "right";
        }
        // Log the computed totals and chosen direction
        RCLCPP_INFO(
            this->get_logger(),
            "right: %.2f, front: %.2f, left: %.2f -> %s",
            total_dist_sec_right,
            total_dist_sec_front,
            total_dist_sec_left,
            response->direction.c_str()
        );
        // Indicate service completion
        RCLCPP_INFO(this->get_logger(), "Service Completed");
    }
    

    
};


int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DirectionService>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}