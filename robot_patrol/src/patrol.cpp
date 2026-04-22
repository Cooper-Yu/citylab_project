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
        

        RCLCPP_INFO(this->get_logger(), "Patrol node started.");
    
    }

private:
    double direction_;
    int best_index_;
    bool obstacle_ahead_;

};



int main(int argc, char** argv) 
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<Patrol>();

    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;

}


