#include <functional>
#include <memory>
#include <thread>
#include <chrono>
#include <cmath>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "robot_patrol/action/go_to_pose.hpp"

#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/quaternion.hpp"

#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

// ==============================
// GoToPose Action Server Node
// ==============================
class GoToPose : public rclcpp::Node
{
public:
    // Alias for action type
    using GoToPoseAction = robot_patrol::action::GoToPose;
    // Alias for goal handle
    using GoalHandleGoToPose = rclcpp_action::ServerGoalHandle<GoToPoseAction>;
    // Alias for Pose2D message
    using Pose2D = geometry_msgs::msg::Pose2D;

    // Constructor: initialize node, action server, subscriber, publisher
    explicit GoToPose(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
    : Node("go_to_pose_action_server", options)
    {
        using namespace std::placeholders;
        // Create action server
        action_server_ = rclcpp_action::create_server<GoToPoseAction>(
            this,
            "/go_to_pose",
            std::bind(&GoToPose::handle_goal, this, _1, _2),
            std::bind(&GoToPose::handle_cancel, this, _1),
            std::bind(&GoToPose::handle_accepted, this, _1)
        );
        // QoS setting for reliable communication
        auto qos = rclcpp::QoS(10).reliability(rclcpp::ReliabilityPolicy::Reliable);
        // Subscribe to odometry data (robot position)
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom",
            qos,
            std::bind(&GoToPose::odom_callback, this, _1)
        );
        // Publisher to control robot velocity
        twist_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "/cmd_vel",
            10
        );
        RCLCPP_INFO(this->get_logger(), "Action Server Ready");
    }

private:
    // Desired goal position (from action request)
    Pose2D desired_pos_;
    // Current robot position (from odometry)
    Pose2D current_pos_;
    // ROS2 interfaces
    rclcpp_action::Server<GoToPoseAction>::SharedPtr action_server_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;
    
    // ==============================
    // Odometry callback
    // ==============================
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        current_pos_.x = msg->pose.pose.position.x;
        current_pos_.y = msg->pose.pose.position.y;
        current_pos_.theta = get_yaw_from_quaternion(msg->pose.pose.orientation);
    }

    // ==============================
    // Convert quaternion to yaw angle
    // ==============================
    double get_yaw_from_quaternion(const geometry_msgs::msg::Quaternion & q_msg)
    {
        tf2::Quaternion q(
            q_msg.x,
            q_msg.y,
            q_msg.z,
            q_msg.w
        );

        tf2::Matrix3x3 m(q);

        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        return yaw;
    }
    // ==============================
    // Normalize angle to [-pi, pi]
    // ==============================
    double normalize_angle(double angle)
    {
        while (angle > M_PI)
        {
            angle -= 2.0 * M_PI;
        }

        while (angle < -M_PI)
        {
            angle += 2.0 * M_PI;
        }

        return angle;
    }

    // ==============================
    // Handle incoming goal request
    // ==============================
    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const GoToPoseAction::Goal> goal)
    {
        (void) uuid;
        // Store goal position
        desired_pos_.x = goal->goal_pos.x;
        desired_pos_.y = goal->goal_pos.y;
        desired_pos_.theta = goal->goal_pos.theta;

        RCLCPP_INFO(this->get_logger(), "Action Called");

        RCLCPP_INFO(
            this->get_logger(),
            "Received goal: x=%.2f, y=%.2f, theta=%.2f",
            desired_pos_.x,
            desired_pos_.y,
            desired_pos_.theta
        );

        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }
    // ==============================
    // Handle goal cancellation
    // ==============================
    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleGoToPose> goal_handle)
    {
        (void) goal_handle;

        RCLCPP_INFO(this->get_logger(), "Goal cancelled");
        // Stop the robot immediately
        geometry_msgs::msg::Twist stop_cmd;
        stop_cmd.linear.x = 0.0;
        stop_cmd.angular.z = 0.0;
        twist_pub_->publish(stop_cmd);

        return rclcpp_action::CancelResponse::ACCEPT;
    }
    // ==============================
    // Accept goal and start execution thread
    // ==============================
    void handle_accepted(const std::shared_ptr<GoalHandleGoToPose> goal_handle)
    {
        std::thread{
            std::bind(&GoToPose::execute, this, std::placeholders::_1),
            goal_handle
        }.detach();
    }
    // ==============================
    // Main control loop
    // ==============================
    void execute(const std::shared_ptr<GoalHandleGoToPose> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Executing goal");

        // control loop at 10 Hz
        rclcpp::Rate rate(10);
        int counter = 0;
        // Tolerances
        const double distance_tolerance = 0.05;
        const double angle_tolerance = 0.08;
        // Speed limits
        const double max_linear_speed = 0.2;
        const double max_angular_speed = 0.6;

        while (rclcpp::ok())
        {
            if (goal_handle->is_canceling())
            {
                geometry_msgs::msg::Twist stop_cmd;
                twist_pub_->publish(stop_cmd);

                auto result = std::make_shared<GoToPoseAction::Result>();
                result->status = false;
                goal_handle->canceled(result);
                return;
            }

            geometry_msgs::msg::Twist cmd;
            // Compute position error
            double dx = desired_pos_.x - current_pos_.x;
            double dy = desired_pos_.y - current_pos_.y;
            double distance = std::sqrt(dx * dx + dy * dy);

            // ==========================
            // Phase 1: Move to position
            // ==========================
            if (distance > distance_tolerance)
            {
                // Desired heading angle
                double theta_target = std::atan2(dy, dx);
                // Angular error
                double direction = normalize_angle(theta_target - current_pos_.theta);

                if (std::fabs(direction) > 0.2)
                {
                    // Rotate first if heading error is large
                    cmd.linear.x = 0.0;
                    cmd.angular.z = std::clamp(
                        direction / 2.0,
                        -max_angular_speed,
                        max_angular_speed
                    );
                }
                else
                {
                    // Move forward and adjust direction
                    cmd.linear.x = max_linear_speed;
                    cmd.angular.z = std::clamp(
                        direction / 2.0,
                        -max_angular_speed,
                        max_angular_speed
                    );
                }
            }
            // ==========================
            // Phase 2: Adjust orientation
            // ==========================
            else
            {
                double error_theta = normalize_angle(desired_pos_.theta - current_pos_.theta);

                if (std::fabs(error_theta) < angle_tolerance)
                {
                   
                    // Final feedback before finishing
                    auto feedback = std::make_shared<GoToPoseAction::Feedback>();
                    feedback->current_pos.x = current_pos_.x;
                    feedback->current_pos.y = current_pos_.y;
                    feedback->current_pos.theta = current_pos_.theta;
                    goal_handle->publish_feedback(feedback);
                    // Stop robot
                    geometry_msgs::msg::Twist stop_cmd;
                    twist_pub_->publish(stop_cmd);
                    // Return success
                    auto result = std::make_shared<GoToPoseAction::Result>();
                    result->status = true;
                    goal_handle->succeed(result);

                    RCLCPP_INFO(this->get_logger(), "Goal reached successfully");
                    RCLCPP_INFO(this->get_logger(), "Action Completed");
                    return;
                }
                // Rotate in place to match final orientation
                cmd.linear.x = 0.0;
                cmd.angular.z = std::clamp(
                    error_theta / 2.0,
                    -max_angular_speed,
                    max_angular_speed
                );
            }

            twist_pub_->publish(cmd);
            // Send feedback every 1 second
            if (counter % 10 == 0)
            {
                auto feedback = std::make_shared<GoToPoseAction::Feedback>();
                feedback->current_pos.x = current_pos_.x;
                feedback->current_pos.y = current_pos_.y;
                feedback->current_pos.theta = current_pos_.theta;

                goal_handle->publish_feedback(feedback);
            }

            counter++;
            rate.sleep();
        }
    }
};
// ==============================
// Main function
// ==============================
int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GoToPose>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}