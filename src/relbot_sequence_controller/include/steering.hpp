#ifndef STEER_RELBOT_HPP_
#define STEER_RELBOT_HPP_

// CPP library headers

// ROS Client Library CPP
#include "rclcpp/rclcpp.hpp"

// message type for velocity
#include "example_interfaces/msg/float64.hpp"

// message type for pose
# include "geometry_msgs/msg/pose.hpp"

// message type for ros2xeno
#include "xrf2_msgs/msg/ros2_xeno.hpp"

// message type for xeno2ros
#include "xrf2_msgs/msg/xeno2_ros.hpp"

// costum message for pose and status of the detected ball
#include "costum_messages/msg/pose_and_status.hpp"

class SteerRelbot : public rclcpp::Node
{
public:
    SteerRelbot();

private:
    // Subscribers
    rclcpp::Subscription<costum_messages::msg::PoseAndStatus>::SharedPtr poseAndStatus_subscription_;

    // Publisher for the relbot
    rclcpp::Publisher<xrf2_msgs::msg::Ros2Xeno>::SharedPtr velocities_relbot_;

    // publishers for relbot simulation
    rclcpp::Publisher<example_interfaces::msg::Float64>::SharedPtr left_wheel_publish_;
    rclcpp::Publisher<example_interfaces::msg::Float64>::SharedPtr right_wheel_publish_;

    // attributes
    double left_velocity;
    double right_velocity;
    float tau = 0.1; // time constant
    float c = 1; // steering constant
    float xDesiredDistance;
    float thetaDesiredOrientation;
    float widthRelbot = 0.2; // m
    float maxVelocity = 15; // m/s

    // methods
    void create_topics();
    void pose_callback(costum_messages::msg::PoseAndStatus::SharedPtr poseAndStatus);
    void calculate_velocity(costum_messages::msg::PoseAndStatus &poseAndStatus);
};

#endif /*STEER_RELBOT_HPP_*/