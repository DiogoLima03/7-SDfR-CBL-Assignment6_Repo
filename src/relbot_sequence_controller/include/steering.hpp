#ifndef STEER_RELBOT_HPP_
#define STEER_RELBOT_HPP_

// CPP library headers

// ROS Client Library CPP
#include "rclcpp/rclcpp.hpp"

// message type for velocity
#include "example_interfaces/msg/float64.hpp"

// message type for pose
# include "geometry_msgs/msg/pose.hpp"

class SteerRelbot : public rclcpp::Node
{
public:
    SteerRelbot();

private:
    // Subscribers
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr pose_subscription_;

    // Publishers
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
    void pose_callback(geometry_msgs::msg::Pose::SharedPtr pose);
    void calculate_velocity(geometry_msgs::msg::Pose::SharedPtr pose);
};

#endif /*STEER_RELBOT_HPP_*/