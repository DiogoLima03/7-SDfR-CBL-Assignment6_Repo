#ifndef POSE_TO_TURTLESIM_HPP_
#define POSE_TO_TURTLESIM_HPP_

// CPP library headers
#include <cstdio>
#include <chrono>

// ROS Client Library CPP
#include "rclcpp/rclcpp.hpp"

// Geometry messages, used for location notation
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"


using std::placeholders::_1;
using namespace std::chrono_literals;

class RELbot2Turtlesim : public rclcpp::Node
{
public:
    /**
     * @brief Construct a new convertor object
     */
    RELbot2Turtlesim();

private:
    // Topics
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr robot_pose_topic_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr turtle_twist_topic_;

    // positions used in calculations
    float x = 0;
    float y = 0;
    float theta = 0;
    int seconds = 0;
    uint nanosec = 0;
    
    // previous positions
    float prev_x = 0;
    float prev_y = 0;
    float prev_theta = 0;
    int prev_seconds = 0;
    uint prev_nanosec = 0;

    // speed to send to turtlesim
    float x_speed_fixed;
    float theta_speed_fixed;

    /**
     * @brief Create all topics for this node
     */
    void create_topics();

    /**
     * @brief Callback upon receiving a Pose from the simulator.
     * Converts the pose to Twist velocity and publishes that to turtlesim
     * 
     * @param pose Position of the simulated RELbot.
     */
    void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr pose);

    /**
     * @brief Saves the pose to the corresponding attributes of this node
     * 
     * @param pose Position of the simulated RELbot.
     */
    void savePose(const geometry_msgs::msg::PoseStamped::SharedPtr pose);

    /**
     * @brief Calculates the velocity from the pose saved to this node
     */
    void poseToVel();

    /**
     * @brief Publishes the velocity as a twist topic to turtlesim
     */
    void publishVel();
};
#endif /* POSE_TO_TURTLESIM_HPP_ */