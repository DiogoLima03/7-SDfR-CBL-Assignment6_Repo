// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

//===============================================================================================
// Filename : steering.cpp
// Authors : Diogo Lima, Ruben Koudijs
// Group : SDfR-CBL 7
// License : Apache 2.0
// Description : Header of the node SteerRelbot() that subscribes to the "/ball_and_pose" topic,
//              calculates the velocity of the relbot based on the position and orientation of the ball,
//              and publishes the velocity to the relbot or the relbot simulator.
//===============================================================================================

#ifndef STEER_RELBOT_HPP_
#define STEER_RELBOT_HPP_

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

    // parameters
    float desired_position_; // proportional controller, desired position of the ball
    float desired_orientation_; // proportional controller, desired orientation of the ball
    float maxVelocity; // for differentiating code behavior between relbot and relbot simulator
    bool is_relbot_; // maximum velocity of the relbot

    // attributes
    double left_velocity; // motor velocity
    double right_velocity; // motor velocity
    float tau = 0.04; // time constant for the foward velocity
    float tau_omega = 0.008; // time constant for the angular velocity
    float widthRelbot = 0.2; // m

    // methods:

    // Callback function for calculating the velocity of the relbot and publishing it
    // It runs when a new pose and status message is received
    void pose_callback(costum_messages::msg::PoseAndStatus::SharedPtr poseAndStatus);

    // calculates the velocity of the relbot
    void calculate_velocity(costum_messages::msg::PoseAndStatus &poseAndStatus);
};

#endif /*STEER_RELBOT_HPP_*/