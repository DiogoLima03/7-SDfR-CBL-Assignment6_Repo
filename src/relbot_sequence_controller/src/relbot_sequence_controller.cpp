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
// Filename : relbot_sequence_controller.hpp
// Authors : Diogo Lima, Ruben Koudijs
// Group : SDfR-CBL 7
// License : Apache 2.0
// Description : This code is a ROS2 node that subscribes to the "/ball_and_pose" topic,
//              calculates the velocity of the relbot based on the position and orientation of the ball,
//              and publishes the velocity to the relbot or the relbot simulator.
//===============================================================================================

#include "steering.hpp"
#include <cmath>

SteerRelbot::SteerRelbot() : Node("relbot_sequence_controller")
{
    RCLCPP_INFO(this->get_logger(), "Init");

    // Declare parameters
    this->declare_parameter<float>("desired_position", 0); // proportional controller, desired position of the ball
    this->declare_parameter<float>("desired_orientation", 0); // proportional controller, desired orientation of the ball
    this->declare_parameter<bool>("is_relbot", true); // for differentiating code behavior between relbot and relbot simulator
    this->declare_parameter<float>("max_velocity", 4.0); // maximum velocity of the relbot
    
    // get parameters
    desired_position_ = this->get_parameter_or("desired_position", 0.2);
    desired_orientation_ = this->get_parameter_or("desired_orientation", 0.0);
    is_relbot_ = this->get_parameter_or("is_relbot", true);
    maxVelocity = this->get_parameter_or("max_velocity", 4.0);

    // Creates a subscription to the "/ball_and_pose" topic
    poseAndStatus_subscription_ = this->create_subscription<costum_messages::msg::PoseAndStatus>(
        "/ball_pose_and_status", 10,
        [this](costum_messages::msg::PoseAndStatus::SharedPtr poseAndStatus)
        {
            this->pose_callback(std::move(poseAndStatus));
        });

    // Creates a publisher according if its running for the relbot or the relbot simulator
    if (is_relbot_)
    {
        RCLCPP_INFO(this->get_logger(), "Running in the relbot");

        // Creates a publisher for Ros2Xeno topic
    velocities_relbot_ = this->create_publisher<xrf2_msgs::msg::Ros2Xeno>(
            "/Ros2Xeno", 1);
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "Running in the relbot simulator");

        // Creates a publisher for running the relbot simulation sending the velocity to the right wheel
        left_wheel_publish_ = this->create_publisher<example_interfaces::msg::Float64>(
            "/input/left_motor/setpoint_vel", 1);

        // Creates a publisher for running the relbot simulation sending the velocity to the left wheel
        right_wheel_publish_ = this->create_publisher<example_interfaces::msg::Float64>(
            "/input/right_motor/setpoint_vel", 1);
    }
}

void SteerRelbot::pose_callback(costum_messages::msg::PoseAndStatus::SharedPtr poseAndStatus)
{
    // check if it was suscessfully detected a ball
    // if not, publish null velocity to relbot
    if (poseAndStatus->status == false)
    {
        RCLCPP_WARN(this->get_logger(), "No setpoint generated.");

        // publish null velocity to relbot
        example_interfaces::msg::Float64 null_velocity;

        null_velocity.data = 0;

        left_wheel_publish_->publish(null_velocity);
        right_wheel_publish_->publish(null_velocity);

        // breaks the function and waits for the next position message
        return;
    }

    // calculate velocity
    calculate_velocity(*poseAndStatus);

    if (is_relbot_)
    {
        // publish velocity to relbot
        xrf2_msgs::msg::Ros2Xeno vel_msg;

        vel_msg.left_wheel_vel = left_velocity;
        vel_msg.right_wheel_vel = right_velocity;

        velocities_relbot_->publish(vel_msg);
    }
    else
    {
        // publish velocity to simulator
        example_interfaces::msg::Float64 left_wheel;
        example_interfaces::msg::Float64 right_wheel;

        left_wheel.data = left_velocity;
        right_wheel.data = right_velocity;

        left_wheel_publish_->publish(left_wheel);
        right_wheel_publish_->publish(right_wheel);
    }
}

// Input: poseAndStatus - Pose and Status of the detected ball
// Output: Nothing
// Description: Calculates the velocity of the relbot
void SteerRelbot::calculate_velocity(costum_messages::msg::PoseAndStatus &poseAndStatus)
{
    // Gets the ball distance 
    // devided by 3 for better performance. 
    // Result obtained from testing
    float xBall = poseAndStatus.position / 3; 

    // Gets the ball orientation
    float thetaBall = poseAndStatus.orientation;

    // Calculates the desired velocity and angular velocity with a proportional controller
    float xVelocity = (1 / tau) * (xBall - desired_position_);
    float omega = (1 / tau_omega) * (thetaBall - desired_orientation_);

    // converts the states velocity and angular velocity to the outputs of the system, the left and right wheel velocity
    right_velocity = xVelocity - (omega * widthRelbot / 2);
    left_velocity = xVelocity + (omega * widthRelbot / 2);

    // set max value for the wheels velocities
    if (right_velocity > maxVelocity)
    {
        right_velocity = maxVelocity;
    }
    if (right_velocity < -maxVelocity)
    {
        right_velocity = -maxVelocity;
    }
    if (left_velocity > maxVelocity)
    {
        left_velocity = maxVelocity;
    }
    if (left_velocity < -maxVelocity)
    {
        left_velocity = -maxVelocity;
    }

    // Convertes the velocity to the right signal for the I/O hardware or the relbot simulator
    if (is_relbot_)
    {
        // for running in the relbot
        right_velocity = right_velocity;
        left_velocity = left_velocity;
    }
    else
    {
        // for running with the relbot simulator in the pc
        right_velocity = right_velocity;
        left_velocity = -left_velocity;
    }
}

// Input: argc - number of arguments, argv - arguments
// Output: 0 - success
// Description: Initializes the ROS2 node, creates an instance of SteerRelbot,
//              and starts the ROS2 event loop to process incoming messages.
int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SteerRelbot>());
    rclcpp::shutdown();
    return 0;
}