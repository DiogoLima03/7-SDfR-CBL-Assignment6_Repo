#include "steering.hpp"
# include <cmath>

SteerRelbot::SteerRelbot() : Node("steer_relbot")
{
    RCLCPP_INFO(this->get_logger(), "Init");

    // initialize topics
    create_topics();
    RCLCPP_INFO(this->get_logger(), "Created Topics");

    // create subscription
    pose_subscription_ = this->create_subscription<geometry_msgs::msg::Pose>(
        "/ball_pose", 10,
        [this](geometry_msgs::msg::Pose::SharedPtr pose) {
            this->pose_callback(std::move(pose));
        });
}

void SteerRelbot::create_topics()
{
    left_wheel_publish_ = this->create_publisher<example_interfaces::msg::Float64>(
        "/input/left_motor/setpoint_vel", 1);

    right_wheel_publish_ = this->create_publisher<example_interfaces::msg::Float64>(
        "/input/right_motor/setpoint_vel", 1);
}


void SteerRelbot::pose_callback(geometry_msgs::msg::Pose::SharedPtr pose)
{
    // calculate velocity
    calculate_velocity(std::move(pose));

    // publish velocity to simulator
    example_interfaces::msg::Float64 left_wheel;
    left_wheel.data = left_velocity;
    example_interfaces::msg::Float64 right_wheel;
    right_wheel.data = right_velocity;
    left_wheel_publish_->publish(left_wheel);
    right_wheel_publish_->publish(right_wheel);
}

void SteerRelbot::calculate_velocity(geometry_msgs::msg::Pose::SharedPtr pose)
{   
    RCLCPP_INFO(this->get_logger(), "Pose (x=%.2f, Angle=%.2f):", pose->position.x, pose->orientation.z*(180/M_PI));

    // Distance
    float xBall = pose->position.x/3;
    xDesiredDistance = 0.0;

    // Orientation
    float thetaBall = pose->orientation.z;
    thetaDesiredOrientation = 0.0;

    float xVelocity = (1/tau) *(xBall - xDesiredDistance); // m/s
    float omega = (1/tau) * (thetaBall - thetaDesiredOrientation); // rad/s

    right_velocity = xVelocity - (omega * widthRelbot / 2); // m/s
    left_velocity =  xVelocity + (omega * widthRelbot / 2); // m/s

    // Convertes the velocity to the right signal for the I/O hardware
    left_velocity = - left_velocity;
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SteerRelbot>());
    rclcpp::shutdown();
    return 0;
}