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

    poseAndStatus_subscription_ = this->create_subscription<costum_messages::msg::PoseAndStatus>(
        "/ball_pose_and_status", 10,
        [this](costum_messages::msg::PoseAndStatus::SharedPtr poseAndStatus) {
            this->pose_callback2(std::move(poseAndStatus));
        });

}

void SteerRelbot::create_topics()
{
    // Creates a publisher for running the relbot simulation sending the velocity to the right wheel 
    left_wheel_publish_ = this->create_publisher<example_interfaces::msg::Float64>(
        "/input/left_motor/setpoint_vel", 1);
    
    // Creates a publisher for running the relbot simulation sending the velocity to the left wheel 
    right_wheel_publish_ = this->create_publisher<example_interfaces::msg::Float64>(
        "/input/right_motor/setpoint_vel", 1);
    
    // Creates a publisher for Ros2Xeno topic
    velocities_relbot_ = this->create_publisher<xrf2_msgs::msg::Ros2Xeno>(
            "/Ros2Xeno", 1);
}


void SteerRelbot::pose_callback(geometry_msgs::msg::Pose::SharedPtr pose)
{   
    if (pose->position.x == 0.0 && pose->orientation.z == 0.0)
    {
        RCLCPP_WARN(this->get_logger(), "No setpoint generated.");
        
        example_interfaces::msg::Float64 null_velocity;
        null_velocity.data = 0;
        left_wheel_publish_->publish(null_velocity);
        right_wheel_publish_->publish(null_velocity);
        return;
    }

    // calculate velocity
    calculate_velocity(std::move(pose));

    // publish velocity to relbot
    xrf2_msgs::msg::Ros2Xeno vel_msg;
    vel_msg.left_wheel_vel = left_velocity;
    vel_msg.right_wheel_vel = right_velocity;
    velocities_relbot_->publish(vel_msg);

    // publish velocity to simulator
    example_interfaces::msg::Float64 left_wheel;
    left_wheel.data = left_velocity;
    example_interfaces::msg::Float64 right_wheel;
    right_wheel.data = right_velocity;
    left_wheel_publish_->publish(left_wheel);
    right_wheel_publish_->publish(right_wheel); 
}

void SteerRelbot::pose_callback2(costum_messages::msg::PoseAndStatus::SharedPtr poseAndStatus)
{   
    if (poseAndStatus->status == false)
    {
        RCLCPP_WARN(this->get_logger(), "No setpoint generated.");
        
        example_interfaces::msg::Float64 null_velocity;
        null_velocity.data = 0;
        left_wheel_publish_->publish(null_velocity);
        right_wheel_publish_->publish(null_velocity);
        return;
    }

    // calculate velocity
    calculate_velocity2(*poseAndStatus);

    // publish velocity to relbot
    xrf2_msgs::msg::Ros2Xeno vel_msg;
    vel_msg.left_wheel_vel = left_velocity;
    vel_msg.right_wheel_vel = right_velocity;
    velocities_relbot_->publish(vel_msg);

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
    xDesiredDistance = 0.300;

    // Orientation
    float thetaBall = pose->orientation.z;
    thetaDesiredOrientation = 0.0;

    float xVelocity = (1/tau) *(xBall - xDesiredDistance); // m/s
    float omega = (1/tau) * (thetaBall - thetaDesiredOrientation); // rad/s

    right_velocity = xVelocity - (omega * widthRelbot / 2); // m/s
    left_velocity =  xVelocity + (omega * widthRelbot / 2); // m/s

    // Convertes the velocity to the right signal for the I/O hardware
    // maybe add a parameter for simulator and relbot launch file ...
    //left_velocity = - left_velocity;
}

void SteerRelbot::calculate_velocity2(costum_messages::msg::PoseAndStatus &poseAndStatus)
{   
    RCLCPP_INFO(this->get_logger(), "Pose (x=%.2f, Angle=%.2f):", poseAndStatus.position, poseAndStatus.orientation);

    // Distance
    float xBall = poseAndStatus.position/3;
    xDesiredDistance = 0.300;

    // Orientation
    float thetaBall = poseAndStatus.orientation;
    thetaDesiredOrientation = 0.0;

    float xVelocity = (1/tau) *(xBall - xDesiredDistance); // m/s
    float omega = (1/tau) * (thetaBall - thetaDesiredOrientation); // rad/s

    right_velocity = xVelocity - (omega * widthRelbot / 2); // m/s
    left_velocity =  xVelocity + (omega * widthRelbot / 2); // m/s

    // Convertes the velocity to the right signal for the I/O hardware
    // maybe add a parameter for simulator and relbot launch file ...
    //left_velocity = - left_velocity;
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SteerRelbot>());
    rclcpp::shutdown();
    return 0;
}