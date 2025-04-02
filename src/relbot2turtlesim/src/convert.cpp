#include "convert.hpp"

RELbot2Turtlesim::RELbot2Turtlesim() : Node("relbot2turtlesim") {
    RCLCPP_INFO(this->get_logger(), "Init");

    create_topics();
    RCLCPP_INFO(this->get_logger(), "Created Topics");
}

void RELbot2Turtlesim::create_topics() {
    RCLCPP_INFO(this->get_logger(), "Creating topics...");

    RCLCPP_INFO(this->get_logger(), "Creating Publishers");
    turtle_twist_topic_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 1);

    RCLCPP_INFO(this->get_logger(), "Creating Subscriptions");
    robot_pose_topic_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/output/robot_pose", 10,
        std::bind(&RELbot2Turtlesim::poseCallback, this, std::placeholders::_1));
    RCLCPP_INFO(this->get_logger(), "Subscribed to topic /output/robot_pose");
}

void RELbot2Turtlesim::poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr pose) {
    // save new pose to node
    savePose(pose);

    // calculate velocity
    poseToVel();

    // publish velocity
    publishVel();
}

void RELbot2Turtlesim::savePose(const geometry_msgs::msg::PoseStamped::SharedPtr pose) {
    // keep old pose
    prev_x = x;
    prev_y = y;
    prev_theta = theta;
    prev_seconds = seconds;
    prev_nanosec = nanosec;

    // get new pose
    x = pose->pose.position.x;
    y = pose->pose.position.y;
    theta = pose->pose.orientation.z;  // in radiants
    seconds = pose->header.stamp.sec;
    nanosec = pose->header.stamp.nanosec;
}

void RELbot2Turtlesim::poseToVel() {
    // calculate time passed
    float time = (seconds - prev_seconds) + ((int)nanosec - (int)prev_nanosec) * 0.000000001;
    if (time == 0) {
        return;
    }

    // calculate speed in ground fixed frame
    float x_speed = (x - prev_x) / time;
    float y_speed = (y - prev_y) / time;
    theta_speed_fixed = (theta - prev_theta) / time;

    // convert to body fixed frame (theta is the same as for ground fixed frame)
    // We pick the largest, as dividing by a small value will result in floating point errors.
    if (abs(std::cos(theta)) < abs(std::sin(theta))) {
        x_speed_fixed = y_speed / std::sin(theta);
    } else {
        x_speed_fixed = x_speed / std::cos(theta);
    }
}

void RELbot2Turtlesim::publishVel() {
    // publish to turtlesim
    geometry_msgs::msg::Twist twist;
    twist.linear.x = x_speed_fixed;
    twist.angular.z = theta_speed_fixed;
    turtle_twist_topic_->publish(twist);
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RELbot2Turtlesim>());
    rclcpp::shutdown();
    return 0;
}
