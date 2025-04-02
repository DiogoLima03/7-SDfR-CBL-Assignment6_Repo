#ifndef ROS_XENO_BRIDGE_HPP
#define ROS_XENO_BRIDGE_HPP

#include <functional>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include <poll.h>
#include <error.h>
#include <errno.h>
#include <iostream>
#include <fstream>
#include <fcntl.h>

#include "debug_settings.h"

// Define message includes here...
#include "xrf2_msgs/msg/ros2_xeno.hpp"
#include "xrf2_msgs/msg/xeno2_ros.hpp"
#include <std_msgs/msg/int32.hpp>

// Placeholder for std::bind.
using std::placeholders::_1;

/**
 * @class RosXenoBridge
 * @brief A ROS2 node for bridging data between ROS2 and Xenomai environments.
 * 
 * This node acts as a bridge to facilitate data exchange between ROS2 and
 * Xenomai real-time systems. It subscribes to and publishes messages to
 * specific topics, while managing associated control and data file descriptors.
 */
class RosXenoBridge : public rclcpp::Node {
public:
    RosXenoBridge();
    int data_fd, control_fd;

private:
    //
    void initialize();

    /// Callback functions.
    /**
     * Veranderd dit doxigen for je zelf
     * @brief Callback to process any incoming Image messages.
     * 
     * @param img The image that was received.
    */
    void controlCallback(const std_msgs::msg::Int32::SharedPtr msg);
    void dataCallback(const xrf2_msgs::msg::Ros2Xeno::SharedPtr msg);
    void timerCallback();
    
    /// Private variables.
    // ...
    
    /// Subscriber variables.
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr control_subscription_; ///< Subscription to control messages.
    rclcpp::Subscription<xrf2_msgs::msg::Ros2Xeno>::SharedPtr data_subscription_; ///< Subscription to data messages.

    /// Publisher variables.
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr control_publisher_; ///< Publisher for control messages.
    rclcpp::Publisher<xrf2_msgs::msg::Xeno2Ros>::SharedPtr data_publisher_; ///< Publisher for data messages.

    /// Timer variables.
    rclcpp::TimerBase::SharedPtr timer_; ///< Timer for periodic tasks.
};

#endif /* ROS_XENO_BRIDGE_HPP */

