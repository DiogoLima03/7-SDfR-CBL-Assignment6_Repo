#ifndef CAM2IMAGE_VM2ROS_HPP
#define CAM2IMAGE_VM2ROS_HPP

#include <chrono>
#include <memory>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "cv_bridge/cv_bridge.hpp"

#include <boost/asio.hpp>

#include "opencv2/core/mat.hpp"
#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/videoio.hpp"

#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "std_msgs/msg/bool.hpp"

#include "../include/visibility_control.h"

#include "./remote_capture.hpp"

namespace cam2image_vm2ros
{

    class Cam2Image : public rclcpp::Node
    {
    public:
        explicit Cam2Image(const rclcpp::NodeOptions &options);

    private:
        void initialize();
        void timerCallback();
        void parse_parameters();
        bool help(const std::vector<std::string> &args);

        cv::VideoCapture cap;
        remote::RemoteCapture remote_cap;
        cv::Mat last_frame;

        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;
        rclcpp::TimerBase::SharedPtr timer_;

        // ROS parameters
        bool show_camera_;
        size_t depth_;
        double freq_;
        std::string input_topic_;
        std::string output_topic_;
        std::string history_;
        std::string reliability_;
        std::string durability_;
        size_t depth;
        size_t width_;
        size_t height_;
        bool remote_mode_;
        size_t remote_timeout_;
        std::string frame_id_;
        int device_id_;

        std::string socket_ip_;
        int socket_port_;

        bool is_flipped_;
        size_t publish_number_;
    };

} // namespace cam2image_vm2ros

#endif