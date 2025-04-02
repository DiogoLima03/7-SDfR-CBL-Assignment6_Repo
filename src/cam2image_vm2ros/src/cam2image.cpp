// Copyright 2015 Open Source Robotics Foundation, Inc.
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

#include "cam2image.hpp"

namespace cam2image_vm2ros
{

  Cam2Image::Cam2Image(const rclcpp::NodeOptions &options)
      : Node("cam2image", options),
        is_flipped_(false),
        publish_number_(1u)
  {
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    if (help(options.arguments()))
    {
      exit(0);
    }
    parse_parameters(); // declare ros2 parameters
    initialize();
  }

  void Cam2Image::initialize()
  {
    // Create a object of type rclcpp:qos which contains RMW profile that can be used for publishers and subscribers
    auto qos = rclcpp::QoS(depth_); // queue depth of=based on depth parameter
    reliability_ == "best_effort" ? qos.best_effort() : qos.reliable();
    durability_ == "transient_local" ? qos.transient_local() : qos.durability_volatile();
    history_ == "keep_all" ? qos.keep_all() : qos.keep_last(depth_);

    // create a publisher with the desired profile based on current state of variables
    pub_ = this->create_publisher<sensor_msgs::msg::Image>("image", qos);

    auto callback = [this](std_msgs::msg::Bool::ConstSharedPtr msg) -> void
    {
      this->is_flipped_ = msg->data;
      RCLCPP_INFO(this->get_logger(), "Set flip mode to: %s", this->is_flipped_ ? "on" : "off");
    };

    sub_ = create_subscription<std_msgs::msg::Bool>(
        "flip_image", rclcpp::SensorDataQoS(), callback);

    // if the remote node setting parameter is false, try to open the default webcam using opencv.
    if (!remote_mode_)
    {
      cap.open(device_id_);
      cap.set(cv::CAP_PROP_FRAME_WIDTH, static_cast<double>(width_));
      cap.set(cv::CAP_PROP_FRAME_HEIGHT, static_cast<double>(height_));
      if (!cap.isOpened())
      {
        RCLCPP_ERROR(this->get_logger(), "Could not open video stream");
        throw std::runtime_error("Could not open video stream");
      }
    }
    else
    {
      // Open a UDP stream to retreive images from there
      remote_cap.set_ip(socket_ip_, socket_port_);
      remote_cap.initialize(width_, height_, remote_timeout_);
    }

    // timer callback to republish the frames retreived from the webcam or UDP stream on the image topic
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(1000.0 / freq_)),
        std::bind(&Cam2Image::timerCallback, this));
  }

  void Cam2Image::timerCallback()
  {
    cv::Mat frame; // create a frame object

    if (remote_mode_)
    {
      frame = remote_cap.get_frame(); // populate frame object
      if (remote_cap.webcam_is_down())
      {
        rclcpp::shutdown();
        return;
      }
    }
    else
    {
      cap >> frame;
    }

    if (frame.empty())
    {
      return;
    }

    if (is_flipped_)
    {
      cv::flip(frame, frame, 1);
    }

    // optionally show the image
    if (show_camera_)
    {
      cv::imshow("cam2image", frame);
      cv::waitKey(1);
    }

    std_msgs::msg::Header header;
    header.frame_id = frame_id_;
    header.stamp = this->now();
    auto msg = cv_bridge::CvImage(header, "bgr8", frame).toImageMsg();

    RCLCPP_INFO(get_logger(), "Publishing image #%zd", publish_number_++);
    pub_->publish(*msg);
  }

  void Cam2Image::parse_parameters()
  {
    remote_mode_ = this->declare_parameter("remote_mode", false);
    remote_timeout_ = this->declare_parameter("remote_timeout", 7);
    frame_id_ = this->declare_parameter("frame_id", "camera_frame");
    socket_ip_ = this->declare_parameter("socket_ip", "127.0.0.1");
    socket_port_ = this->declare_parameter("socket_port", 80);
    freq_ = this->declare_parameter("frequency", 30.0);
    show_camera_ = this->declare_parameter("show_camera", false);
    device_id_ = static_cast<int>(this->declare_parameter("device_id", 0));
    width_ = this->declare_parameter("width", 320);
    height_ = this->declare_parameter("height", 240);
    reliability_ = this->declare_parameter("reliability", "reliable");
    durability_ = this->declare_parameter("durability", "volatile");
    history_ = this->declare_parameter("history", "keep_last");
    depth_ = this->declare_parameter("depth", 10);
  }

  // this code generates the output for the --help command
  bool Cam2Image::help(const std::vector<std::string> &args)
  {
    if (std::find(args.begin(), args.end(), "--help") != args.end() ||
        std::find(args.begin(), args.end(), "-h") != args.end())
    {
      std::stringstream ss;
      ss << "Usage: cam2image [-h] [--ros-args [-p param:=value] ...]" << std::endl;
      ss << "Publish images from a camera stream." << std::endl;
      ss << "Example: ros2 run image_tools cam2image --ros-args -p reliability:=best_effort";
      ss << std::endl
         << std::endl;
      ss << "Options:" << std::endl;
      ss << "  -h, --help\t\tDisplay this help message and exit";
      ss << std::endl
         << std::endl;
      ss << "Parameters:" << std::endl;
      ss << "  reliability\t\tReliability QoS setting. Either 'reliable' (default) or 'best_effort'";
      ss << std::endl;
      // ... (rest of the help message)
      std::cout << ss.str();
      return true;
    }
    return false;
  }

} // namespace cam2image_vm2ros

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(cam2image_vm2ros::Cam2Image)
