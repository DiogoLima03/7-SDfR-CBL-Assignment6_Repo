#pragma once

#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

class ImageToPose : public rclcpp::Node
{
public:
  ImageToPose();

private:
  void image_callback(sensor_msgs::msg::Image::UniquePtr msg);

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
  size_t count_;
};

