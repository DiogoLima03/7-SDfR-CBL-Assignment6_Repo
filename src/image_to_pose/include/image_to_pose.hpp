#pragma once

#include <memory>
#include <string>
#include <opencv2/opencv.hpp>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.hpp"

class ImageToPose : public rclcpp::Node
{
public:
  ImageToPose();

private:
  void image_callback(sensor_msgs::msg::Image::UniquePtr msg);
  cv::Point3f estimate_position(const cv::Point2f& image_point, float radius_pixels);

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
  size_t count_;

  // Camera intrinsic parameters (replace with real calibration if needed)
  float fx_ = 525.0;
  float fy_ = 525.0;
  float cx_ = 320.0;
  float cy_ = 240.0;
  float real_ball_radius_m_ = 0.05; // 5cm radius

  const std::string window_name_ = "Green Ball Detection";
};
