#pragma once

#include <memory>
#include <string>
#include <opencv2/opencv.hpp>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "costum_messages/msg/pose_and_status.hpp"

class ImageToPose : public rclcpp::Node
{
public:
  ImageToPose();

private:
  void image_callback(sensor_msgs::msg::Image::UniquePtr msg);
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
  rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr pose_publisher_;
  rclcpp::Publisher<costum_messages::msg::PoseAndStatus>::SharedPtr poseAndStatus_publisher_;
  size_t count_;


  cv::Point2f estimate_position(const cv::Point2f &image_point, const float &radius_pixels);
  bool imageProcessing(sensor_msgs::msg::Image::UniquePtr &msg, cv::Mat &image, cv::Mat &mask);
  bool detect_ball(const cv::Mat &mask, cv::Point2f &center, float &radius);
  void displayDetection(cv::Mat &image, const cv::Point2f &center, const float &radius, const geometry_msgs::msg::Pose &pose);
  geometry_msgs::msg::Pose calculate_pose(cv::Point2f &position2D);
  void comunicateBallNotFound();


  // Camera intrinsic parameters from camera matrix
  // fx is the focal length in pixels
  // cx is the optical center in pixels
  // These values are obtained from OpenCV camera calibration
  float cx_ = 320.0;
  float fx_ = 525.0;

  // Real ball radius in meters
  float real_ball_radius_m_ = 0.0615; 

  // Name of windoe with ball detected
  const std::string window_name_ = "Green Ball Detection";

};
