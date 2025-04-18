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
// Filename : image_to_pose.hpp
// Authors : Diogo Lima, Ruben Koudijs
// Group : SDfR-CBL 7
// License : Apache 2.0
// Description : Header of the node ImageToPose() that subscribes to the "/image" topic,
//              processes the image to detect a green ball, estimates its 3D position,
//              and publishes the pose (x - Length, Theta - Orientation) as a Pose message.
//===============================================================================================

#pragma once

#include <memory>
#include <string>
#include <opencv2/opencv.hpp>
#include "rclcpp/rclcpp.hpp"

// ROS2 message types
#include "sensor_msgs/msg/image.hpp"

// OpenCV message type
#include "cv_bridge/cv_bridge.hpp"
#include "geometry_msgs/msg/pose.hpp"

// Custom message type for pose and status of the detected ball
#include "costum_messages/msg/pose_and_status.hpp"

class ImageToPose : public rclcpp::Node
{
public:
  ImageToPose();

private:
  // methods:

  // Callback function for image processing, runs when a new image is received
  void image_callback(sensor_msgs::msg::Image::UniquePtr msg);

  // tries to process the image
  // If it fails, it logs an error message and returns as false
  // The image is converted to BGR8 format then a mask is created for the green color
  bool imageProcessing(sensor_msgs::msg::Image::UniquePtr &msg, cv::Mat &image, cv::Mat &mask);

  // If the ball was not detected, it logs an error message and waits for the next image message
  // It also publishes a null pose and status false to the "/ball_pose_and_status" topic,
  // to indicate that finding the ball was not successful
  bool detect_ball(const cv::Mat &mask, cv::Point2f &center, float &radius);

  // Estimate the position of the ball in 2D space (Oxz plane horizontal plane of the camera)
  cv::Point2f estimate_position(const cv::Point2f &image_point, const float &radius_pixels);

  // Calculates the pose of the ball based on the real 2D position of the ball
  geometry_msgs::msg::Pose calculate_pose(cv::Point2f &position2D);

  // Displays the detected ball on the image
  void displayDetection(cv::Mat &image, const cv::Point2f &center, const float &radius, const geometry_msgs::msg::Pose &pose);

  // publishes a null pose and status false to the "/ball_pose_and_status" topic,
  // to indicate that finfing the ball was not successful
  void comunicateBallNotFound();

  // Create a subscription to the "/image" topic
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;

  // Create a publisher for the "/ball_pose_and_status" topic
  rclcpp::Publisher<costum_messages::msg::PoseAndStatus>::SharedPtr poseAndStatus_publisher_;

  // atributes:
  size_t count_; // counts the number of images received

  // Camera intrinsic parameters from camera matrix
  // fx is the focal length in pixels
  // cx is the optical center in pixels
  // These values were obtained from OpenCV camera calibration for the running in the pc
  float cx_ = 320.0;
  float fx_ = 525.0;

  // define minimum and maximum angle to exclude the edge of the image
  // this prevents the RELbot from speeding up if the ball goes out of frame
  float theta_min = -0.50;
  float theta_max = -0.05;

  // Real ball radius in meters
  float real_ball_radius_m_ = 0.0615; 

  // Name of window with ball detected
  const std::string window_name_ = "Green Ball Detection";

  // Parameters:
  bool is_relbot_;

};
