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

/* This example creates a subclass of Node and uses a fancy C++11 lambda
 * function to shorten the callback syntax, at the expense of making the
 * code somewhat more difficult to understand at first glance. */

#include "image_to_pose.hpp"
#include <iomanip>
#include <sstream>
#include <cmath>



ImageToPose::ImageToPose() : Node("image_to_pose"), count_(0)
{
  subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/image", 10,
      std::bind(&ImageToPose::image_callback, this, std::placeholders::_1));

      pose_publisher_ = this->create_publisher<geometry_msgs::msg::Pose>("/ball_pose", 10);
}

void ImageToPose::image_callback(sensor_msgs::msg::Image::UniquePtr msg)
{
  RCLCPP_INFO(this->get_logger(), "Received image #%zu", count_++);

  // Convert to OpenCV
  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(*msg, "bgr8");
  }
  catch (cv_bridge::Exception &e)
  {
    RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    return;
  }

  // Converts the image to HSV color space
  // and creates a mask for the green color
  cv::Mat hsv, mask;
  cv::cvtColor(cv_ptr->image, hsv, cv::COLOR_BGR2HSV);
  cv::inRange(hsv, cv::Scalar(35, 100, 100), cv::Scalar(85, 255, 255), mask);

  // Countours detection
  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

  // Checks for the green ball
  if (contours.empty())
  {
    RCLCPP_WARN(this->get_logger(), "No green ball found.");
    cv::imshow(window_name_, cv_ptr->image);
    cv::waitKey(1);
    return;
  }

  // Find the largest contour
  auto max_contour = std::max_element(contours.begin(), contours.end(),
                                      [](const auto &a, const auto &b)
                                      {
                                        return cv::contourArea(a) < cv::contourArea(b);
                                      });

  // tries to find the ball (center, radius) in the mask
  cv::Point2f center;
  float radius;
  cv::minEnclosingCircle(*max_contour, center, radius);

  // Check if the detected radius is valid
  if (radius < 5.0)
  {
    RCLCPP_WARN(this->get_logger(), "Detected ball is too small.");
    cv::imshow(window_name_, cv_ptr->image);
    cv::waitKey(1);
    return;
  }

  // Estimate the position of the ball in 3D space
  cv::Point3f position = estimate_position(center, radius);
  RCLCPP_INFO(this->get_logger(), "Ball position (x=%.2f, y=%.2f, z=%.2f) m", position.x, position.y, position.z);

  // Draw detection on image
  cv::circle(cv_ptr->image, center, static_cast<int>(radius), cv::Scalar(0, 0, 255), 2);
  std::ostringstream label;
  label << "[" << std::fixed << std::setprecision(2)
        << position.x << ", " << position.y << ", " << position.z << "]";

  // Draw a red dot at the center of the ball
  cv::circle(cv_ptr->image, center, 3, cv::Scalar(0, 0, 255), -1);

  cv::putText(cv_ptr->image, label.str(), center + cv::Point2f(-90, -(radius + 10)),
              cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255), 2);

  cv::imshow(window_name_, cv_ptr->image);
  cv::waitKey(1);

  // calculates and publishes the position in 2D
  calculate_pose(position);
}

// Description: Estimates the 3D position of the ball in meters
// Input: image_point - 2D point in the image (in pixels)
//        radius_pixels - radius of the ball in pixels
// Output: 3D point in meters
// Note: This function assumes a pinhole camera model and that the ball is spherical
//       and the camera is calibrated. The function uses the camera intrinsic parameters
//       (fx, fy, cx, cy) to convert the 2D image point to a 3D point in the camera coordinate system.
cv::Point3f ImageToPose::estimate_position(const cv::Point2f &image_point, float radius_pixels)
{
  float z = (real_ball_radius_m_ * fx_) / radius_pixels;
  float x = (image_point.x - cx_) * z / fx_;
  float y = (image_point.y - cy_) * z / fy_;
  return cv::Point3f(x, y, z);
}

void ImageToPose::calculate_pose(cv::Point3f position){
  // Create a Pose message
  geometry_msgs::msg::Pose pose_msg;

  // Set the x position of the ball
  pose_msg.position.x = sqrt(pow(position.x,2) + pow(position.z, 2));

  // Set z orientation of the ball in a 2D plane
  pose_msg.orientation.z = (M_PI/2) - atan2(position.z, position.x);

  // Log and publish the pose
  //RCLCPP_INFO(this->get_logger(), "Publishing Pose: x=%.2f, y=%.2f, z=%.2f, theta=%.2f", 
   //           pose_msg.position.x, pose_msg.orientation.z);
  
  // Publish the message
  pose_publisher_->publish(pose_msg);
}

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImageToPose>());
  rclcpp::shutdown();
  return 0;
}

