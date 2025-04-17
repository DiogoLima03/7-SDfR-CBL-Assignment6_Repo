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
// Filename : image_to_pose.cpp.cpp
// Authors : Diogo Lima, Ruben Koudijs
// Group : SDfR-CBL 7
// License : Apache 2.0
// Description : This code is a ROS2 node that subscribes to an image topic, processes the image
// to detect a green ball, and estimates its 3D position. The estimated position is then
// published as a Pose message (x - Length, Theta - Orientation).
//===============================================================================================

#include "image_to_pose.hpp"
#include <iomanip>
#include <sstream>
#include <cmath>

// Input: Nothing
// Output: Nothing
// Decription: Constructor of ImageToPose. It initializes a node ("image_to_pose"), creates a subscription
// to the "/image" topic, and creates a publisher for the "ball_pose" topic. It also initializes the count variable.
ImageToPose::ImageToPose() : Node("image_to_pose"), count_(0)
{
  // Creates a subscription to the "/image" topic
  // Allows for 10 messages in the queue
  // A callback function is called when a new message is received to process the image
  subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/image", 10,
      std::bind(&ImageToPose::image_callback, this, std::placeholders::_1));

  // Creates a publisher for the "/ball_pose_and_status" topic
  // Allows for 10 messages in the queue
  // The publisher sends messages of type costum_messages::msg::PoseAndStatus (float32 position, float32 orientation, bool status)
  poseAndStatus_publisher_ = this->create_publisher<costum_messages::msg::PoseAndStatus>("/ball_pose_and_status", 10);
}

// Input: msg - Image message
// Output: Nothing
// Description: This function is called when a new image message is received.
// It processes the image to detect a green ball, estimates its 3D position, and publishes the pose (x - Length, Theta - Orientation).
// It uses OpenCV to convert the image to HSV color space, creates a mask for the green color,
// detects contours, and finds the largest contour. It then estimates the position of the ball in 3D space
// calculates and publishes the pose as a geometry_msgs::msg::Pose message.
// It also draws the detected ball on the image and displays it using OpenCV.
// If no ball is detected, it publishes a null pose and displays a warning message.
// Note: The function assumes that the camera is calibrated and uses the camera intrinsic parameters (camera matrix)
// (fx, fy, cx, cy) to convert the 2D image point to a 3D point in the camera coordinate system.
void ImageToPose::image_callback(sensor_msgs::msg::Image::UniquePtr msg)
{
  // Logs having received a new image
  RCLCPP_INFO(this->get_logger(), "Received image #%zu", count_++);

  // Processes the image
  cv::Mat image;
  cv::Mat mask;
  bool checkImageProcessing = imageProcessing(msg, image, mask);
  if (!checkImageProcessing){
    // detects an error in the image processing
    RCLCPP_WARN(this->get_logger(), "Image Processing Error:");

    comunicateBallNotFound();
    return; 
  }

  cv::Point2f center;
  float radius;

  bool checkBallDetection = detect_ball(mask, center, radius);

  if (!checkBallDetection)
  {
    RCLCPP_WARN(this->get_logger(), "No valid green ball found.");
    
    comunicateBallNotFound();

    cv::imshow(window_name_, image);
    cv::waitKey(1);
    return;
  }

  // Estimate the position of the ball in 2D space (Oxz plane horizontal plane of the camera)
  cv::Point2f position2D = estimate_position(center, radius);

  // The 2D position of the ball is loged for debugging
  RCLCPP_INFO(this->get_logger(), "Ball position (x=%.2f, z=%.2f) m", position2D.x, position2D.y);

  // calculates and publishes the pose
  geometry_msgs::msg::Pose pose = calculate_pose(position2D);

  // Display the detected ball on the image
  displayDetection(image, center, radius, pose);
}

void ImageToPose::comunicateBallNotFound()
{
  costum_messages::msg::PoseAndStatus poseAndStatus;
  poseAndStatus.position = 0.0;
  poseAndStatus.orientation = 0.0;
  poseAndStatus.status = false;
  poseAndStatus_publisher_->publish(poseAndStatus);
}


bool ImageToPose::imageProcessing(sensor_msgs::msg::Image::UniquePtr &msg, cv::Mat &image, cv::Mat &mask)
{
  // Convert from ROS2 image message to OpenCV image
  // Initializes a pointer for the image
  

  // Uses a try cacth for attempting to convert the image
  // If it fails, it logs an error message and returns
  // The image is converted to BGR8 format
  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(*msg, "bgr8");
  }
  catch (const std::exception &e)
  {
    RCLCPP_WARN(this->get_logger(), ("image Processing Error: " + std::string(e.what())).c_str());
    
    // Return with a false if the image conversion fails
    return false; 
  }
  // Save the converted image 
  image = cv_ptr->image;

  // Convert the image to HSV color space
  cv::Mat new_hsv, new_mask;
  cv::cvtColor(cv_ptr->image, new_hsv, cv::COLOR_BGR2HSV);

  // Create a mask for the green color and saves it in the image input argument
  cv::inRange(new_hsv, cv::Scalar(35, 100, 100), cv::Scalar(85, 255, 255), new_mask);

  // Save the new mask 
  mask = new_mask;

  // Confirm that the Image Processing was successful
  return true;
}


bool ImageToPose::detect_ball(const cv::Mat &mask, cv::Point2f &center, float &radius)
{
  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

  if (contours.empty()) return false;

  auto max_contour = *std::max_element(contours.begin(), contours.end(),
                                       [](const auto &a, const auto &b) {
                                         return cv::contourArea(a) < cv::contourArea(b);
                                       });

  cv::minEnclosingCircle(max_contour, center, radius);
  return radius >= 5.0;
}

// Input: image_point - 2D point in the image (in pixels)
//        radius_pixels - radius of the ball (in pixels)
// Output: 2D point (x,z) in meters
// Description: Estimates the 2D position (Oxz plane horizontal plane of the camera) of the ball in meters.
// This function assumes a pinhole camera model. The function uses the camera intrinsic parameters
// (fx, cx) to convert the 2D image point to a 3D point in the camera coordinate system.
cv::Point2f ImageToPose::estimate_position(const cv::Point2f &image_point, const float &radius_pixels)
{
  // z is the depth position of the ball to the camera
  float z = (real_ball_radius_m_ * fx_) / radius_pixels;

  // x is the transverse position of the ball to the camera
  float x = (image_point.x - cx_) * z / fx_;

  // Note: y (vertical position) is not computed here to save computation,
  // as it is currently unused in downstream pose calculations.

  // Return the estimated position as a 3D point in meters
  return cv::Point2f(x, z);
}

// Input: position - 2D position of the ball in meters
// Output: pose - x - Length [m], Theta - Orientation [rad]
// Description: Calculates the pose of the ball base on the real 2D position of the ball
// and publishes it to the "/ball_pose" topic.
geometry_msgs::msg::Pose ImageToPose::calculate_pose(cv::Point2f &position2D)
{
  // Create a new Pose variable
  geometry_msgs::msg::Pose pose;

  costum_messages::msg::PoseAndStatus poseAndStatus;

  // Calculate the x "pose" position of the ball
  // The x "pose" position is the distance from the camera to the ball
  // we don't use the y posittion of the ball, we assume that the ball is projected into the xOz plane
  // The x "pose" is in meters
  pose.position.x = sqrt(pow(position2D.x, 2) + pow(position2D.y, 2));
  poseAndStatus.position = pose.position.x;
  // Calculate Theta "pose" orientation of the ball in a 2D plane
  // and store it in the z orientation of the Pose message
  // The theta "pose" orientation is the angle between the front acis of the camera
  // and the line connecting the camera to the ball
  // The angle is in radians
  pose.orientation.z = (M_PI / 2) - atan2(position2D.y, position2D.x);
  poseAndStatus.orientation = pose.orientation.z;
  poseAndStatus.status = true;

  // Publishes the Pose and Status as True 
  poseAndStatus_publisher_->publish(poseAndStatus);

  return pose;
}

// Input:, image - OpenCV image, center of the ball, radius of the ball, pose of the ball
// Output: Nothing
// Description: Displays the detected ball on the image.
// This function draws a circle around the detected ball using OpenCV.
// Also display the pose of the ball in the image.
void ImageToPose::displayDetection(cv::Mat &image,
                                   const cv::Point2f &center,
                                   const float &radius,
                                   const geometry_msgs::msg::Pose &pose)
{
  // Draw a circle detection on image
  cv::circle(image, center, static_cast<int>(radius), cv::Scalar(0, 0, 255), 2);

  // Label for displaying the pose values
  std::ostringstream label;
  label << "(" << std::fixed << std::setprecision(2)
                             << pose.position.x << ", " << pose.orientation.z << ") ";

  // Draw the label on the image
  cv::putText(image, label.str(), center + cv::Point2f(-90, -(radius + 10)),
              cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255), 2);

  // Draw a red dot at the center of the ball
  cv::circle(image, center, 3, cv::Scalar(0, 0, 255), -1);

  // Display the image with the detected ball
  cv::imshow(window_name_, image);

  // Display the image for 1 ms
  cv::waitKey(1);
}

// Input: argc - number of command line arguments
//        argv - array of command line arguments
// Output: 0 on success, non-zero on failure
// Description: Initializes the ROS2 node, creates an instance of ImageToPose,
//              and starts the ROS2 event loop to process incoming messages.
int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImageToPose>());
  rclcpp::shutdown();
  return 0;
}
