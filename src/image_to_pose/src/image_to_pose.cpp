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

 ImageToPose::ImageToPose() : Node("image_to_pose"), count_(0)
 {
   subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
     "/image", 10,
     std::bind(&ImageToPose::image_callback, this, std::placeholders::_1));
 }
 
 void ImageToPose::image_callback(sensor_msgs::msg::Image::UniquePtr msg)
 {
   RCLCPP_INFO(this->get_logger(), "Received image #%zu: width=%u, height=%u, encoding=%s",
               count_++, msg->width, msg->height, msg->encoding.c_str());
 }
 
 int main(int argc, char *argv[])
 {
   rclcpp::init(argc, argv);
   rclcpp::spin(std::make_shared<ImageToPose>());
   rclcpp::shutdown();
   return 0;
 }







// #include <memory>

// #include "rclcpp/rclcpp.hpp"
// #include "sensor_msgs/msg/image.hpp"

// using std::placeholders::_1;

// class Image_to_pose : public rclcpp::Node
// {
//   public:
//     Image_to_pose()
//     : Node("image_to_pose")
//     {
//       RCLCPP_INFO(this->get_logger(), "Subscription created for topic 'image'");
//       subscription_ = this->create_subscription<sensor_msgs::msg::Image>("/image", 10, std::bind(&Image_to_pose::image_callback, this, _1));
//     }

//   private:
//     rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;

//     void image_callback(const sensor_msgs::msg::Image::SharedPtr img)
//     {
//       RCLCPP_INFO(this->get_logger(), "Processing image beep boop");
//     }
// };

// int main(int argc, char * argv[])
// {
//   rclcpp::init(argc, argv);
//   rclcpp::spin(std::make_shared<Image_to_pose>());
//   rclcpp::shutdown();
//   return 0;
// }
