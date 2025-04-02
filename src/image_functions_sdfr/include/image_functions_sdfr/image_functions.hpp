/**
 * @file image_functions.hpp
 * @author Gijs van Oort (g.vanoort@utwente.nl)
 * @author Jelle Hierck(j.j.hierck@student.utwente.nl)
 * @brief Common operations on images.
 * @version 1.1
 * @date 2023-06-27
 *
 * @copyright Copyright (c) 2024
 *
 */

#ifndef IMAGE_FUNCTIONS__IMAGE_FUNCTIONS_HPP_
#define IMAGE_FUNCTIONS__IMAGE_FUNCTIONS_HPP_

#include <stdexcept>
#include <tuple>

#include "rclcpp/rclcpp.hpp"
#include "image_functions_sdfr/visibility_control.h"
#include "sensor_msgs/image_encodings.hpp"
#include "sensor_msgs/msg/image.hpp"

namespace image_functions {

/**
 * @brief Get the Pixel Brightness object
 * Return the (rounded-to-int) brightness of a certain pixel in the image.
 * For normal images, the value lies between 0 and 255 (including).
 * If the image type is not supported or if the coordinate (x,y) is out of range,
 * a runtime error is thrown.
 *
 * @param im The image we're looking at
 * @param x X-coordinate. Valid range: 0...(im->width -1)
 * @param y Y-coordinate. Valid range: 0...(im->height-1)
 * @return int  (Range: 0..255)
 */
int getPixelBrightness(sensor_msgs::msg::Image::ConstSharedPtr im, int x, int y);

/**
 * @brief Get the Pixel Brightness per color channel of a certain pixel in the image
 * For normal images, the value lies between 0 and 255 (including).
 * If the image type is not supported or if the coordinate (x,y) is out of range,
 * a runtime error is thrown.
 *
 * @param im The image we're looking at
 * @param x X-coordinate. Valid range: 0...(im->width -1)
 * @param y Y-coordinate. Valid range: 0...(im->height-1)
 * @return A tuple of three ints (Range: 0..255) corresponding to three color channels. Which color each channel belongs
 * to depends on the encoding of the image.
 */
std::tuple<int, int, int> getPixelChannels(sensor_msgs::msg::Image::ConstSharedPtr im, int x, int y);

/**
 * @brief Set the color of a pixel of the given image.
 * This assumes that the image is configured correctly (i.e., im->width, im->height,
 * im->step etc are correct and im->data is already allocated).
 *
 * Usually, the channels ch1, ch2, ch3, default to Red, Green and Blue, but
 * the order may differ. Check im->encoding to see the order.
 * If the image type is not supported or the coordinate (x,y) is out of range,
 * a runtime error is thrown.
 *
 * @param im The image we're looking at
 * @param x X-coordinate. Valid range: 0...(im->width -1)
 * @param y Y-coordinate. Valid range: 0...(im->height-1)
 * @param ch1_value Value for channel 1. Valid range: 0..255
 * @param ch2_value Value for channel 2. Valid range: 0..255
 * @param ch3_value Value for channel 3. Valid range: 0..255
 * @return nothing
 */
void setPixelColor(sensor_msgs::msg::Image::SharedPtr im, int x, int y, int ch1_value, int ch2_value, int ch3_value);

/**
 * @brief Copy the properties of the `src` image (width, height, encoding etc) to the dst image.
 * This is actually done by making an exact copy of the image (so the dst->data holds the same image as src->data)
 *
 * @param dst The (pointer to) image that needs its properties set
 * @param src The (pointer to) image from which the properties are looked up
 */
void copyImageProperties(sensor_msgs::msg::Image::SharedPtr dst, sensor_msgs::msg::Image::ConstSharedPtr src);

/**
 * @brief Get the width of the image in pixels
 * @param im The image of which you want to know the width
 * @return int
 */
int getImageWidth(sensor_msgs::msg::Image::ConstSharedPtr im);

/**
 * @brief Get the height of the image in pixels
 * @param im The image of which you want to know the height
 * @return int
 */
int getImageHeight(sensor_msgs::msg::Image::ConstSharedPtr im);

}  // namespace image_functions

#endif  // IMAGE_FUNCTIONS__IMAGE_FUNCTIONS_HPP_
