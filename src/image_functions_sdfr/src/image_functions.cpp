/**
 * @file image_functions.cpp
 * @author Gijs van Oort (g.vanoort@utwente.nl)
 * @author Jelle Hierck(j.j.hierck@student.utwente.nl)
 * @brief Common operations on images.
 * @version 1.1
 * @date 2023-06-27
 *
 * @copyright Copyright (c) 2024
 *
 */
#include "image_functions_sdfr/image_functions.hpp"

namespace image_functions {

void assertCoordinateWithinRange(sensor_msgs::msg::Image::ConstSharedPtr im, int x, int y) {
    // Cast unsigned stuff to signed to avoid compiler warnings
    if (x < 0 || x >= (int)im->width) {
        throw std::out_of_range("x-coordinate out of range");
    }
    if (y < 0 || y >= (int)im->height) {
        throw std::out_of_range("y-coordinate out of range");
    }
}

void assertSupportedImageType(sensor_msgs::msg::Image::ConstSharedPtr im) {
    if (sensor_msgs::image_encodings::numChannels(im->encoding) != 3) {
        throw std::runtime_error("Unsupported image type (number of color channels is not equal to 3)");
    }
    if (sensor_msgs::image_encodings::bitDepth(im->encoding) != 8) {
        throw std::runtime_error(
            "Unsupported image type (bit depth (number of bits per channel per pixel is not equal to 8)");
    }
}

int getPixelBrightness(sensor_msgs::msg::Image::ConstSharedPtr im, int x, int y) {
    assertSupportedImageType(im);
    assertCoordinateWithinRange(im, x, y);

    // Each line of the image takes im->step bytes. In order to access the y'th row, we should skip y*im->step bytes.
    uint8_t ch1_value = im->data[im->step * y + 3 * x + 0];
    uint8_t ch2_value = im->data[im->step * y + 3 * x + 1];
    uint8_t ch3_value = im->data[im->step * y + 3 * x + 2];

    return (int)round((ch1_value + ch2_value + ch3_value) / 3.0);
}

std::tuple<int, int, int> getPixelChannels(sensor_msgs::msg::Image::ConstSharedPtr im, int x, int y) {
    assertSupportedImageType(im);
    assertCoordinateWithinRange(im, x, y);

    // Each line of the image takes im->step bytes. In order to access the y'th row, we should skip y*im->step bytes.
    uint8_t ch1_value = im->data[im->step * y + 3 * x + 0];
    uint8_t ch2_value = im->data[im->step * y + 3 * x + 1];
    uint8_t ch3_value = im->data[im->step * y + 3 * x + 2];

    return std::tuple<int, int, int>(ch1_value, ch2_value, ch3_value);
}

void setPixelColor(sensor_msgs::msg::Image::SharedPtr im, int x, int y, int ch1_value, int ch2_value, int ch3_value) {
    assertSupportedImageType(im);
    try {
        assertCoordinateWithinRange(im, x, y);
        // Each line of the image takes im->step bytes. In order to access the y'th row, we should skip y*im->step
        // bytes.
        im->data[im->step * y + 3 * x + 0] = ch1_value;
        im->data[im->step * y + 3 * x + 1] = ch2_value;
        im->data[im->step * y + 3 * x + 2] = ch3_value;
    } catch (std::runtime_error &e) {
        std::cout << "Warning: tried to draw a pixel at (" << x << "," << y << ") but that's outside the image canvas"
                  << std::endl;
    }
}

void copyImageProperties(sensor_msgs::msg::Image::SharedPtr dst, sensor_msgs::msg::Image::ConstSharedPtr src) {
    dst->header = src->header;
    dst->height = src->height;
    dst->width = src->width;
    dst->is_bigendian = src->is_bigendian;
    dst->encoding = src->encoding;
    dst->step = src->step;
    dst->data = src->data;
}

int getImageWidth(sensor_msgs::msg::Image::ConstSharedPtr im) { return (int)im->width; }

int getImageHeight(sensor_msgs::msg::Image::ConstSharedPtr im) { return (int)im->height; }

}  // namespace image_functions
