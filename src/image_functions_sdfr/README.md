# Image functions
This ROS 2 package serves as a helper library for common image operations on ROS 2 Image messages.

## Usage
1. Download and extract to your favorite folder
2. Add to your ROS 2 workspace.
3. Run `colcon build --packages-select image_functions_sdfr` from your workspace folder.
   This package should *always* compile as standalone.
4. Add this package as dependency for your own package:
   * In `package.xml` add the following:
     - `<depend>image_functions_sdfr</depend>`.
   * In `CMakeLists.txt` add the following:
     - `find_package(image_functions_sdfr REQUIRED)`
     - In `ament_target_dependencies` for the respective target, so: `ament_target_dependencies(<target> [other dependencies] image_functions_sdfr)`
      
5. Make use of the functions in your own package. 

## Functions in this library
*Note: The functions are defined within the namespace `image_functions`*

### `int getPixelBrightness(sensor_msgs::msg::Image::ConstSharedPtr im, int x, int y)`
Get the pixel brightness of the specified pixel. Returns the brightness as an integer.
* `im` - Image we are looking at
* `x`  - X-coordinate. Valid range: 0...(im->width -1)
* `y`  - Y-coordinate. Valid range: 0...(im->height-1)

### `std::tuple<int, int, int> getPixelChannels(sensor_msgs::msg::Image::ConstSharedPtr im, int x, int y)`
Get the color channel values for a specified pixel. Returns a tuple with three integers.
* `im` - Image we are looking at
* `x`  - X-coordinate. Valid range: 0...(im->width -1)
* `y`  - Y-coordinate. Valid range: 0...(im->height-1)

### `void setPixelColor(sensor_msgs::msg::Image::SharedPtr im, int x, int y, int ch1_value, int ch2_value, int ch3_value)`
Set the pixel color of the specified pixel.
* `im` - Image we are looking at
* `x`  - X-coordinate. Valid range: 0...(im->width -1)
* `y`  - Y-coordinate. Valid range: 0...(im->height-1)
* `ch1_value` - Value for channel 1. Valid range: 0..255
* `ch2_value` - Value for channel 2. Valid range: 0..255
* `ch3_value` - Value for channel 3. Valid range: 0..255

### `void copyImageProperties(sensor_msgs::msg::Image::SharedPtr dst, sensor_msgs::msg::Image::ConstSharedPtr src)`
Copies the properties of the `src` image (width, height, encoding etc) to the `dst` image.
* `dst` - The (pointer to) image that needs its properties set.
* `src` - The (pointer to) image from which the properties are looked up

### `int getImageWidth(sensor_msgs::msg::Image::ConstSharedPtr im)`
Get the width of the image in pixels. Returns an integer.
* `im` - Image to extract the wdith from

### `int getImageHeight(sensor_msgs::msg::Image::ConstSharedPtr im)`
Get the height of the image in pixels. Returns an integer.
* `im` - Image to extract the height from