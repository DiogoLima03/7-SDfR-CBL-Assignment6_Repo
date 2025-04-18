cmake_minimum_required(VERSION 3.8)
project(image_to_pose)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(costum_messages REQUIRED)

# include header
include_directories(include)

# Executable
add_executable(image_to_pose src/image_to_pose.cpp)

# Executable dependencies
ament_target_dependencies(image_to_pose
  rclcpp
  sensor_msgs
  cv_bridge
  geometry_msgs
  costum_messages
)

# link OpenCV library to executable
target_link_libraries(image_to_pose ${OpenCV_LIBRARIES})

# Install the executable
install(TARGETS
  image_to_pose
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
