cmake_minimum_required(VERSION 3.10)
project(UT_Logger)

#set(CMAKE_CXX_STANDARD 11)

add_compile_options(-Wno-pointer-arith)

# Find dependencies
find_package(Threads REQUIRED)
find_package(xrf2_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable( UT_Logger main.cpp
                ../../Xenomai/src/XenoFrtLogger.cpp
                ../../Xenomai/src/XenoFileHandler.cpp
                )

ament_target_dependencies(UT_Logger PUBLIC rclcpp xrf2_msgs std_msgs)

# Specify paths to header files
target_include_directories( UT_Logger BEFORE PUBLIC 
                            /usr/evl/include 
                            ../../Xenomai/include
                            ../../Common/include
                            )

# Specify paths to library files
target_link_directories(UT_Logger PUBLIC /usr/evl/lib)

target_link_libraries(UT_Logger PRIVATE evl)
