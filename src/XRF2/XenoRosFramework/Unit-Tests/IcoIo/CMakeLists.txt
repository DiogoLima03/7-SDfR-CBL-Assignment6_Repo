cmake_minimum_required(VERSION 3.10)
project(IcoIo)

#set(CMAKE_CXX_STANDARD 11)

add_compile_options(-Wno-pointer-arith)

# Find dependencies
find_package(Threads REQUIRED)
find_package(xrf2_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable( IcoIo main.cpp 
                ../../Xenomai/src/IcoIo.cpp
                ../../Xenomai/src/XenoSpiControllerHandler.cpp
                )

ament_target_dependencies(IcoIo PUBLIC rclcpp xrf2_msgs std_msgs)

# Specify paths to header files
target_include_directories( IcoIo BEFORE PUBLIC 
                            /usr/evl/include
                            ../../Xenomai/include
                            ../../Common/include
                            )

# Specify paths to library files
target_link_directories(IcoIo PUBLIC /usr/evl/lib)

target_link_libraries(IcoIo PRIVATE evl)
