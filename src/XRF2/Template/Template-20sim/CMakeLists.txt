cmake_minimum_required(VERSION 3.10)
project(template_20sim)

#set(CMAKE_CXX_STANDARD 11)

add_compile_options(-Wno-pointer-arith)

set(locationFramework "../XRF2/XenoRosFramework/")

set(locationXenoSrc "${locationFramework}Xenomai/src/")
set(locationXenoInclude "${locationFramework}Xenomai/include")
set(locationCommonInclude "${locationFramework}Common/include")

# Path to the XML file
set(XML_FILE_PATH "${CMAKE_SOURCE_DIR}/controller/LoopController_tokens.xml")

# Run the Python script to extract TIME_STEP_SIZE, passing the XML file path
execute_process(
    COMMAND python3 ${CMAKE_SOURCE_DIR}/${locationFramework}Common/python_script/extracte_token_time_step.py ${XML_FILE_PATH}
    OUTPUT_VARIABLE TIME_STEP_SIZE_VALUE
    OUTPUT_STRIP_TRAILING_WHITESPACE
)
message("TIME_STEP_SIZE_VALUE = " ${TIME_STEP_SIZE_VALUE})

# Find dependencies
find_package(Threads REQUIRED)
find_package(xrf2_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
file(GLOB CONTROLLER_SRC "controller/common/*.cpp")
file(GLOB SRC "src/*.cpp")
set(XENO  ${locationXenoSrc}FullStateMachine.cpp
          ${locationXenoSrc}IcoIo.cpp
          ${locationXenoSrc}XenoFileHandler.cpp
          ${locationXenoSrc}XenoFrtLogger.cpp
          ${locationXenoSrc}XenoFrtLoop.cpp
          ${locationXenoSrc}XenoFrtMonitor.cpp
          ${locationXenoSrc}XenoFrtRosIco.cpp
          ${locationXenoSrc}XenoRosCommunication.cpp
          ${locationXenoSrc}XenoRosCommandBridge.cpp
          ${locationXenoSrc}XenoRosDataBridge.cpp
          ${locationXenoSrc}XenoSpiControllerHandler.cpp
          ${locationXenoSrc}XenoFrt20Sim.cpp
)
add_executable( template_20sim main.cpp 
                ${XENO}
                ${SRC}
                controller/LoopController.cpp
                ${CONTROLLER_SRC} 
                )

# Add the constant TIME_STEP_SIZE to definitions
target_compile_definitions(main PRIVATE TIME_STEP_SIZE=${TIME_STEP_SIZE_VALUE})

ament_target_dependencies(template_20sim PUBLIC rclcpp xrf2_msgs std_msgs)

# Specify paths to header files
target_include_directories( template_20sim BEFORE PUBLIC 
                            /usr/evl/include 
                            include/ 
                            ${locationXenoInclude}
                            ${locationCommonInclude}
                            controller/
                            controller/common/
                            )

# Specify paths to library files
target_link_directories(template_20sim PUBLIC /usr/evl/lib)

target_link_libraries(template_20sim PRIVATE evl)
