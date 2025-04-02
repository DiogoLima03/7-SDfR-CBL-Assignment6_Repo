#ifndef XENOROSDATABRIDGE_HPP
#define XENOROSDATABRIDGE_HPP

#include <unistd.h>
#include <stdint.h>
#include <cstdio>

#include <sys/types.h>
#include <sys/sysinfo.h>

#include <error.h>
#include <errno.h>
#include <fcntl.h>
#include <string.h>
#include <time.h>

#include "ICommunication.h"

#include "xrf2_msgs/msg/ros2_xeno.hpp"
#include "xrf2_msgs/msg/xeno2_ros.hpp"

#define MSEC        1000000L
#define SEC         1000000000L

/**
 * @class XenoRosDataBridge
 * @brief Bridge class for communication between Xenomai and ROS 2 using custom messages.
 * 
 * This class facilitates the communication between ROS 2 and Xenomai by providing methods
 * to handle data transfer between the systems, including decimation and updating data states.
 * 
 * Data will only be sent to ROS after the first ROS message is received.
 * 
 */
class XenoRosDataBridge
{
public:
    XenoRosDataBridge(ICommunication& _communicator, uint _cycle_time_freq, uint _write_decimator_freq, xrf2_msgs::msg::Ros2Xeno* _ros_data_ptr, xrf2_msgs::msg::Xeno2Ros* _xeno_data_ptr);
    ~XenoRosDataBridge();

    // Check if any command has been received and read and execute them  
    // Updates the decimation for the write and if decimation has reached 0 write the current state
    int update();

private:
    ICommunication &communicator;                               ///< Reference to the communication interface (e.g., XenoRosCommunication)
    uint cycle_time_freq;                                       ///< Frequency of the loop cycle in Hz.
    uint cycle_time;                                            ///< Cycle time in milliseconds.
    uint write_decimator_freq;                                  ///< Frequency of write decimation.
    uint decimation_total;                                      ///< Total decimation for one write cycle.
    uint decimation_counter;                                    ///< Current decimation count until write.

    bool write_flag;                                            ///< Flag indicating whether data should be written.

    xrf2_msgs::msg::Ros2Xeno* ros_data_ptr;                   ///< Pointer to the ROS data (Ros2Xeno message).
    xrf2_msgs::msg::Xeno2Ros* xeno_data_ptr;                  ///< Pointer to the Xenomai data (Xeno2Ros message).
    int ros_data_size = sizeof(xrf2_msgs::msg::Ros2Xeno);     ///< Size of the ROS message Ros2Xeno.
    int xeno_data_size = sizeof(xrf2_msgs::msg::Xeno2Ros);    ///< Size of the ROS message Xeno2Ros.
};
#endif // XENOROSDATABRIDGE_HPP