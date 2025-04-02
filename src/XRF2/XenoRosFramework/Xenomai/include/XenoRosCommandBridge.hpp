#ifndef XENOROSCOMMANDBRIDGE_HPP
#define XENOROSCOMMANDBRIDGE_HPP

// A type of this class we be used in one of the included .h files
class FullStateMachine;
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

#include <evl/evl.h>

#include "ICommunication.h"
#include "debug_settings.h"

//ROS standard message type int32
#include <std_msgs/msg/int32.hpp>

#define MSEC        1000000L
#define SEC         1000000000L


/**
 * @enum COMMANDS
 * @brief Enumeration of available commands for the state machine.
 */
enum COMMANDS { INITIALISE_CMD  = 1, 
                START_CMD       = 2, 
                STOP_CMD        = 3, 
                RESET_CMD       = 4, 
                PAUSE_CMD       = 5, 
                QUIT_CMD        = 6};


/**
 * @class XenoRosCommandBridge
 * @brief Manages communication between Xenomai and ROS.
 *
 * Handles commands received from ROS, updates the state machine, and manages write decimation.
 */
class XenoRosCommandBridge
{
public:
    XenoRosCommandBridge(ICommunication& _communicator, FullStateMachine& _state_machine, uint _cycle_time_freq, uint _write_decimator_freq);
    ~XenoRosCommandBridge();

    // Check if any command has been received and read and execute them  
    // Updates the decimation for the write and if decimation has reached 0 write the current state
    int update();

private:
    ICommunication &communicator;                    ///< Reference to communicator of the type IXenoComm.
    FullStateMachine &state_machine;                 ///< Reference to state machine.

    uint cycle_time_freq;                            ///< Cycle time in Hz of the loop.
    uint cycle_time;                                 ///< Cycle time in milliseconds of the loop.
    uint write_decimator_freq;                       ///< Frequency of the write decimation.
    uint decimation_total;                           ///< Total decimation for one write cycle.
    uint decimation_counter;                         ///< Current decimation till write.

    bool write_flag;                                 ///< Flag indicating whether to write data.

    std_msgs::msg::Int32 ros_data;                   ///< Data received from ROS implementation.
    std_msgs::msg::Int32 xeno_data;                  ///< Data from this Xenomai thread.
    int data_size = sizeof(std_msgs::msg::Int32);    ///< Size of the data.
};

#endif // XENOROSCOMMANDBRIDGE_HPP