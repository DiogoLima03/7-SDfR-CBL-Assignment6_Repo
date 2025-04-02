#ifndef XENOFRTROSICO_HPP
#define XENOFRTROSICO_HPP

class FullStateMachine;

// Standard library
#include <functional>

// Custom includes
#include "XenoFrtLoop.hpp"
#include "XenoSpiControllerHandler.hpp"
#include "IcoIo.hpp"
#include "XenoFileHandler.hpp"
#include "XenoFrtMonitor.hpp"
#include "XenoFrtLogger.hpp"
#include "XenoRosCommunication.hpp"
#include "XenoRosDataBridge.hpp"
#include "XenoRosCommandBridge.hpp"

#include "xrf2_msgs/msg/ros2_xeno.hpp"
#include "xrf2_msgs/msg/xeno2_ros.hpp"

#include "configuration.h"

/**
 * @class XenoFrtRosIco
 * @brief A class that extends XenoFrtLoop to manage SPI communication and interaction with ROS.
 * 
 * This class interfaces with various communication protocols, such as SPI for hardware 
 * control and ROS for data communication. It handles data transfer between the FPGA 
 * (via IcoIo) and ROS (via XenoRosCommunication), and it monitors and logs system 
 * performance through XenoFrtMonitor and XenoFrtLogger.
 */
class XenoFrtRosIco : public XenoFrtLoop
{
public:
    
    XenoFrtRosIco(uint _cycle_time_freq, uint write_decimator_freq, uint monitor_freq, XenoFileHandler& _log_file_handler, void *data_to_be_logged_pntr);
    ~XenoFrtRosIco();

private:
    // Class instance
    FullStateMachine state_machine;

protected:
    /**
     * @brief Placeholder for initialising state.
     * 
     * @return Integer status code indicating success or failure.
     */
    virtual int initialising(){return 1;};

    /**
     * @brief Placeholder for initialised state.
     * 
     * @return Integer status code indicating success or failure.
     */
    virtual int initialised(){return 0;};

    /**
     * @brief Placeholder for running state.
     * 
     * @return Integer status code indicating success or failure.
     */
    virtual int run(){return 1;};

    /**
     * @brief Placeholder for stopping state.
     * 
     * @return Integer status code indicating success or failure.
     */
    virtual int stopping(){return 1;};

    /**
     * @brief Placeholder for stopped state.
     * 
     * @return Integer status code indicating success or failure.
     */
    virtual int stopped(){return 0;};

    /**
     * @brief Placeholder for pausing state.
     * 
     * @return Integer status code indicating success or failure.
     */
    virtual int pausing(){return 1;};

    /**
     * @brief Placeholder for paused state.
     * 
     * @return Integer status code indicating success or failure.
     */
    virtual int paused(){return 0;};

    /**
     * @brief Placeholder for error state.
     * 
     * @return Integer status code indicating success or failure.
     */
    virtual int error(){return 0;};

    /**
     * @brief Main step function to be executed in the loop.
     * 
     * @return Integer status code indicating success or failure.
     */
    int step() override;

    // Class instances
    // SPI communication
    XenoSpiControllerHandler spi_controller;
    IcoIo ico_io;
    
    // Control communication
    XenoRosCommunication command_com;
    XenoRosCommandBridge command_bridge;
    
    // Data communication
    XenoRosCommunication data_com;
    XenoRosDataBridge data_bridge;
    
    // Monitoring and logging
    XenoFrtMonitor monitor;
    XenoFrtLogger logger;

    // Variables
    int monitor_freq;                           ///< Monitor frequency of decimator frequency.
    IcoIo::IcoWrite actuate_data;               ///< Data to be written to the FPGA (PWM + digital output data).
    IcoIo::IcoRead sample_data;                 ///< Input data collected from the FPGA (encoder + digital input data).

    xrf2_msgs::msg::Ros2Xeno ros_data;        ///< Data received from ROS implementation.
    xrf2_msgs::msg::Xeno2Ros xeno_data;       ///< Data sent from this Xenomai thread to ROS.

    #if GET_TIMINGS
    struct timespec t1;         ///< Start time of FPGA communication.
    struct timespec t2;         ///< Start time of ROS command data transfer.
    struct timespec t3;         ///< Start time of ROS data transfer.

    struct timespec t4;         ///< Start time of XenoFrtMonitor update.
    struct timespec t5;         ///< Start time of state implementation.
    struct timespec t6;         ///< Start time of logging.
    struct timespec t7;         ///< End time of logging.
    #endif
};

#endif // XENOFRTROSICO_HPP
