#include "XenoFrtRosIco.hpp"
#include "FullStateMachine.hpp"

/**
     * @brief Constructor for XenoFrtRosIco class.
     * 
     * @param _cycle_time_freq The frequency of the loop cycle time in Hz.
     * @param write_decimator_freq The frequency at which data is decimated for writing.
     * @param monitor_freq Frequency of monitoring data.
     * @param _log_file_handler Reference to the log file handler.
     * @param data_to_be_logged_pntr Pointer to data that needs to be logged.
     */
XenoFrtRosIco::XenoFrtRosIco(uint _cycle_time_freq, uint write_decimator_freq, uint monitor_freq, XenoFileHandler& _log_file_handler, void *data_to_be_logged_pntr): 
    XenoFrtLoop(_cycle_time_freq),
    state_machine(  std::bind(&XenoFrtRosIco::initialising, this),
                    std::bind(&XenoFrtRosIco::initialised, this),
                    std::bind(&XenoFrtRosIco::run, this),
                    std::bind(&XenoFrtRosIco::stopping, this),
                    std::bind(&XenoFrtRosIco::stopped, this),
                    std::bind(&XenoFrtRosIco::pausing, this),
                    std::bind(&XenoFrtRosIco::paused, this),
                    std::bind(&XenoFrtRosIco::error, this)),
    spi_controller((uint8_t)1, 12, (uint32_t)31 * MILLION), 
    ico_io(spi_controller),
    command_com("Control", sizeof(std_msgs::msg::Int32), sizeof(std_msgs::msg::Int32) * 10),
    command_bridge(command_com, state_machine, _cycle_time_freq, write_decimator_freq),
    data_com("Ros-Xeno", sizeof(xrf2_msgs::msg::Ros2Xeno) * 1, sizeof(xrf2_msgs::msg::Xeno2Ros) * 1000),
    data_bridge(data_com, _cycle_time_freq, write_decimator_freq, &ros_data , &xeno_data),
    monitor(_cycle_time_freq, monitor_freq),
    logger(_log_file_handler, data_to_be_logged_pntr)
{
    printf("%s: Constructing rampio\n", __FUNCTION__);
}

/**
     * @brief Destructor for XenoFrtRosIco class.
*/
XenoFrtRosIco::~XenoFrtRosIco()
{
}

/**
 * @brief The step function is called on each loop iteration to handle the main processing.
 * 
 * This function performs the following tasks:
 * 1. Updates sample_data and sends actuate data to the FPGA via `ico_io`.
 * 2. Updates ros_data variable with current data and sent xeno_data to ROS2.
 * 3. Send current state to ROS2 and check for received commands from ROS2.
 * 4. Updates the monitoring system via `monitor`.
 * 5. Executes current satte of the state machine.
 * 6. Storring log data in preset file.
 * 
 * @return Integer status code indicating the outcome of the step operation.
 */
int XenoFrtRosIco::step()
{
    #if GET_TIMINGS
    evl_read_clock(EVL_CLOCK_MONOTONIC, &t1);
    #endif
    
    ico_io.update_io(actuate_data, &sample_data);       //  1) Communication with the FPGA
    
    #if GET_TIMINGS
    evl_read_clock(EVL_CLOCK_MONOTONIC, &t2);
    #endif 
    
    data_bridge.update();                           //  2) Communicate data on the data bridge to ros
    
    #if GET_TIMINGS
    evl_read_clock(EVL_CLOCK_MONOTONIC, &t3);
    #endif
    
    command_bridge.update();                        //  3) Communicate command data on the command bridge to ros
    
    #if GET_TIMINGS
    evl_read_clock(EVL_CLOCK_MONOTONIC, &t4);
    #endif
    
    monitor.update();                               //  4) Update the depricator of the monitor
    
    #if GET_TIMINGS
    evl_read_clock(EVL_CLOCK_MONOTONIC, &t5);
    #endif
    
    int ret = state_machine.step();                 //  5) Call the step function
    
    #if GET_TIMINGS
    evl_read_clock(EVL_CLOCK_MONOTONIC, &t6);
    #endif
    
    logger.log();                                   // 6) Log data if needed to
    
    #if GET_TIMINGS
    evl_read_clock(EVL_CLOCK_MONOTONIC, &t7);
    #endif
    return ret;
}


