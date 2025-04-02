#include "XenoFrt20Sim.hpp"

/**
 * @brief Constructs a XenoFrt20Sim object.
 * 
 * Inherits from the base class XenoFrtRosIco and configures parameters such as write decimation, 
 * monitoring frequency, and logging functionality.
 * 
 * @param write_decimator_freq Frequency for write decimation, controlling data writing intervals.
 * @param monitor_freq Frequency for monitoring operations, ensuring timely system checks.
 * @param _log_file_handler Reference to the file handler for logging operations.
 * @param data_to_be_logged_pntr Pointer to the data structure that needs to be logged.
 */
XenoFrt20Sim::XenoFrt20Sim(uint write_decimator_freq, uint monitor_freq, XenoFileHandler& _log_file_handler, void *data_to_be_logged_pntr) :
    XenoFrtRosIco(uint(1 / TIME_STEP_SIZE), write_decimator_freq, monitor_freq, _log_file_handler, data_to_be_logged_pntr)
{
    // Print a message indicating successful construction.
    printf("%s: Constructing rampio\n", __FUNCTION__);
}

/**
 * @brief Destroys the XenoFrt20Sim object.
 * 
 * Cleans up resources allocated during the object's lifetime.
 */
XenoFrt20Sim::~XenoFrt20Sim()
{
    printf("%s: Destructing rampio\n", __FUNCTION__);
}

