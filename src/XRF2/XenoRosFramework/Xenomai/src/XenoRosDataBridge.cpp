#include "XenoRosDataBridge.hpp"

/**
 * @brief Constructs a XenoRosDataBridge object.
 * 
 * Initializes the communication bridge between ROS 2 and Xenomai. Configures cycle times,
 * decimation parameters, and opens the communication interface.
 * 
 * @param _communicator Reference to the communication interface used for data exchange.
 * @param _cycle_time_freq Frequency (in Hz) for the loop cycle.
 * @param _write_decimator_freq Frequency for the decimation process to control write intervals.
 * @param _ros_data_ptr Pointer to the ROS data structure for receiving data.
 * @param _xeno_data_ptr Pointer to the Xenomai data structure for sending data.
 */
XenoRosDataBridge::XenoRosDataBridge(ICommunication& _communicator, uint _cycle_time_freq, uint _write_decimator_freq, xrf2_msgs::msg::Ros2Xeno* _ros_data_ptr, xrf2_msgs::msg::Xeno2Ros* _xeno_data_ptr):
    communicator(_communicator),
    cycle_time_freq(_cycle_time_freq),
    write_decimator_freq(_write_decimator_freq),
    write_flag(0),
    ros_data_ptr(_ros_data_ptr),
    xeno_data_ptr(_xeno_data_ptr)
{
    printf("%s: Constructing rampio\n", __FUNCTION__);
    cycle_time = SEC/cycle_time_freq;
    decimation_total = cycle_time_freq/write_decimator_freq;
    decimation_counter = decimation_total - 1;
    write_flag = 0;
    communicator.open();
}

/**
 * @brief Destroys the XenoRosDataBridge object.
 * 
 * Ensures proper closure of the communication interface upon destruction.
 */
XenoRosDataBridge::~XenoRosDataBridge()
{
    communicator.close();
}

/**
 * @brief Updates the data bridge state and handles data exchange.
 * 
 * This method checks if new data is received, manages decimation for writing,
 * and writes data to the communication buffer when conditions are met.
 * 
 * @return int Returns 1 if the update was executed successfully.
 */
int XenoRosDataBridge::update()
{
    int ret = communicator.read(ros_data_ptr, ros_data_size);

    // Can i begin sending data? To limit writing to after a succesfull read
    if(ret>0)
        write_flag = 1;

    //If all requirements are met, write the data in &xeno_data to the buffer   
    if(write_flag && (decimation_counter == 0) && (write_decimator_freq != 0))
    {
        communicator.write(xeno_data_ptr, xeno_data_size);
    }

    if(decimation_counter == 0)
    {
        decimation_counter = decimation_total - 1;
    }
    else
        decimation_counter--;

    return 1;
}
