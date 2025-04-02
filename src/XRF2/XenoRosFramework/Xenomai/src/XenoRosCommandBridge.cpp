#include "XenoRosCommandBridge.hpp"
#include "FullStateMachine.hpp"

/**
 * @brief Constructs a XenoRosCommandBridge object.
 *
 * Initializes communication, sets cycle times, and opens the communicator.
 *
 * @param _communicator Reference to the communication interface.
 * @param _state_machine Reference to the state machine for handling commands.
 * @param _cycle_time_freq Frequency of the update cycle in Hz.
 * @param _write_decimator_freq Frequency of write decimation.
 */
XenoRosCommandBridge::XenoRosCommandBridge(ICommunication& _communicator, FullStateMachine& _state_machine, uint _cycle_time_freq, uint _write_decimator_freq):
    communicator(_communicator),
    state_machine(_state_machine),
    cycle_time_freq(_cycle_time_freq),
    write_decimator_freq(_write_decimator_freq)
{
    printf("%s: Constructing rampio\n", __FUNCTION__);
    // Calculate cycle time and total decimation cycles 
    cycle_time = SEC/cycle_time_freq;
    decimation_total = cycle_time_freq/write_decimator_freq;
    decimation_counter = decimation_total-1;
    write_flag = 0;
    // Open the communicator
    communicator.open();
}

/**
 * @brief Destroys the XenoRosCommandBridge object.
 *
 * Closes the communicator before destruction.
 */
XenoRosCommandBridge::~XenoRosCommandBridge()
{
    // Bridge will be closed so close the communicator
    communicator.close();
}

/**
 * @brief Updates the state machine based on received commands and writes the current state.
 *
 * Reads commands, executes corresponding actions, and writes the current state with decimation control.
 *
 * @return int Returns 1 upon successful update.
 */
int XenoRosCommandBridge::update()
{
    int ret = communicator.read(&ros_data, data_size);
    if(ret > 0)
    {   
        // Can i begin sending data? To limit writing to after a succesfull read
        write_flag = 1;

        // Set "CONTROL_BRIDGE_DEBUG" to 1 in debug_settings.h to see the commando's received during run time
        #if CONTROL_BRIDGE_DEBUG
        evl_printf("Received command from ROS %d\n", ros_data.data);
        #endif

        // Based on the data received call the correct action
        int action_command = ros_data.data;
        switch (action_command)
        {
        case INITIALISE_CMD:
            state_machine.initialise();
            break;

        case START_CMD:
            state_machine.start();
            break;

        case STOP_CMD:
            state_machine.stop();
            break;

        case RESET_CMD:
            state_machine.reset();
            break;

        case PAUSE_CMD:
            state_machine.pause();
            break;

        case QUIT_CMD:
            state_machine.quit();
            break;

        default:
            break;
        } 

    }

    //If all requirements are met, write the data in xeno_data to the buffer 
    if(write_flag && (decimation_counter == 0))
    {
        xeno_data.data = state_machine.getState();
        communicator.write(&xeno_data, data_size);      
    }

    if(decimation_counter == 0)
    {
        decimation_counter = decimation_total - 1;
    }
    else
        decimation_counter--;
    
    return 1;
}
