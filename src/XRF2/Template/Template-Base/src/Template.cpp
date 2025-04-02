#include "Template.hpp"

Template::Template(uint cycle_time_freq, uint write_decimator_freq, uint monitor_freq) :
    XenoFrtRosIco(cycle_time_freq, write_decimator_freq, monitor_freq, file, &data_to_be_logged),
    file(1,"/temp/template","bin")
{
     printf("%s: Constructing rampio\n", __FUNCTION__);
    // Add variables to logger to be logged, has to be done before you can log data
    logger.addVariable("this_is_a_int", integer);
    logger.addVariable("this_is_a_double", double_);
    logger.addVariable("this_is_a_float", float_);
    logger.addVariable("this_is_a_char", character);
    logger.addVariable("this_is_a_bool", boolean);
    // The logger has to be initialised at only once
    if(!logger.isInitialised())
        logger.initialise();
}

Template::~Template()
{
    
}

int Template::initialising()
{
    // Set physical and cyber system up for use
    // Return 1 to go to initialised state

    evl_printf("Hello from initialising\n");      // Do something

    // The FPGA has to be initialised at least once
    ico_io.init();

    return 1;
}

int Template::initialised()
{
    // Keep the physical syste in a state to be used in the run state
    // Call start() or return 1 to go to run state

    monitor.printf("Hello from initialised\n");       // Do something

    return 0;
}

int Template::run()
{
    // Do what you need to do
    // Return 1 to go to stopping state

    // Start logger
    if(!logger.isStarted())
        logger.start();                             
    
    //  Change some data for logger 
    monitor.printf("Hello from run\n");             
    data_to_be_logged.this_is_a_bool = !data_to_be_logged.this_is_a_bool;
    data_to_be_logged.this_is_a_int++;
    if(data_to_be_logged.this_is_a_char == 'R')
        data_to_be_logged.this_is_a_char = 'A';
    else if (data_to_be_logged.this_is_a_char == 'A')
        data_to_be_logged.this_is_a_char = 'M';
    else
        data_to_be_logged.this_is_a_char = 'R';
    data_to_be_logged.this_is_a_float = data_to_be_logged.this_is_a_float/2;
    data_to_be_logged.this_is_a_double = data_to_be_logged.this_is_a_double/4; 

    // Printf encoder 1 to 4 data
    monitor.printf("Encoder 1 value : %d\n",sample_data.channel1);
    monitor.printf("Encoder 2 value : %d\n",sample_data.channel2);
    monitor.printf("Encoder 3 value : %d\n",sample_data.channel3);
    monitor.printf("Encoder 4 value : %d\n",sample_data.channel4);

    // Set motor outputs to 25% of max
    actuate_data.pwm1 = 2047 * 0.25;
    actuate_data.pwm1 = 2047 * -0.25;
    return 0;
}

int Template::stopping()
{
    // Bring the physical system to a stop and set it in a state that the system can be deactivated
    // Return 1 to go to stopped state
    logger.stop();                                // Stop logger
    evl_printf("Hello from stopping\n");          // Do something

    return 1;
}

int Template::stopped()
{
    // A steady state in which the system can be deactivated whitout harming the physical system
    // Call reset command to go to the initialising state

    monitor.printf("Hello from stopped\n");          // Do something

    return 0;
}

int Template::pausing()
{
    // Bring the physical system to a stop as fast as possible without causing harm to the physical system

    evl_printf("Hello from pausing\n");           // Do something
    return 1 ;
}

int Template::paused()
{
    // Keep the physical system in the current physical state

    monitor.printf("Hello from paused\n");            // Do something
    return 0;
}

int Template::error()
{
    // Error detected in the system 
    // Can go to error if the previous state returns 1 from every other state function but initialising 

    monitor.printf("Hello from error\n");             // Do something

    return 0;
}
