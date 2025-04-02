// This code is made to unit test the XenoFrtLogger
// The result of this code is a binairy file from which the first row is the names of the variables that has been logger
// The second row is the indentifiers for the data types
// From the third row and onwars the file contains the data that has been logged
// Use the python code binairy_to_mat or binairy_to_csv contained in XenoRosFramework/Common/XenoFrtLogger_decoder to decode the binairy file to a .csv or .mat file

#include "XenoFrtLogger.hpp"

// Struct containing the data to be logger
#pragma pack (1)    //https://carlosvin.github.io/langs/en/posts/cpp-pragma-pack/#_performance_test
struct LoggedVariable
{
    int this_is_a_int = 0;
    double this_is_a_double = 100.0;
    float this_is_a_float = 10.0;
    char this_is_a_char = 'R';
    bool this_is_a_bool = false;
};
#pragma pack(0)



int main(int argc, char *argv[])
{
    //Variables
    XenoFileHandler file(1,"/temp/UT_Logger","bin");
    struct LoggedVariable data_to_be_logged;
    XenoFrtLogger logger(file, &data_to_be_logged);
    

    // Add data to be logger
    logger.addVariable("this_is_a_int", integer);
    logger.addVariable("this_is_a_double", double_);
    logger.addVariable("this_is_a_float", float_);
    logger.addVariable("this_is_a_char", character);
    logger.addVariable("this_is_a_bool", boolean);
    // Initialise file for log data
    if(!logger.isInitialised())
        logger.initialise();

    //Start logger
    logger.start();
    
    //Do first initial log
    logger.log();

    for (size_t i = 0; i < 99; i++)
    {
        // Change the data to be logger 
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
        // Log data
        logger.log();
    }
    
    //Switch to Xenomai kernel
     int tfd = evl_attach_self("XenoThread");
    if (tfd < 0)
        error(1, errno, "evl_attach_self()");
    evl_printf("Attached to evl.\n");

    // Set shed attributes for thread
    struct evl_sched_attrs attrs;
    attrs.sched_policy = SCHED_FIFO;
    attrs.sched_priority = 1;
    int ret = evl_set_schedattr(tfd, &attrs);
    if(ret < 0)
        error(1, errno, "evl_set_schedattr()");

    for (size_t i = 0; i < 100; i++)
    {
        // Change the data to be logger 
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
        // Log data
        logger.log();
    }

    return 0;
}
