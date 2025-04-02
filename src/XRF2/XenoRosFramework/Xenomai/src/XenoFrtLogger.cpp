#include "XenoFrtLogger.hpp"

/**
 * @brief Construct a new Xeno Frt Logger:: Xeno Frt Logger object
 * 
 * @param _file_handler Reference to the log file handler.
 * @param _data_ptr Pointer to data that needs to be logged.
 */
XenoFrtLogger::XenoFrtLogger(ICommunication& _file_handler, void* _data_ptr):
    file_handler(_file_handler)
{
    printf("%s: Constructing rampio\n", __FUNCTION__);
    logger_info.data_ptr = _data_ptr;
    logger_info.data_size = 0;
    logger_info.number_var = 0;
}

/**
 * @brief Destroy the XenoFrtLogger object
 */
XenoFrtLogger::~XenoFrtLogger()
{
    printf("%s: Destructing rampio\n", __FUNCTION__);
}

/**
 * @brief Initializes the logger by preparing headers and metadata in the log file.
 * 
 * This function opens the log file and writes the names and types of the variables to the file.
 * It prepares the file for storing actual data entries.
 * 
 * @return int Returns 1 on success, negative value on error.
 */
int XenoFrtLogger::initialise()
{
    char str[6];
    // Open & create file to write to
    file_handler.open();
    int ret;

    for (int i = 0; i < logger_info.number_var; i++)
    {
        // Add first row of variable names
        ret = file_handler.write(logger_info.var_info[i].name, strlen(logger_info.var_info[i].name));
        #if LOGGER_DEBUG
        evl_printf("Logger variable %d, name = %s, name length = %d \n", i, logger_info.var_info[i].name, strlen(logger_info.var_info[i].name));
        #endif
        if(ret<=0)
            error(1, errno, "XenoFrtLogger : file_handler.write()");
        // Add ',' except for the last variable
        if(i != (logger_info.number_var-1))
        {
            sprintf(str, ",");
            ret = file_handler.write(str, strlen(str));
            if(ret<=0)
                error(1, errno, "XenoFrtLogger : file_handler.write()");
        }
    }
    sprintf(str, "\n");
    ret = file_handler.write(str, strlen(str));
    if(ret<=0)
            error(1, errno, "XenoFrtLogger : file_handler.write()");

    for(int i = 0; i < logger_info.number_var; i++)
    {
        sprintf(str, "%d", logger_info.var_info[i].type);
        ret = file_handler.write(str, strlen(str));
        #if LOGGER_DEBUG
        evl_printf("Logger variable %d, type = %s \n", i, str);
        #endif
        if(ret<=0)
            error(1, errno, "XenoFrtLogger : file_handler.write()");

        if(i != (logger_info.number_var-1))
        {
            sprintf(str, ",");
            ret = file_handler.write(str, strlen(str));
            if(ret<=0)
                error(1, errno, "XenoFrtLogger : file_handler.write()");
        }
    }
    sprintf(str, "\n");
    ret = file_handler.write(str, strlen(str));
    if(ret<=0)
            error(1, errno, "XenoFrtLogger : file_handler.write()");
    
    initialsed = true;

    return 1;
}

/**
 * @brief Adds a variable to the logger.
 * 
 * This function allows adding a variable name and its type to the logger.
 * It updates the total data size based on the variable type.
 * 
 * @param name Name of the variable.
 * @param type Type of the variable.
 * @return int Returns 1 on success, -1 if the maximum number of variables is exceeded.
 */
int XenoFrtLogger::addVariable(const char* name, VariableTypes type)
{
    assert(!initialsed);
    
    if(logger_info.number_var == 100)
        return -1;

    logger_info.var_info[logger_info.number_var].name = name;
    logger_info.var_info[logger_info.number_var].type = type;

    if(logger_info.var_info[logger_info.number_var].type <= 2)
    {
        logger_info.data_size = logger_info.data_size + 1;
    }  
    else if(logger_info.var_info[logger_info.number_var].type <= 4)
    {
        logger_info.data_size = logger_info.data_size + 4;
    }  
    else if(logger_info.var_info[logger_info.number_var].type >= 5)
    {
        logger_info.data_size = logger_info.data_size + 8;
    }
        
    #if LOGGER_DEBUG
    evl_printf("Added a variable: %s, type: %d, current data size: %d \n",logger_info.var_info[logger_info.number_var].name, logger_info.var_info[logger_info.number_var].type, logger_info.data_size);
    #endif
    
    logger_info.number_var = logger_info.number_var + 1;
    
    return 1;
}

/**
 * @brief Logs the current data.
 * 
 * This function writes the data to the file if the logger has been started.
 * It writes the current data pointer to the file, with the size defined during initialization.
 * 
 * @return int Returns the number of bytes written, or -1 if logging is not started.
 */
int XenoFrtLogger::log()
{   
    if(!started)
        return -1;
    assert(initialsed);

    return file_handler.write(logger_info.data_ptr, logger_info.data_size);
}

/**
 * @brief Starts the logging process.
 * 
 * This function starts the logging process, allowing data to be written to the log file.
 * 
 * @return int Returns 1 on success.
 */
int XenoFrtLogger::start()
{   
    started = true;
    return 1;
}

/**
 * @brief Stops the logging process.
 * 
 * This function stops the logging process and prevents any further data from being logged.
 * 
 * @return int Returns 1 on success.
 */
int XenoFrtLogger::stop()
{   
    started = false;
    return 1;
}
