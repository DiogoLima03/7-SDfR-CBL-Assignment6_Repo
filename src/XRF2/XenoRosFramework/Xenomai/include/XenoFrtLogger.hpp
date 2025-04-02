#ifndef XENOFRTLOGGER_HPP
#define XENOFRTLOGGER_HPP

#include "XenoFileHandler.hpp"
#include <string.h>
#include <cassert>

#include "debug_settings.h"
#define MSEC        1000000L
#define SEC         1000000000L

/**
 * @enum VariableTypes
 * @brief Enum to define the types of variables that can be logged.
 */
enum VariableTypes {
    boolean = 1,  ///< Boolean type
    character,    ///< Character type
    integer,      ///< Integer type
    float_,       ///< Float type
    double_,      ///< Double type
    long_int      ///< Long integer type
};

/**
 * @struct variableInfo
 * @brief Struct to store the name and type of a variable to be logged.
 */
struct variableInfo {
    const char* name;        ///< Name of the variable
    VariableTypes type;      ///< Type of the variable (using the VariableTypes enum)
};

/**
 * @struct loggerInfo
 * @brief Struct to hold information about the data being logged.
 */
struct loggerInfo {
    void* data_ptr;            ///< Pointer to the data that needs to be logged
    int data_size;             ///< Size of the data to be logged
    int number_var;            ///< Number of variables to be logged
    variableInfo var_info[100];///< Array holding information about the variables being logged
};

/**
 * @class XenoFrtLogger
 * @brief Class responsible for logging variable data to a file.
 */
class  XenoFrtLogger
{
public:
    XenoFrtLogger(ICommunication& _file_handler, void* _data_ptr);
    ~XenoFrtLogger();

    int log();                                              // Log data
    int addVariable(const char* name, VariableTypes type);  // Add variable to logger info
    int initialise();                                       // Initialise the file to be able to decode the logged data
    bool isInitialised(){return initialsed;};               // Check whether XenoFrtLogger is initialised
    bool isStarted(){return started;};                      // Check Whether XenoFrtLogger is started

    int start();                                            // Start logger
    int stop();                                             // Stop logger
    
private:
    ICommunication &file_handler; ///< Reference to the file handler
    loggerInfo logger_info;       ///< Struct holding information about the data to be logged
    bool initialsed;              ///< Flag indicating whether the logger is initialized
    bool started;                 ///< Flag indicating whether the logger has started
};

#endif // XENOFRTLOGGER_HPP