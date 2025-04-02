#ifndef FULLSTATEMACHINE_HPP
#define FULLSTATEMACHINE_HPP

#include <functional> 

#include "debug_settings.h"

// Define enum STATE of the state machine,
// Numbers are added as states are sent to ROS
/** 
 * @enum STATE
 * @brief Enum representing the different states of the state machine.
 * 
 * Each state corresponds to a specific phase of operation, such as initialization, running, stopping, etc.
 */
enum STATE {IDLE_STATE          = 1, 
            INITIALISING_STATE  = 2, 
            INITIALISED_STATE   = 3, 
            RUN_STATE           = 4, 
            STOPPING_STATE      = 5, 
            STOPPED_STATE       = 6, 
            PAUSING_STATE       = 7, 
            PAUSED_STATE        = 8, 
            ERROR_STATE         = 9, 
            QUIT_STATE          = 10};

/** 
 * @class FullStateMachine
 * @brief Class for managing the state machine in the system.
 * 
 * This class allows for state transitions between different states of the system, such as initializing, running, pausing, and stopping. 
 * It also handles error states and provides functionality to query the current state and any errors that occur.
 * See state_machine.pdf for interaction between these functions and the state machine
 */
class FullStateMachine 
{
public:
    using stateFcn = std::function<int()>; ///< Alias for state function type 

    
    FullStateMachine(   stateFcn _initialising,
                        stateFcn _initialised,
                        stateFcn _run,
                        stateFcn _stopping,
                        stateFcn _stopped,
                        stateFcn _pausing,
                        stateFcn _paused,
                        stateFcn _error
    );
    ~FullStateMachine();

    
    int step();
    int initialise();
    int start();
    int stop();
    int reset();
    int pause();
    int quit();

    /** 
     * @brief Gets the current state of the state machine.
     * 
     * @return The current state.
     */
    STATE getState(){ return current_state; };

    /** 
     * @brief Gets the last error code that occurred during state machine operations.
     * 
     * @return The error code.
     */
    int getError() {return last_error; };

private:
    STATE current_state;    ///< The current active state of the state machine 
    STATE previous_state;   ///< The previous state before the last transition 

    int last_error = 0;     ///< The last error code encountered during execution 

    stateFcn current_state_fcn;  ///< Function corresponding to the current active state 
    stateFcn initialising;       ///< Function for initialising state 
    stateFcn initialised;        ///< Function for initialised state 
    stateFcn run;                ///< Function for run state 
    stateFcn stopping;           ///< Function for stopping state 
    stateFcn stopped;            ///< Function for stopped state 
    stateFcn pausing;            ///< Function for pausing state 
    stateFcn paused;             ///< Function for paused state 
    stateFcn error;              ///< Function for error state 
};

#endif // FULLSTATEMACHINE_HPP
