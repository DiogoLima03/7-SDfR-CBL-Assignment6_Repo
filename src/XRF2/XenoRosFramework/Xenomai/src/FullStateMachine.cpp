#include "FullStateMachine.hpp"
/** 
 * @brief Constructor to initialize state machine with given state functions.
 * 
 * @param _initialising Function for the initialising state.
 * @param _initialised Function for the initialised state.
 * @param _run Function for the run state.
 * @param _stopping Function for the stopping state.
 * @param _stopped Function for the stopped state.
 * @param _pausing Function for the pausing state.
 * @param _paused Function for the paused state.
 * @param _error Function for the error state.
*/
FullStateMachine::FullStateMachine( stateFcn _initialising,
                                    stateFcn _initialised,
                                    stateFcn _run,
                                    stateFcn _stopping,
                                    stateFcn _stopped,
                                    stateFcn _pausing,
                                    stateFcn _paused,
                                    stateFcn _error):
    initialising(_initialising),
    initialised(_initialised),
    run(_run),
    stopping(_stopping),
    stopped(_stopped),
    pausing(_pausing),
    paused(_paused),
    error(_error),
    current_state(IDLE_STATE)
{
    
}

/** 
 * @brief Destructor for FullStateMachine class.
*/
FullStateMachine::~FullStateMachine()
{
    
}

/**
 * @brief Executes the current state function and manages state transitions.
 * 
 * This function calls the current state's function and updates the state machine's
 * current and previous states based on the return value of the state function. 
 * If the state function returns 1, it transitions to the next state. If it returns 
 * a negative value, the state machine transitions to the error state. The function 
 * also handles special conditions, such as the quit state, where the loop terminates.
 * 
 * @return 1 if the current state is QUIT_STATE, indicating the loop is complete.
 *         0 if the current state is IDLE_STATE or if no state function is defined.
 *         Otherwise, it returns 0 after executing the state function and handling state transitions.
 */

int FullStateMachine::step()
{
    // If current state is QUIT, looping is complete
    if(current_state == QUIT_STATE)
        return 1;
    // If current state is IDLE nothing wil happen till initialise command has been given
    if((current_state == IDLE_STATE) || (current_state_fcn == nullptr))
        return 0;
    // call current state function
    int ret = current_state_fcn();
    // update the current and previous state to their new values based on the return value
    if(ret != 0)
    {
        if(ret == 1)
        {
            if(current_state == INITIALISING_STATE)
            {
                current_state_fcn =  initialised;
                previous_state = current_state;
                current_state = INITIALISED_STATE;
            }  
            else if(current_state == INITIALISED_STATE)
            {
                current_state_fcn = run;
                previous_state = current_state;
                current_state = RUN_STATE;
            }
            else if(current_state == RUN_STATE)
            {
                current_state_fcn =  stopping;
                previous_state = current_state;
                current_state = STOPPING_STATE;
            }
            else if(current_state == STOPPING_STATE)
            {
                current_state_fcn =  stopped;
                previous_state = current_state;
                current_state = STOPPED_STATE;
            }
            else if(current_state == PAUSING_STATE)
            {
                
                current_state_fcn = paused;
                previous_state = current_state;
                current_state = PAUSED_STATE;
            }
        }       
        else if((ret < 0))
        {
            current_state_fcn = error;
            previous_state = current_state;
            current_state = ERROR_STATE;
            last_error = ret;
        }

        #if STATE_MACHINE_DEBUG
        // Debug message
        printf("current state:  %d \n", current_state);
        #endif
    }
    return 0;
}



/** 
 * @brief Initialises the state machine.
 * 
 * Transition to the `initialising` state.
 * 
 * @return A return value of 1 signifies success, and 0 signifies failure of the operation.
 */
int FullStateMachine::initialise()
{
    if(current_state == IDLE_STATE)
    {
        previous_state = current_state;
        current_state = INITIALISING_STATE;
        current_state_fcn = initialising;
        return 1;
    }    
    return 0;
}

/** 
 * @brief Starts the state machine.
 * 
 * Transition to the `run` state if the system is initialised or paused.
 * 
 * @return A return value of 1 signifies success, and 0 signifies failure of the operation.
 */
int FullStateMachine::start()
{
    if((current_state == INITIALISED_STATE)||(current_state == PAUSED_STATE))
    {
        previous_state = current_state;
        current_state = RUN_STATE;
        current_state_fcn = run;
        return 1;
    }    
    return 0;
}

/** 
 * @brief Stops the state machine.
 * 
 * Transition to the `stopping` state if the system is running or paused.
 * 
 * @return A return value of 1 signifies success, and 0 signifies failure of the operation.
 */
int FullStateMachine::stop()
{
    if((current_state == RUN_STATE)||(current_state == PAUSED_STATE))
    {
        previous_state = current_state;
        current_state = STOPPING_STATE;
        current_state_fcn = stopping;
        return 1;
    }   
    return 0; 
}

/** 
 * @brief Resets the state machine to the idle state.
 * 
 * Transition to the `initialising` state if the system is stopped, in error, or running.
 * 
 * @return A return value of 1 signifies success, and 0 signifies failure of the operation.
 */
int FullStateMachine::reset()
{
    if((current_state == STOPPED_STATE)||(current_state == ERROR_STATE)||(current_state == RUN_STATE))
    {
        previous_state = current_state;
        current_state = INITIALISING_STATE;
        current_state_fcn = initialising;
        return 1;
    }    
    return 0;
}

/** 
 * @brief Causes the state machine to go to the pausing state.
 * 
 * Transition to the `pausing` state if the system is running.
 * 
 * @return A return value of 1 signifies success, and 0 signifies failure of the operation.
 */
int FullStateMachine::pause()
{
    if(current_state == RUN_STATE)
    {
        previous_state = current_state;
        current_state = PAUSING_STATE;
        current_state_fcn = pausing;
        return 1;
    }   
    return 0; 
}

/** 
 * @brief Quits the state machine.
 * 
 * Transition to the `quit` state if the system is stopped or in error.
 * 
 * @return A return value of 1 signifies success, and 0 signifies failure of the operation.
 */
int FullStateMachine::quit()
{
    if((current_state == STOPPED_STATE)||(current_state == ERROR_STATE))
    {
        previous_state = current_state;
        current_state = QUIT_STATE;
        current_state_fcn = nullptr;
        return 1;
    }   
    return 0; 
}