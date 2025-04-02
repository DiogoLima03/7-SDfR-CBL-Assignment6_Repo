#ifndef XENOFRTLOOP_HPP
#define XENOFRTLOOP_HPP

#include <time.h>

#include <evl/evl.h>
#include <evl/thread.h>
#include <error.h>

#include "FullStateMachine.hpp"
#include "debug_settings.h"

#define MSEC        1000000L ///< Microseconds in one millisecond
#define SEC         1000000000L ///< Nanoseconds in one second

/// @class XenoFrtLoop
/// @brief A real-time loop for cyclic execution with precise timing.
/// 
/// This class provides a framework for cyclic execution, where the step() method
/// should be implemented in subclasses to define the work done during each cycle.
/// The loop ensures real-time performance by managing the timing of each cycle and
/// tracking any missed deadlines.
class XenoFrtLoop 
{
public:
    
    XenoFrtLoop(uint _cycle_time_freq);
    ~XenoFrtLoop();

    int loop();

    /// @brief Gets the total number of completed cycles.
    /// @return The count of completed cycles.
    int getCyclesCount() { return cycle_count; }

    /// @brief Gets the total number of missed cycles.
    /// @return The count of missed cycles.
    int getMissedCycles() { return missed_cycles; }

    /// @brief Gets the time when the last idle period started.
    /// @return A timespec structure representing the last idle time.
    struct timespec getIdleTime() { return idle_time; }

private:
    int cycle_time;         ///< The time for each cycle to execute (in nanoseconds or microseconds)
    int cycle_count;        ///< The count of completed cycles
    int missed_cycles;      ///< The count of missed cycles due to timing issues

protected:
    uint cycle_time_freq;   ///< The frequency of the cycle time in Hz

    /// @brief A pure virtual function that should be implemented in a derived class to define the work done in each cycle.
    /// @return An integer indicating the success or failure of the cycle step.
    virtual int step() = 0;

    struct timespec start_time;     ///< The time when the loop started running
    struct timespec stop_time;      ///< The time when the loop was stopped
    struct timespec idle_time;      ///< The time when the last idle period started
    struct timespec wakeup_time;    ///< The start time of the current cycle
    struct timespec run_time;       ///< The total run time of the loop since start
};

#endif // XENOFRTLOOP_HPP
