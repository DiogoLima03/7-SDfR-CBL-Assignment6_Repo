/**
 * @file XenoFrtMonitor.hpp
 * @brief Header file for the XenoFrtMonitor class, enabling frequency-based monitoring and formatted output.
 */

#ifndef XENOFRTMONITOR_HPP
#define XENOFRTMONITOR_HPP

#include <stdio.h> 
#include <stdarg.h>

#include "evl/evl.h"
#include "evl/proxy.h"

/** @brief Time constants in nanoseconds. */
#define MSEC        1000000L ///< Milliseconds to nanoseconds conversion factor.
#define SEC         1000000000L ///< Seconds to nanoseconds conversion factor.

/**
 * @class XenoFrtMonitor
 * @brief A class for monitoring and printing at a specified frequency relative to a cycle frequency.
 */
class XenoFrtMonitor
{
public:
    XenoFrtMonitor(uint _cycle_freq, uint _monitor_freq); ///< Constructor.
    ~XenoFrtMonitor(); ///< Destructor.

    void update(); ///< Updates the monitor's current delay.
    void printf(const char* fmt, ...); ///< Prints formatted output based on the monitor frequency.

private:
    uint cycle_freq;        ///< Cycle frequency in Hz.
    uint monitor_freq;      ///< Monitor frequency in Hz.
    uint monitor_delay;     ///< Delay between monitor outputs relative to cycle frequency.
    uint current_delay;     ///< Current delay counter till the next formatted output.
};

#endif // XENOFRTMONITOR_HPP
