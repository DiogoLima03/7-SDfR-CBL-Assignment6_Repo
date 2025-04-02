/**
 * @file XenoFrtMonitor.cpp
 * @brief Implementation of the XenoFrtMonitor class for monitoring with adjustable frequency.
 */

#include "XenoFrtMonitor.hpp"

/**
 * @brief Constructs a XenoFrtMonitor object.
 *
 * Initializes the monitor with specified cycle and monitor frequencies.
 * Calculates the monitor delay and ensures it is at least 1.
 *
 * @param _cycle_freq Frequency of the cycle in Hz.
 * @param _monitor_freq Frequency of the monitor in Hz. To disable monitoring set to 0.
 */
XenoFrtMonitor::XenoFrtMonitor(uint _cycle_freq, uint _monitor_freq) 
{
    evl_printf("%s: Constructing rampio\n", __FUNCTION__);
    cycle_freq = _cycle_freq;              
    monitor_freq = _monitor_freq;          

    // Calculating the monitor delay.
    monitor_delay = cycle_freq / monitor_freq;

    // Ensure monitor_delay is at least 1.
    if(monitor_delay < 1)
        monitor_delay = 1;

    current_delay = monitor_delay - 1;     ///< Initialize current delay.

    // Display monitor configuration.
    evl_printf("Monitor frequency = %d\n", monitor_freq);
    evl_printf("Monitor delay = %d\n", monitor_delay);
}

/**
 * @brief Destructor for the XenoFrtMonitor class.
 *
 * Cleans up any allocated resources or settings.
 */
XenoFrtMonitor::~XenoFrtMonitor()
{
    evl_printf("%s: Destructing rampio\n", __FUNCTION__);
}

/**
 * @brief Updates the monitor's current delay.
 *
 * Decrements the current delay counter and resets it if necessary.
 * Skips processing if the monitor frequency is zero.
 */
void XenoFrtMonitor::update()
{
    // If monitor_freq == 0 then nothing will be printed.
    if(monitor_freq == 0)
        return;

    // Decrement or reset current delay.
    if(current_delay == 0)
        current_delay = monitor_delay - 1;
    else
        current_delay--;
}

/**
 * @brief Prints formatted output when the delay condition is met.
 *
 * Uses variable arguments to format and print output when the current delay reaches zero.
 *
 * @param fmt Format string for the output.
 * @param ... Additional arguments for formatting.
 */
void XenoFrtMonitor::printf(const char* fmt, ...)
{
    // Variable argument list for formatting.
    va_list arg;

    // Only print when current_delay reaches 0.
    if(current_delay == 0)
    {
        va_start(arg, fmt); //Initialize argument list.

        // Print formatted output using proxy function.
        evl_vprint_proxy(proxy_outfd, fmt, arg);

        va_end(arg); // Clean up argument list.
    }
}
