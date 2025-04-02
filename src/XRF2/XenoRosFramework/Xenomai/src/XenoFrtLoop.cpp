#include "XenoFrtLoop.hpp"

/**
 * @brief Initializes the loop and sets up real-time scheduling.
 * 
 * @param _cycle_time_freq The frequency of the cycle in Hz.
 */
XenoFrtLoop::XenoFrtLoop(uint _cycle_time_freq):
    cycle_time_freq(_cycle_time_freq)
{
    printf("%s: Constructing rampio\n", __FUNCTION__);
    cycle_time = SEC / cycle_time_freq;
    evl_printf("Cycle frequency :    %d Hz\n",cycle_time_freq);
    evl_printf("Cycle time :         %d msec\n",cycle_time/MSEC);
}

/// @brief Destructor for cleaning up resources.
XenoFrtLoop::~XenoFrtLoop()
{
    printf("%s: Destructing rampio\n", __FUNCTION__);
}

/**
 * @brief Main loop function that runs until `step()` returns a non-zero value.
 * 
 * This function attaches the current thread to the EVL core, sets scheduling
 * attributes, and enters a loop where it calls the `step()` function each cycle.
 * It tracks cycle time, calculates missed cycles, and adjusts the loop's timing 
 * to maintain real-time performance.
 * 
 * @return Returns 0 if the loop runs successfully; otherwise, it returns an error code.
 */
int XenoFrtLoop::loop()
{
    bool exit = false;

    // Attack thread to evl core
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

    evl_printf("Start with looping.\n");
    
    // Get start time of the first loop
    evl_read_clock(EVL_CLOCK_MONOTONIC, &start_time);
    wakeup_time = start_time;
    while (!exit)
    {
        // Run one step and increase cycle count
        ret = step();
        cycle_count = cycle_count + 1;

        //Check return value of step() 
        //      >0 means done with looping
        //      ==0 means keep continuing with loop
        if(ret != 0)
            exit = true;

        //Calculate new wake up time  
        wakeup_time.tv_nsec = wakeup_time.tv_nsec + cycle_time;
        if(wakeup_time.tv_nsec >= SEC)
        {
            wakeup_time.tv_nsec = wakeup_time.tv_nsec - SEC;
            wakeup_time.tv_sec = wakeup_time.tv_sec + 1;
        }

        //Check if any cycles are missed
        evl_read_clock(EVL_CLOCK_MONOTONIC, &idle_time);
        if((idle_time.tv_sec > wakeup_time.tv_sec) ||((idle_time.tv_sec == wakeup_time.tv_sec)&&(idle_time.tv_nsec > wakeup_time.tv_nsec)))
            missed_cycles++;

        #if DEBUG_LOOP
            evl_printf("I start sleeping at:  %ds, %dns \n", idle_time.tv_sec, idle_time.tv_nsec);
            evl_printf("I am going to sleep till:  %ds, %dns \n", wakeup_time.tv_sec, wakeup_time.tv_nsec);
        #endif
        
        evl_sleep_until(EVL_CLOCK_MONOTONIC, &wakeup_time);
    }

    // Calculate total run time
    evl_read_clock(EVL_CLOCK_MONOTONIC, &stop_time);
    run_time.tv_sec = stop_time.tv_sec - start_time.tv_sec;
    if (stop_time.tv_nsec < start_time.tv_nsec)
    {
        run_time.tv_nsec = SEC + stop_time.tv_nsec - start_time.tv_nsec;
        run_time.tv_sec -=1;
    }
    else
        run_time.tv_nsec = stop_time.tv_nsec - start_time.tv_nsec;

    // Printf a relevant info
    evl_printf("Runtime: %d sec, %d nsec \n", run_time.tv_sec, run_time.tv_nsec);
    evl_printf("Cycle count: %ld\n", cycle_count);
    evl_printf("Missed cycles count: %d\n", missed_cycles);

    return ret;
    
}

