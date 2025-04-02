// This code is made to unit test the XenoFrtMonitor

#include "XenoFrtMonitor.hpp"
#include <iostream>
#include <error.h>

int main(int argc, char *argv[])
{
    printf("This unit test will simulate 5 sec of run time\n");
    printf("Give frequency of the loop in Hz\n");
    int loop_frequency;
    std::cin >> loop_frequency;

    printf("Give frequency of the monitor in Hz\n");
    int monitor_frequency;
    std::cin >> monitor_frequency;

    XenoFrtMonitor monitor(loop_frequency, monitor_frequency);
    
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

    //The loop will     
    for (size_t i = 1; i <= 5*loop_frequency; i++)
    {
        monitor.update();
        monitor.printf("This is printed in the %d cycle\n", i);
    }

    return 0;
}
