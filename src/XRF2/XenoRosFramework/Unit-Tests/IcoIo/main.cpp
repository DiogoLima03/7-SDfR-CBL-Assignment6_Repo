#include <iostream>
#include <string>
#include <sys/types.h>
#include <pthread.h>
#include <fstream>

#include "IcoIo.hpp"
#include "XenoSpiControllerHandler.hpp"

volatile bool exitBool = false;

void help()
{
    printf("This unit test is made for the XenoSpiControllerHandler and IcoIo classes. The test will test if all OPCODE' commands are working. ");
}

int main(int argc, char *argv[])
{
    help();

    XenoSpiControllerHandler spi_controller((uint8_t)1, 12, (uint32_t)8 * MILLION);
    IcoIo ico_io(spi_controller);
    
    // Switch to Xenomai kernel
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

    // Check IcoIo.init()
    ret = ico_io.init();
    if(ret<0)
    {
        evl_printf("Failed to init \n");
        return 0;
    } 
    else
    {
        evl_printf("Suceeded in init \n");
    }

    // Check IcoIo.test()
    ret = ico_io.test();
    if(ret<0)
    {
        evl_printf("Failed to test \n");
        return 0;
    } 
    else
    {
        evl_printf("Suceeded in test \n");
    } 

    // Setup for checking the channels
    IcoIo::IcoWrite w_val;
    IcoIo::IcoRead r_val;
    IcoIo::ReadValue encValue;
    int pwmval[4] = {0,0,0,0};

    pwmval[0] = -2000;
    pwmval[1] = -1000;
    pwmval[2] = 500;
    pwmval[3] = 1500;
    bool pinValue[4] = {true, true, true, true};
    
    uint8_t channel;
    int counter = 1;

    // Test for individual channels
    while(channel > 0 && channel < 5)
    {
        evl_printf("Test %d : PWM & ENC \n", channel);
        evl_printf("PWM_%d \n", channel);
        ico_io.setOutput(channel, pwmval[channel-1],pinValue[channel-1]);
        evl_printf("ENC_%d \n", channel);
        encValue = ico_io.getInput(channel);
        evl_printf("ENC_%i = %5d, pin1 = %d , pin2 = %d\n\n", channel, encValue.enc ,encValue.pin1, encValue.pin2);
        
        counter++;
        
        // If counter reaches 1000 go to next channel
        if(counter == 1000)
        {
            channel++;
            counter = 0;
        }

        // Sleep for 5ms
        evl_usleep(5000);
    }
    // Test for joint channel call
    while(channel == 5)
    {
        for(int i = 0; i<4; i++)
        {
            w_val.pwm1 = pwmval[i];
            w_val.pwm2 = pwmval[i];
            w_val.pwm3 = pwmval[i];
            w_val.pwm4 = pwmval[i];
            w_val.val1 = !w_val.val1;
            w_val.val2 = !w_val.val2;
            w_val.val3 = !w_val.val3;
            w_val.val4 = !w_val.val4;
            ico_io.update_io(w_val, &r_val);

            evl_printf("                 |  PORT 1  |  PORT 2  |  PORT 3  |  PORT 4  |\n" );
            evl_printf("PWM (output)     |   %5i  |   %5i  |   %5i  |   %5i  |\n",w_val.pwm1, w_val.pwm2, w_val.pwm3, w_val.pwm4);
            evl_printf("Aux inputs (1/2) |   %d / %d  |   %d / %d  |   %d / %d  |   %d / %d  |\n", r_val.channel1_1, r_val.channel1_2, r_val.channel2_1, r_val.channel2_2, r_val.channel3_1, r_val.channel3_2, r_val.channel4_1, r_val.channel4_2);
            evl_printf("Encoders         |   %5d  |   %5d  |   %5d  |   %5d  |\n", r_val.channel1, r_val.channel2, r_val.channel3, r_val.channel4);

            counter++;

            if(counter == 1000)
            {
                channel++;
                counter = 0;
            }
            evl_usleep(5000);
        }
    }
    return 0;
}
