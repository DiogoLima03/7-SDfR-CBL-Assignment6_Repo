#include "RelBotDemo.hpp"

RelBotDemo::RelBotDemo(uint write_decimator_freq, uint monitor_freq) :
    XenoFrt20Sim(write_decimator_freq, monitor_freq, file, &data_to_be_logged),
    file(1,"./xrf2_logging/demo","bin"),
    controller()
{
    printf("%s: Constructing rampio\n", __FUNCTION__);

    // To infinite run the controller, uncomment line below
    controller.SetFinishTime(0.0);
}

RelBotDemo::~RelBotDemo()
{
    
}

int RelBotDemo::initialising()
{
    // The logger has to be initialised at only once
    // logger.initialise();   // ENABLE IF WANTING TO TEST THE LOGGER CREATION
    // The FPGA has to be initialised at least once
    ico_io.init();

    return 1;
}

int RelBotDemo::initialised()
{
    return 1;
}

int RelBotDemo::run()
{
    //float diameter = 0.101;
    float gear_ratio = 15.58;

    int32_t diff_channel2 = sample_data.channel2 - this->last_channel2;
    if (diff_channel2 > 8192) // wrap around forward i.e. from 16383 to 0
        diff_channel2 -= 16384;
    else if (diff_channel2 < -8192) // 
        diff_channel2 += 16384;

    

    last_channel2 = sample_data.channel2;

    // The negative sign is because the encoder of the right motor is inverted, it counts in the opposite direction.
    int32_t diff_channel1 = -(sample_data.channel1 - this->last_channel1); 
    if (diff_channel1 > 8192)
        diff_channel1 -= 16384;
    else if (diff_channel1 < -8192)
        diff_channel1 += 16384;


    last_channel1 = sample_data.channel1;

    pos_left += (diff_channel2 / (1024.0 * gear_ratio * 4)) * (2*3.14159); // 
    pos_right += (diff_channel1 / (1024.0 * gear_ratio * 4)) * (3.14159 * 2);
    monitor.printf("Left motor: %f, Right motor: %f\n",this->pos_left, this->pos_right);

    u[0] = pos_left;
    u[1] = pos_right;

    monitor.printf("ANGULAR Left motor: %f, ANGULAR Right motor: %f\n",this->u[0], this->u[1]);

    u[2] = ros_msg.left_wheel_vel; // Velocity setpoint for the left motor (coming from ROS)
    u[3] = ros_msg.right_wheel_vel;

    controller.Calculate(u, y);

    monitor.printf("OUTPUT Controller: %f, Right Controller: %f\n",y[0], y[1]);

    double output2 = y[0];//-2047*(this->y[0])/100;
    double output1 = y[1];//2047*(this->y[1])/100;

    if (output1 < -2047) {
        output1 = -2047;
    } 
    else if (output1 > 2047) {
        output1 = 2047;
    }

    if (output2 < -2047) {
        output2 = -2047;
    } 
    else if (output2 > 2047) {
        output2 = 2047;
    }

    actuate_data.pwm1 = -output1;
    actuate_data.pwm2 = output2;

    if(controller.IsFinished())
        return 1;


    return 0;
}

int RelBotDemo::stopping()
{
    logger.stop();                                // Stop logger
    return 1;
}

int RelBotDemo::stopped()
{
    // A steady state in which the system can be deactivated whitout harming the physical system

    monitor.printf("Hello from stopping\n");          // Do something

    return 0;
}

int RelBotDemo::error()
{
    // Error detected in the system 
    // Can go to error if the previous state returns 1 from every other state function but initialising 

    monitor.printf("Hello from error\n");             // Do something

    return 0;
}
