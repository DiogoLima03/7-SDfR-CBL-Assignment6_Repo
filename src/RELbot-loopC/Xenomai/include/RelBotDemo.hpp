#ifndef RELBOTDEMO_HPP
#define RELBOTDEMO_HPP

#include "XenoFrt20Sim.hpp"
#include "LoopController.h"

#pragma pack (1)    //https://carlosvin.github.io/langs/en/posts/cpp-pragma-pack/#_performance_test
struct ThisIsAStruct
{
    int this_is_a_int = 0;
    double this_is_a_double = 100.0;
    float this_is_a_float = 10.0;
    char this_is_a_char = 'R';
    bool this_is_a_bool = false;
};
#pragma pack(0)

class RelBotDemo : public XenoFrt20Sim
{
public:
    RelBotDemo(uint write_decimator_freq, uint monitor_freq);
    ~RelBotDemo();
private:
    XenoFileHandler file;
    struct ThisIsAStruct data_to_be_logged;
    LoopController controller;

    double u[4];
    double y[4];
   

    int32_t last_channel2 = 0; 
    int32_t last_channel1 = 0;
    float pos_left = 0; 
    float pos_right = 0; 
protected:
    //Functions
    int initialising() override;
    int initialised() override;
    int run() override;
    int stopping() override;
    int stopped() override;
    int error() override;

    // current error
    int current_error = 0;
};

#endif // RELBOTDEMO_HPP