#ifndef TEMPLATE_HPP
#define TEMPLATE_HPP

#include "XenoFrtRosIco.hpp"

#pragma pack (1)    //https://carlosvin.github.io/langs/en/posts/cpp-pragma-pack/#_performance_test
struct LoggedVariable
{
    int this_is_a_int = 0;
    double this_is_a_double = 100.0;
    float this_is_a_float = 10.0;
    char this_is_a_char = 'R';
    bool this_is_a_bool = false;
};

#pragma pack(0)

class Template : public XenoFrtRosIco
{
public:
    Template(uint cycle_time_freq, uint write_decimator_freq, uint monitor_freq);
    ~Template();
private:
    XenoFileHandler file;
    struct LoggedVariable data_to_be_logged;

protected:
    //Functions
    int initialising() override;
    int initialised() override;
    int run() override;
    int stopping() override;
    int stopped() override;
    int pausing() override;
    int paused() override;
    int error() override;

    // current error
    int current_error = 0;
};

#endif // TEMPLATE_HPP