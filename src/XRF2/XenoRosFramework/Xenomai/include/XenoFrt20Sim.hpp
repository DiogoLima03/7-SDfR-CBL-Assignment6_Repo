#ifndef XENOFRT20SIM_HPP
#define XENOFRT20SIM_HPP

#include "XenoFrtRosIco.hpp"
#include "xxmodel.h"

/**
 * @class XenoFrt20Sim
 * @brief A derived class for interfacing with a 20-simulation model in the FRT (Fault Resilient Timing) framework.
 * 
 * This class inherits from XenoFrtRosIco and automatically takes the cycle time from the 20-sim generated code 
 * to handle simulation data logging, monitoring, and real-time operations. 
 * It integrates simulation control with logging and monitoring through the use of a 
 * custom file handler and data pointer, ensuring high performance in real-time systems.
 */
class XenoFrt20Sim : public XenoFrtRosIco
{
public:
    XenoFrt20Sim(uint write_decimator_freq, uint monitor_freq, XenoFileHandler& _log_file_handler, void *data_to_be_logged_pntr);
    ~XenoFrt20Sim();
    
protected:

};

#endif // XENOFRT20SIM_HPP