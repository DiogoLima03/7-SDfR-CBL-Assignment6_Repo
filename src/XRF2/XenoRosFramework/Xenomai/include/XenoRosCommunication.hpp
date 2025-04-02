#ifndef XENOROSCOMMUNICATION_HPP
#define XENOROSCOMMUNICATION_HPP

#include <evl/evl.h>

#include "ICommunication.h"
#include "debug_settings.h"

/**
 * @class XenoRosCommunication
 * @brief Implementation of communication interface for ROS with Xenomai.
 * 
 * This class handles communication between the ROS environment and Xenomai using a shared buffer.
 * It implements the ICommunication interface to provide methods for opening, closing, reading, and writing data.
 */
class XenoRosCommunication : public ICommunication
{
public:
    XenoRosCommunication(const char* _name, int _linux_xeno_buffer_size, int _xeno_linux_buffer_size);
    ~XenoRosCommunication();

    void open() override;
    void close() override;
    int read(void *rx_data, int data_size) override;
    int write(const void *tx_data, int data_size) override;

private:
    /* data */
    const char* name;           ///< The name of the communication object
    int linux_xeno_buffer_size; ///< Buffer size for Linux to Xenomai communication
    int xeno_linux_buffer_size; ///< Buffer size for Xenomai to Linux communication
};

#endif // XENOROSCOMMUNICATION_HPP