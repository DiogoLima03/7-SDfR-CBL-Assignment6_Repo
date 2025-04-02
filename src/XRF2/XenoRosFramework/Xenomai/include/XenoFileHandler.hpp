#ifndef XENOFRTFILEHANDLER_HPP
#define XENOFRTFILEHANDLER_HPP

#include <unistd.h>
#include <stdint.h>
#include <cstdio>
#include <string>
#include <error.h>
#include <time.h> 
#include <sys/stat.h>

#include "ICommunication.h"
#include "debug_settings.h"

#include <evl/evl.h>
#include <evl/proxy.h>

#define MSEC        1000000L   ///< Number of nanoseconds in one millisecond.
#define SEC         1000000000L ///< Number of nanoseconds in one second.

/**
 * @brief Class for file handling with support for communication over files.
 * 
 * This class provides the functionality to open, read from, and write to a file, 
 * as well as manage file access permissions. It inherits from `ICommunication` to 
 * provide standard communication interface methods for file-based communication.
 */
class XenoFileHandler : public ICommunication
{
public:
    XenoFileHandler(bool add_timestamp, const char *name, const char *_file_type, mode_t mode = 00666);
    ~XenoFileHandler();
    void open() override;
    void close() override;
    int read(void *rx_data, int data_size) override;
    int write(const void *tx_data, int data_size) override;

private:
    char *file_name;       ///< The name of the file to interact with.
    char *proxy_name;      ///< The proxy name for file interaction.
    mode_t mode;           ///< The file access mode (permissions).
    int file_fd;           ///< File descriptor for the opened file.
};

#endif // XENOFRTFILEHANDLER_HPP