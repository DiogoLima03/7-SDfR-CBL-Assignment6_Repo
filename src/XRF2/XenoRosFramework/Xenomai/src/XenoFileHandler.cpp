#include "XenoFileHandler.hpp"

/**
 * @brief Construct a new XenoFileHandler object.
 * 
 * Initializes a file handler with a given name, file type, and optional timestamp.
 * 
 * @param add_timestamp Flag to determine whether to add a timestamp to the file name.
 * @param _file_name The base name of the file.
 * @param _file_type The file type extension.
 * @param _mode File permissions (default: 0666).
 */
XenoFileHandler::XenoFileHandler(bool add_timestamp, const char *_file_name, const char *_file_type, mode_t _mode) 
{
    printf("%s: Constructing rampio: %s\n", __FUNCTION__, _file_name);
    // If add_timestamp is true add data and time to filename if not do not add it
    if(add_timestamp)
    {
        struct timespec t_current_time;
        evl_read_clock(EVL_CLOCK_REALTIME, &t_current_time);                                // Get current time
        struct tm *my_tm = localtime(&t_current_time.tv_sec);                               // Go from struct timespec to struct tm
        char timestamp[20];
        strftime(timestamp, sizeof (timestamp),"_%y%m%d_%H%M%S",my_tm);                     // Translate time from struct to char  

        int file_name_len = strlen(_file_name) + 20 + strlen(_file_type) + 1;               // Calculate name lenght
        int proxy_name_len = file_name_len + 6;                                             // Calculate proxy name lenght

        file_name = (char*)malloc(file_name_len);                                           // Reserve space for name
        proxy_name = (char*)malloc(proxy_name_len);                                         // Reserve space for proxy name

        snprintf(file_name, file_name_len, "%s%s.%s", _file_name, timestamp, _file_type);   // Fill in the placeholders to create name  
        snprintf(proxy_name, proxy_name_len, "proxy_%s", file_name);                        // Fill in the placeholders to create proxy name                        
    }
    else
    {
        int file_name_len = strlen(_file_name) + strlen(_file_type) + 1;                    // Calculate name lenght
        int proxy_name_len = file_name_len + 6;                                             // Calculate proxy name lenght

        file_name = (char*)malloc(file_name_len);                                           // Reserve space for name
        proxy_name = (char*)malloc(proxy_name_len);                                         // Reserve space for proxy name

        snprintf(file_name, file_name_len, "%s.%s", _file_name, _file_type);                // Combine name and file type
        snprintf(proxy_name, proxy_name_len, "proxy_%s", file_name);                        // Fill in the placeholders to create proxy name   
    }
        

    mode = _mode;
    evl_printf("Created a file with the name : %s , permissions : %d \n", file_name, mode);
}

/**
 * @brief Destroy the XenoFileHandler object.
 * 
 * Closes the file associated with the handler upon destruction.
 */
XenoFileHandler::~XenoFileHandler()
{
    printf("%s: Destructing rampio\n", __FUNCTION__);
    // Close the file that has been used
    close();
}

/**
 * @brief Open the file for reading and writing.
 * 
 * Opens the file with the specified file name and permissions. The file is opened for reading, writing, 
 * and appending. Also creates a proxy for the file if using Xenomai.
 */
void XenoFileHandler::open()
{
    // Do not open file if file already open
    if(fd>0)
        return;
    // Open file
    file_fd = ::open(file_name, O_CREAT | O_APPEND | O_RDWR , mode);
    if(file_fd <= 0)
        error(1, errno, "XenoFileHandler.open(%s), fd : %d\n", file_name, file_fd);
    
    fd = evl_new_proxy(file_fd, 1024*1024, proxy_name);
    if(file_fd <= 0)
        error(1, errno, "evl_new_proxy(%s), fd : %d\n", proxy_name, fd);

    #if FILE_HANDLER_DEBUG
    evl_printf("FileHandler opened file: %s, fd : %d, proxy fd: %d \n", file_name, file_fd, fd);
    #endif
}

/**
 * @brief Close the file.
 * 
 * Closes the file associated with this handler.
*/
void XenoFileHandler::close()
{
    // Close file
    ::close(fd);
}

/**
 * @brief Read data from the file.
 * 
 * Reads data from the file into the provided buffer. It determines whether to use Xenomai's OOB read 
 * or the regular Linux read based on the current kernel.
 * 
 * @param rx_data Pointer to the buffer where the data will be stored.
 * @param data_size The size of the data to be read.
 * @return The number of bytes read on success, or -1 on failure.
*/
int XenoFileHandler::read(void *rx_data, int data_size)
{
    int ret;
    // Read data from file
    if(!evl_is_inband())
    {
        // If current kernel is Xenomai
        ret = oob_read(fd, rx_data, data_size);
    }
    else
    {
        // If current kernel is linux
        ret = ::read(file_fd, rx_data, data_size);
    }
    
    return ret;
}

/**
 * @brief Write data to the file.
 * 
 * Writes data to the file. If using Xenomai, it writes using OOB write; otherwise, it uses regular Linux write.
 * 
 * @param tx_data Pointer to the data to be written.
 * @param data_size The size of the data to be written.
 * @return The number of bytes written on success, or -1 on failure.
*/
int XenoFileHandler::write(const void *tx_data, int data_size)
{
    int ret;
    // Write data to file
     if(!evl_is_inband())
    {
        // If current kernel is Xenomai
        ret = oob_write(fd, tx_data, data_size);
    }
    else
    {
        // If current kernel is linux
        ret = ::write(file_fd, tx_data, data_size);
    }
    
    if(ret<0)
        error(0,errno,"oob_write(%s)", file_name);


    return ret;
}