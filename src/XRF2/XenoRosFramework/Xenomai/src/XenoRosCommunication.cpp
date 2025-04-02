#include "XenoRosCommunication.hpp"

/**
 * @brief Construct a new Xeno Ros Communication object
 * 
 * Initializes the communication object with the provided name and buffer sizes.
 * 
 * @param _name The name of the communication object.
 * @param _linux_xeno_buffer_size The buffer size for communication between Linux and Xenomai.
 * @param _xeno_linux_buffer_size The buffer size for communication between Xenomai and Linux.
 */
XenoRosCommunication::XenoRosCommunication(const char* _name, int _linux_xeno_buffer_size, int _xeno_linux_buffer_size):
    name(_name),
    linux_xeno_buffer_size(_linux_xeno_buffer_size),
    xeno_linux_buffer_size(_xeno_linux_buffer_size)
{
    printf("%s: Constructing rampio : %s\n", __FUNCTION__, name);
}

/**
 * @brief Destroy the Xeno Ros Communication object
 * 
 * Cleans up resources, ensuring the communication channel is properly closed.
 */
XenoRosCommunication::~XenoRosCommunication()
{
    // Cleanup of fd
    close();
}

/**
 * @brief Opens the communication channel by creating an Xenomai buffer
 * 
 * This function sets up the communication buffer between Linux and Xenomai using the provided buffer sizes
 * and creates a file descriptor for the communication.
 * 
 * @throws error If the creation of the buffer fails, an error is thrown.
 */
void XenoRosCommunication::open()
{
    fd = evl_create_xbuf(xeno_linux_buffer_size, linux_xeno_buffer_size, EVL_CLONE_PUBLIC | EVL_CLONE_NONBLOCK , name);
    if (fd < 0)
        error(1, errno, "evl_create_xbuf(%s)", name);
    #if DEBUG_X_BUF
        printf("%s x-buffer\n", name);
        printf("File descriptor     :    %d\n", fd);
        printf("Buffer size X->L    :    %d\n", xeno_linux_buffer_size);
        printf("Buffer size L->X    :    %d\n", linux_xeno_buffer_size);
    #endif
}

/**
 * @brief Closes the communication channel
 * 
 * This function closes the communication channel by closing the file descriptor.
 */
void XenoRosCommunication::close()
{
    ::close(fd);
}

/**
 * @brief Reads data from the communication buffer
 * 
 * Reads the specified amount of data from the Xenomai communication buffer into the provided memory space.
 * 
 * @param rx_data Pointer to the buffer where the received data will be stored.
 * @param data_size The size of the data to read.
 * @return int Returns the number of bytes read, or a negative value if an error occurs.
 */
int XenoRosCommunication::read(void *rx_data, int data_size)
{
    return oob_read(fd, rx_data, data_size);
}

/**
 * @brief Writes data to the communication buffer
 * 
 * Writes the specified amount of data to the Xenomai communication buffer from the provided memory space.
 * 
 * @param tx_data Pointer to the buffer containing the data to send.
 * @param data_size The size of the data to write.
 * @return int Returns the number of bytes written, or a negative value if an error occurs.
 */
int XenoRosCommunication::write(const void *tx_data, int data_size)
{
    return oob_write(fd, tx_data, data_size);
}