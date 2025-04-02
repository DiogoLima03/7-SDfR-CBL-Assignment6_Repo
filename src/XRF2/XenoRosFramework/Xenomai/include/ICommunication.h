#ifndef ICOMMUNICATION_H
#define ICOMMUNICATION_H

#include <unistd.h>
#include <stdint.h>
#include <cstdio>
#include <string.h>
#include <error.h>
#include <fcntl.h>

/**
 * @brief Abstract base class for communication interfaces.
 * 
 * This class defines the basic interface for communication protocols, such as reading 
 * and writing data through different types of communication channels (e.g., serial, SPI, etc.).
 * Derived classes must implement the specific communication details for opening, closing, 
 * reading, and writing data through a communication channel.
 */
class ICommunication
{
public:
    /**
     * @brief Virtual destructor for ICommunication class.
     * 
     * Ensures that derived classes can be destroyed properly.
     */
    virtual ~ICommunication(){};
    
    /**
     * @brief Opens the communication channel.
     * 
     * This method should initialize the communication channel (e.g., opening a file descriptor, 
     * setting up the connection) as per the derived class implementation.
     */
    virtual void open() = 0;
    
    /**
     * @brief Closes the communication channel.
     * 
     * This method should close the communication channel, freeing any resources or handles 
     * acquired during the `open` method.
     */
    virtual void close() = 0;
    
    /**
     * @brief Reads data from the communication channel.
     * 
     * @param rx_data Pointer to the buffer where received data will be stored.
     * @param data_size The size of the buffer to read into.
     * 
     * @return The number of bytes successfully read, or a negative value if an error occurs.
     */
    virtual int read(void *rx_data, int data_size) = 0;
    
    /**
     * @brief Writes data to the communication channel.
     * 
     * @param tx_data Pointer to the data to be transmitted.
     * @param data_size The size of the data to write.
     * 
     * @return The number of bytes successfully written, or a negative value if an error occurs.
     */
    virtual int write(const void *tx_data, int data_size) = 0;

protected:
    int fd; /**< File descriptor for the communication channel. */
};

#endif // ICOMMUNICATION_H
