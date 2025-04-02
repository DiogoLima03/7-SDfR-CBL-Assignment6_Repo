#ifndef ICOMMUNICATIONFULLDUPLEX_H
#define ICOMMUNICATIONFULLDUPLEX_H

#include "ICommunication.h" 

/**
 * @brief Abstract class for full-duplex communication interfaces.
 * 
 * This class extends the `ICommunication` interface to provide support for full-duplex 
 * communication, where data can be transmitted and received simultaneously. 
 * Derived classes must implement the specific communication details for transferring 
 * data in both directions.
 */
class ICommunicationFullDuplex : public ICommunication
{
public:
    
    /**
     * @brief Virtual destructor for ICommunicationFullDuplex class.
     * 
     * Ensures that derived classes can be destroyed properly.
     */
    virtual ~ICommunicationFullDuplex(){};
    
    /**
     * @brief Transfers data between the communication channel.
     * 
     * This method handles the transfer of data, where data is both transmitted and received 
     * simultaneously over a communication channel.
     * 
     * @param tx_data Pointer to the data to be transmitted.
     * @param rx_data Pointer to the buffer where received data will be stored.
     * @param data_size The size of the data to transfer.
     * 
     * @return The number of bytes successfully transferred, or a negative value if an error occurs.
     */
    virtual int transfer(const void *tx_data, void *rx_data, int data_size) = 0;
};

#endif //ICOMMUNICATIONFULLDUPLEX_H
