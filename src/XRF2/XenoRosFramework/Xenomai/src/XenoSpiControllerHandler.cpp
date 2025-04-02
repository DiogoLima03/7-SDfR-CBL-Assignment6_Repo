#include "XenoSpiControllerHandler.hpp"

/**
 * @brief Constructs a XenoSpiControllerHandler object with specified configurations.
 * 
 * @param _lsb Specifies the LSB/MSB order. (0 = MSB first, 1 = LSB first)
 * @param data_lenght Length of the data buffer for SPI communication.
 * @param _speed SPI communication speed in Hz.
 */
XenoSpiControllerHandler::XenoSpiControllerHandler(uint8_t _lsb = 0, int data_lenght = 12, uint32_t _speed =  8 * MILLION) 
{
    printf("%s: Constructing rampio\n", __FUNCTION__);
    lsb = _lsb;
    len = data_lenght;
    speed = _speed;
}


/**
 * @brief Destructor for XenoSpiControllerHandler. Ensures the SPI device is properly closed.
 */
XenoSpiControllerHandler::~XenoSpiControllerHandler()
{
    printf("%s: Destructing rampio\n", __FUNCTION__);
    // Close the fd
    close();
}


/**
 * @brief Opens and configures the SPI device for communication.
 */
void XenoSpiControllerHandler::open()
{
    // Open the SPI device
    char device_name[20];

#if DEBUG_SPI
    evl_printf("SPI: Start with opening device");
#endif

    sprintf(device_name, "/dev/spidev%d.0", 0);
    fd = ::open(device_name, O_RDWR);
    if (fd < 0)
        error(1, errno, "can't open device %p", &device);

        /*This does not work
        ret = ioctl(fd, SPI_IOC_WR_LSB_FIRST, &lsb_first);
        if (ret)
            error(1, errno, "ioctl(SPI_IOC_WR_LSB_FIRST)");
        */

#if DEBUG_SPI
    evl_printf("SPI: setting up mode\n");
#endif

    // Setting SPI mode
    int ret = ioctl(fd, SPI_IOC_WR_MODE, &mode);
    if (ret)
        error(1, errno, "ioctl(SPI_IOC_WR_MODE32)");

#if DEBUG_SPI
    evl_printf("SPI: Setting up bit per word: %d\n", bits);
#endif

    ret = ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bits);
    if (ret)
        error(1, errno, "ioctl(SPI_IOC_WR_BITS_PER_WORD)");

#if DEBUG_SPI
    evl_printf("SPI: Setting up max speed: %d\n", speed);
#endif

    ret = ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed);
    if (ret)
        error(1, errno, "ioctl(SPI_IOC_WR_MAX_SPEED_HZ)");

#if DEBUG_SPI
    evl_printf("SPI: Switching to OOB operation\n");
#endif

    /* Switch to Out-of-Band operation */
    device.oob_setup.frame_len = len;
    device.oob_setup.speed_hz = speed;
    device.oob_setup.bits_per_word = bits;

    ret = ioctl(fd,
                SPI_IOC_ENABLE_OOB_MODE, &device.oob_setup);
    if (ret)
        error(1, errno, "ioctl(SPI_IOC_ENABLE_OOB_MODE)");

    /* Map the I/O area */
    device.iobuf = mmap(NULL,
                        device.oob_setup.iobuf_len,
                        PROT_READ | PROT_WRITE,
                        MAP_SHARED, fd, 0);
    if (device.iobuf == MAP_FAILED)
        error(1, errno, "mmap()");

    /* Define TX/RX from I/O Buffer */
    device.tx = (char *)(device.iobuf + device.oob_setup.tx_offset);
    memset(device.tx, 0, len);
    device.rx = (char *)(device.iobuf + device.oob_setup.rx_offset);
    memset(device.rx, 0, len);

    /* Extra information */
    device.len = len;
    
    return;
}



/**
 * @brief Closes the SPI device.
 */
void XenoSpiControllerHandler::close()
{
    ::close(fd);
    return;
}


/**
 * @brief Reads data from the SPI device RX buffer.
 * 
 * Write function has to be called first for something to be read
 * 
 * @param rx_data Pointer to the buffer where received data will be stored.
 * @param data_size Size of the data to be read.
 * @return int Number of bytes read.
 */
int XenoSpiControllerHandler::read(void *rx_data, int data_size)
{
    //Check if data size has the same size as buffer and give warning if smaller and error if bigger
    if(data_size < len)
        evl_printf("SPI controller: Warning data size asked is smaller than data buffer size!/n");
    if(data_size > len)
        error(1,0,"SPI controller: Warning data size asked is bigger than data buffer size!/n");

    // to be removed, temp solution for lsb
    if(lsb == 1)
        reverseOutputBytes(data_size);
        
    memcpy(rx_data, device.rx, data_size);

    return data_size;
}

/**
 * @brief Writes data to the SPI device TX buffer.
 * 
 * @param tx_data Pointer to the data to be transmitted.
 * @param data_size Size of the data to be written.
 * @return int Number of bytes written.
 */
int XenoSpiControllerHandler::write(const void *tx_data, int data_size)
{
    //Check if data size has the same size as buffer and five warnig if smaller and error if bigger
    if(data_size < len)
        evl_printf("SPI controller: Warning data size to be written is smaller than data buffer size!/n");
    if(data_size > len)
        error(1,0,"SPI controller: Warning data size to be written is bigger than data buffer size!/n");

    memset(device.rx, 0, data_size);
    memcpy(device.tx, tx_data, data_size);
#if DEBUG_SPI
    evl_printf("write: ");
    for (int i = 0; i < data_size; i++)
        evl_printf("0x%02x ", device.tx[i]);
    evl_printf("\n");
#endif

    // to be removed, temp solution for lsb
    if(lsb == 1)
        reverseInputBytes(data_size);

    int ret = oob_ioctl(fd, SPI_IOC_RUN_OOB_XFER);
    if (ret)
        error(1, errno, "oob_ioctl(SPI_IOC_RUN_OOB_XFER)");

#if DEBUG_SPI
    evl_printf("read: ");
    for (int i = 0; i < data_size; i++)
        evl_printf("0x%02x ", device.rx[i]);
    evl_printf("\n");
#endif
    return data_size;
}

/**
 * @brief Transfers data to and from the SPI device.
 * 
 * @param tx_data Pointer to the data to be transmitted.
 * @param rx_data Pointer to the buffer where received data will be stored.
 * @param data_size Size of the data to be transferred.
 * @return int Number of bytes transferred.
 */
int XenoSpiControllerHandler::transfer(const void *tx_data, void *rx_data, int data_size)
{
     //Check if data size has the same size as buffer and five warnig if smaller and error if bigger
    if(data_size < len)
        evl_printf("SPI controller: Warning data size to be transfered is smaller than data buffer size!/n");
    if(data_size > len)
        error(1,0,"SPI controller: Warning data size to be transfered is bigger than data buffer size!/n");

    memset(device.rx, 0, data_size);
    memcpy(device.tx, tx_data, data_size);

#if DEBUG_SPI
    evl_printf("write: ");
    for (int i = 0; i < data_size; i++)
        evl_printf("0x%02x ", device.tx[i]);
    evl_printf("\n");
#endif

    // to be removed, temp solution for lsb
    if(lsb == 1)
        reverseInputBytes(data_size);

    int ret = oob_ioctl(fd, SPI_IOC_RUN_OOB_XFER);
    if (ret)
        error(1, errno, "oob_ioctl(SPI_IOC_RUN_OOB_XFER)");

    // to be removed, temp solution for lsb
    if(lsb == 1)
        reverseOutputBytes(data_size);

    memcpy(rx_data, device.rx, data_size);

#if DEBUG_SPI
    evl_printf("read: ");
    for (int i = 0; i < data_size; i++)
        evl_printf("0x%02x ", device.rx[i]);
    evl_printf("\n");
#endif

    return data_size;
}

/**
 * @brief Reverses the bytes in the TX buffer for LSB-first configuration.
 * 
 * This function is a temperally solution to the problem of not beiing able to set the LSB register
 * 
 * @param data_size Size of the data to reverse.
 */
void XenoSpiControllerHandler::reverseInputBytes(int data_size)
{
    for (int x = 0; x < data_size; x++)
    {
        device.tx[x] = reverseTable[device.tx[x]];
    }
    return;
}

/**
 * @brief Reverses the bytes in the RX buffer for LSB-first configuration.
 * 
 * This function is a temperally solution to the problem of not beiing able to set the LSB register
 * 
 * @param data_size Size of the data to reverse.
 */
void XenoSpiControllerHandler::reverseOutputBytes(int data_size)
{
    for (int x = 0; x < data_size; x++)
    {
        device.rx[x] = reverseTable[device.rx[x]];
    }
    return; 
}
