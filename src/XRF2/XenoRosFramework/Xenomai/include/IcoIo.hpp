#ifndef ICOIO_HPP
#define ICOIO_HPP

#include <unistd.h>
#include <stdint.h>
#include <cstdio>

#include <sys/types.h>
#include <sys/sysinfo.h>
#include <sys/mman.h>
#include <sys/ioctl.h>

#include <error.h>
#include <errno.h>
#include <fcntl.h>
#include <string.h>
#include <time.h>

#include "ICommunicationFullDuplex.h"

#define FRAME_SIZE 12
#define MILLION 1000000L
#define BILLION 1000000000L

#if !ENABLE_OUTPUT_LIMIT
#define MAX_PWM_VALUE 2047
#endif

#if ENABLE_OUTPUT_LIMIT
#define MAX_PWM_VALUE (int)(2047 * MAX_PWM_OUTPUT)
#endif

/**
 * @class IcoIo
 * @brief Class for managing I/O operations using SPI communication.
 */
class IcoIo
{
public:
    /**
     * @brief Constructor for IcoIo.
     * @param _communicator Reference to a full-duplex communication interface.
     */
    IcoIo(ICommunicationFullDuplex& _communicator);//, int _max_pwm_output);

    /**
     * @brief Destructor for IcoIo.
     */
    ~IcoIo();

    /**
     * @struct IcoWrite
     * @brief Structure for output values.
     */
    struct IcoWrite
    {
        int16_t pwm1;        ///< PWM value for channel 1
        bool val1;           ///< Digital output 1 of channel 1
        int16_t pwm2;        ///< PWM value for channel 2
        bool val2;           ///< Digital output 1 of channel 2
        int16_t pwm3;        ///< PWM value for channel 3
        bool val3;           ///< Digital output 1 of channel 3
        int16_t pwm4;        ///< PWM value for channel 4
        bool val4;           ///< Digital output 1 of channel 4
    };

    /**
     * @struct IcoRead
     * @brief Structure for input values.
     */
    struct IcoRead
    {
        int channel1;        ///< Encoder value from channel 1
        int channel2;        ///< Encoder value from channel 2
        int channel3;        ///< Encoder value from channel 3
        int channel4;        ///< Encoder value from channel 4
        bool channel1_1;     ///< Digital input 1 from channel 1
        bool channel1_2;     ///< Digital input 2 from channel 1
        bool channel2_1;     ///< Digital input 1 from channel 2
        bool channel2_2;     ///< Digital input 2 from channel 2
        bool channel3_1;     ///< Digital input 1 from channel 3
        bool channel3_2;     ///< Digital input 2 from channel 3
        bool channel4_1;     ///< Digital input 1 from channel 4
        bool channel4_2;     ///< Digital input 2 from channel 4
    };

    /**
     * @struct ReadValue
     * @brief Encoded read value structure.
     */
    struct ReadValue
    {
        unsigned int enc : 14; ///< 14-bit encoder value.
        unsigned int pin1 : 1; ///< Digital input pin 1.
        unsigned int pin2 : 1; ///< Digital input pin 2.
    };

    // Functions
    void setOutput(uint8_t channel, int16_t value, bool pin_val);
    ReadValue getInput(uint8_t channel);
    size_t update_io(IcoWrite write_val, IcoRead *read_val);
    int spi_init();
    int init();
    int reset();
    int test();

private:
    ICommunicationFullDuplex &communicator; ///< Reference to the communication interface.
    int max_pwm_output; ///< Maximum PWM output value.

    /**
     * @enum OP_CODE
     * @brief Enumeration for SPI command codes.
     */
    enum OP_CODE
    {
        NOOP = 0,  ///< No operation.
        INIT = 1,  ///< Initialization command.
        RESET = 2, ///< Reset command.
        ALL = 3,   ///< Apply all outputs command.
        PWM_1 = 4, ///< PWM command for channel 1.
        PWM_2 = 5, ///< PWM command for channel 2.
        PWM_3 = 6, ///< PWM command for channel 3.
        PWM_4 = 7, ///< PWM command for channel 4.
        ENC_1 = 8, ///< Encoder read for channel 1.
        ENC_2 = 9, ///< Encoder read for channel 2.
        ENC_3 = 10,///< Encoder read for channel 3.
        ENC_4 = 11,///< Encoder read for channel 4.
        TEST = 12  ///< Test command.
    };

    /**
     * @struct WriteValue
     * @brief Structure for encoded PWM and digital output values.
     */
    struct WriteValue
    {
        unsigned int pwm_val : 14; ///< 14-bit PWM value.
        unsigned int pwm_dir : 1; ///< PWM direction.
        unsigned int pin_val : 1; ///< Digital output value.
    };

    /**
     * @struct WriteArray
     * @brief Structure for multiple PWM and digital output values.
     */
    struct WriteArray
    {
        unsigned int pwm_val1 : 14;
        unsigned int pwm_dir1 : 1;
        unsigned int pin_val1 : 1;
        unsigned int pwm_val2 : 14;
        unsigned int pwm_dir2 : 1;
        unsigned int pin_val2 : 1;
        unsigned int pwm_val3 : 14;
        unsigned int pwm_dir3 : 1;
        unsigned int pin_val3 : 1;
        unsigned int pwm_val4 : 14;
        unsigned int pwm_dir4 : 1;
        unsigned int pin_val4 : 1;
    };

    union WriteBytes
    {
        WriteValue n; ///< Encoded write value.
        uint8_t s[2]; ///< Byte representation.
    };

    union WriteArrayBytes
    {
        WriteArray in; ///< Encoded write array.
        uint8_t out[6]; ///< Byte representation.
    };

    union ReadBytes
    {
        ReadValue v; ///< Encoded read value.
        uint8_t s[2]; ///< Byte representation.
    };

    // Variables
    WriteValue write_value_;
    WriteBytes write_bytes_;
    ReadValue read_value_;
    ReadBytes read_bytes_;

    int max_pwm_value; ///< Maximum allowed PWM value.
    int ret = 0; ///< Return value.

    const uint8_t rd_msg[4]; ///< Read message buffer.
    int device_handle_; ///< Device handle identifier.

    uint8_t tx_data[FRAME_SIZE]; ///< Transmission data buffer.
    uint8_t rx_data[FRAME_SIZE]; ///< Reception data buffer.

    // functions
    int create_pwm_msg(uint8_t channel, int16_t value, bool pin_val);
    int create_all_msg(IcoWrite data);
    WriteValue calc_value(int pwm_val, bool pin_val);
    WriteBytes create_pwm_bytes(int16_t pwm_val, bool pin_val);
};

 #endif //ICOIO_HPP
