#include "IcoIo.hpp"

/**
 * @brief Constructor for IcoIo class.
 * 
 * Initializes the IcoIo instance and opens the communication channel.
 * 
 * @param _communicator A reference to an object implementing the ICommunicationFullDuplex interface for communication.
 */
IcoIo::IcoIo(ICommunicationFullDuplex& _communicator) : 
    device_handle_{-1}, 
    rd_msg{0, 0, 0, 0},
    communicator(_communicator)
{
    printf("%s: Constructing rampio\n", __FUNCTION__);
    communicator.open();
}

/**
 * @brief Destructor for IcoIo class.
 * 
 * Cleans up resources and closes the communication channel.
 */
IcoIo::~IcoIo()
{
    printf("%s: Destructing rampio\n", __FUNCTION__);
}

/**
 * @brief Initializes the device by sending an initialization command.
 * 
 * Sends an initialization message to the device and checks the response.
 * 
 * @return Returns the frame size on success, or 1 if the initialization fails.
 */
int IcoIo::init()
{
    //msg type
    uint8_t _tx_data[FRAME_SIZE] = {
        OP_CODE::INIT, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00};

    communicator.transfer(_tx_data, rx_data, FRAME_SIZE);
    if (rx_data[4] == 11)
        return FRAME_SIZE;
    else
        error(1, 0, "Failed to initialise FPGA : %d ", rx_data[4]);
        
    return 1;
}

/**
 * @brief Tests the device by sending a test message.
 * 
 * Sends a test message to the device and processes the response.
 * 
 * @return Returns 1 on success.
 */
int IcoIo::test()
{
    uint8_t _tx_data[FRAME_SIZE] = {
        OP_CODE::TEST, 0x00, 0x01, 0x02,
        0x03, 0x04, 0x05, 0x06,
        0x07, 0x08, 0x09, 0x10};

    // transieve data
    communicator.transfer(_tx_data, rx_data, FRAME_SIZE);

    for (int i = 0; i < FRAME_SIZE; i++)
    {
        // evl_printf("%d", rx_data[i]);
        error(1, 0, "This has to be fixed");
    }

    return 1;
}

/**
 * @brief Resets the device by sending a reset message.
 * 
 * Sends a reset message to the device.
 * 
 * @return Returns the frame size.
 */
int IcoIo::reset()
{
    uint8_t _tx_data[FRAME_SIZE] = {
        OP_CODE::RESET, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00};

    // transieve data
    communicator.write(_tx_data, FRAME_SIZE);

    return FRAME_SIZE;
}

/**
 * @brief Sets the output value for a given channel.
 * 
 * Creates a PWM message and sends it to set the output value for the specified channel.
 * 
 * @param channel The channel to set the output for.
 * @param value The PWM value to set.
 * @param pin_val The pin value to set.
 */
void IcoIo::setOutput(uint8_t channel, int16_t value, bool pin_val)
{
    create_pwm_msg(channel, value, pin_val);
    // transieve data
    communicator.write(tx_data, FRAME_SIZE);
}

/**
 * @brief Reads input values from a given channel.
 * 
 * Sends a request to get the encoder and digital input values for the specified channel.
 * 
 * @param channel The channel to read input from.
 * @return The read values, encapsulated in a ReadValue structure.
 */
IcoIo::ReadValue IcoIo::getInput(uint8_t channel)
{
    uint8_t opcode = OP_CODE::ENC_1 + channel - 1;

    uint8_t _tx_data[FRAME_SIZE] = {opcode, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

    // transieve data
    communicator.transfer(_tx_data, rx_data, FRAME_SIZE);

    ReadBytes read_value;
    read_value.s[0] = static_cast<char>(rx_data[1]);
    read_value.s[1] = static_cast<char>(rx_data[2]);

    return read_value.v;
}

/**
 * @brief Updates the input/output values by sending a write request and receiving the response.
 * 
 * Sends data to the device and retrieves the updated values for all channels.
 * 
 * @param write_val The data to write to the device.
 * @param read_val A pointer to the structure to store the read values.
 * @return Returns the frame size.
 */
size_t IcoIo::update_io(IcoWrite write_val, IcoRead *read_val)
{
    int ret = create_all_msg(write_val);

    // transieve data
    communicator.transfer(tx_data, rx_data, FRAME_SIZE);

    ReadBytes enc_bytes1;
    ReadBytes enc_bytes2;
    ReadBytes enc_bytes3;
    ReadBytes enc_bytes4;

    enc_bytes1.s[0] = static_cast<char>(rx_data[1]);
    enc_bytes1.s[1] = static_cast<char>(rx_data[2]);
    enc_bytes2.s[0] = static_cast<char>(rx_data[3]);
    enc_bytes2.s[1] = static_cast<char>(rx_data[4]);
    enc_bytes3.s[0] = static_cast<char>(rx_data[5]);
    enc_bytes3.s[1] = static_cast<char>(rx_data[6]);
    enc_bytes4.s[0] = static_cast<char>(rx_data[7]);
    enc_bytes4.s[1] = static_cast<char>(rx_data[8]);

    read_val->channel1 = enc_bytes1.v.enc;
    read_val->channel1_1 = enc_bytes1.v.pin1;
    read_val->channel1_2 = enc_bytes1.v.pin2;
    read_val->channel2 = enc_bytes2.v.enc;
    read_val->channel2_1 = enc_bytes2.v.pin1;
    read_val->channel2_2 = enc_bytes2.v.pin2;
    read_val->channel3 = enc_bytes3.v.enc;
    read_val->channel3_1 = enc_bytes3.v.pin1;
    read_val->channel3_2 = enc_bytes3.v.pin2;
    read_val->channel4 = enc_bytes4.v.enc;
    read_val->channel4_1 = enc_bytes4.v.pin1;
    read_val->channel4_2 = enc_bytes4.v.pin2;
    return FRAME_SIZE;
}

/**
 * @brief Creates a PWM message for a given channel, value, and pin state.
 * 
 * Creates a message that represents a PWM value and associated pin state.
 * 
 * @param channel The channel number.
 * @param value The PWM value.
 * @param pin_val The pin state.
 * @return Returns 1 to indicate successful message creation.
 */
int IcoIo::create_pwm_msg(uint8_t channel, int16_t value, bool pin_val)
{
    WriteValue write_value;
    WriteBytes write_bytes;
    if (value < 0)
    {
        write_value.pwm_dir = 0;
        if (value < -MAX_PWM_VALUE)
        {
            write_value.pwm_val = MAX_PWM_VALUE;
        }
        else
        {
            write_value.pwm_val = -value;
        }
    }
    else
    {
        write_value.pwm_dir = 1;
        if (value > MAX_PWM_VALUE)
        {
            write_value.pwm_val = MAX_PWM_VALUE;
        }
        else
        {
            write_value.pwm_val = value;
        }
    }

    write_value.pin_val = static_cast<unsigned int>(pin_val);

    write_bytes.n = write_value;
    uint8_t opcode = OP_CODE::PWM_1 + channel - 1;
    uint8_t pwm_cmd[FRAME_SIZE] = {
        opcode, 0, write_bytes.s[0], write_bytes.s[1],
        0, 0, 0, 0,
        0, 0, 0, 0};
    memcpy(tx_data, pwm_cmd, FRAME_SIZE);
    return 1;
}

/**
 * @brief Creates a message containing all PWM values for the device.
 * 
 * Constructs a message that sets the PWM values for all four channels.
 * 
 * @param data The data containing PWM values for all channels.
 * @return Returns 1 to indicate successful message creation.
 */
int IcoIo::create_all_msg(IcoWrite data)
{
    WriteArray write_array;

    WriteValue val1 = calc_value(data.pwm1, data.val1);
    WriteValue val2 = calc_value(data.pwm2, data.val2);
    WriteValue val3 = calc_value(data.pwm3, data.val3);
    WriteValue val4 = calc_value(data.pwm4, data.val4);

    write_array.pwm_val1 = val1.pwm_val;
    write_array.pwm_dir1 = val1.pwm_dir;
    write_array.pin_val1 = val1.pin_val;
    write_array.pwm_val2 = val2.pwm_val;
    write_array.pwm_dir2 = val2.pwm_dir;
    write_array.pin_val2 = val2.pin_val;
    write_array.pwm_val3 = val3.pwm_val;
    write_array.pwm_dir3 = val3.pwm_dir;
    write_array.pin_val3 = val3.pin_val;
    write_array.pwm_val4 = val4.pwm_val;
    write_array.pwm_dir4 = val4.pwm_dir;
    write_array.pin_val4 = val4.pin_val;

    WriteArrayBytes write_bytes;
    write_bytes.in = write_array;
    uint8_t pwm_cmd[FRAME_SIZE] = {
        OP_CODE::ALL, 0,
        write_bytes.out[0], write_bytes.out[1],
        write_bytes.out[2], write_bytes.out[3],
        write_bytes.out[4], write_bytes.out[5],
        write_bytes.out[6], write_bytes.out[7],
        0, 0};
    memcpy(tx_data, pwm_cmd, FRAME_SIZE);

    return 1;
}

/**
 * @brief Creates PWM bytes representing the specified value and pin state.
 * 
 * Generates a struct containing the PWM value and pin state.
 * 
 * @param pwm_val The PWM value.
 * @param pin_val The pin state.
 * @return A WriteBytes structure containing the PWM data.
 */
IcoIo::WriteBytes IcoIo::create_pwm_bytes(int16_t pwm_val, bool pin_val)
{
    WriteValue write_value;
    WriteBytes write_bytes;
    if (pwm_val < 0)
    {
        write_value.pwm_dir = 0;
        if (pwm_val < -MAX_PWM_VALUE)
        {
            write_value.pwm_val = MAX_PWM_VALUE;
        }
        else
        {
            write_value.pwm_val = -pwm_val;
        }
    }
    else
    {
        write_value.pwm_dir = 1;
        if (pwm_val > MAX_PWM_VALUE)
        {
            write_value.pwm_val = MAX_PWM_VALUE;
        }
        else
        {
            write_value.pwm_val = pwm_val;
        }
    }

    write_value.pin_val = static_cast<unsigned int>(pin_val);
    write_bytes.n = write_value;
    return write_bytes;
}

/**
 * @brief Calculates the PWM value and direction based on input.
 * 
 * Determines the PWM value and direction for a specified input value.
 * 
 * @param pwm_val The PWM value.
 * @param pin_val The pin state.
 * @return A WriteValue structure containing the calculated PWM value and direction.
 */
IcoIo::WriteValue IcoIo::calc_value(int pwm_val, bool pin_val)
{
    WriteValue write_value;
    if (pwm_val < 0)
    {
        write_value.pwm_dir = 0;
        if (pwm_val < -MAX_PWM_VALUE)
        {
            write_value.pwm_val = MAX_PWM_VALUE;
        }
        else
        {
            write_value.pwm_val = -pwm_val;
        }
    }
    else
    {
        write_value.pwm_dir = 1;
        if (pwm_val > MAX_PWM_VALUE)
        {
            write_value.pwm_val = MAX_PWM_VALUE;
        }
        else
        {
            write_value.pwm_val = pwm_val;
        }
    }
    write_value.pin_val = static_cast<unsigned int>(pin_val);
    return write_value;
}
