#ifndef UART_COMM_H
#define UART_COMM_H

#include <Arduino.h>

// We define message types for the entire system:
enum class UARTMsgType : uint8_t
{
    // Main <- ESP32-1
    IMU_DATA = 0x01,
    SONAR_SWEEP_DATA = 0x02,

    // Main <- ESP32-2
    DOPPLER_PRESS_FLOW = 0x03,

    // Main -> ESP32-3
    COMMAND_ESC_PUMP = 0x04,

    // Optionally define other message types (servo config, ack, etc.)
    // ...
};

// A struct to hold a "received packet" breakdown
struct UARTPacket
{
    UARTMsgType msgType;
    uint16_t length;
    uint8_t *payload; // Dynamically allocated or pointer to a buffer
};

// A callback type for "when a valid packet is received":
typedef void (*PacketHandlerCallback)(const UARTPacket &packet);

class UARTComm
{
public:
    UARTComm(HardwareSerial &serial, uint32_t baudRate);

    // Should be called in setup()
    void begin();

    // Call frequently in loop(), non-blocking
    void update();

    // Send a packet
    bool sendPacket(UARTMsgType msgType, const uint8_t *payload, uint16_t length);

    // Register a callback that will be called upon valid packet receipt
    void onPacketReceived(PacketHandlerCallback cb);

private:
    HardwareSerial &_serial;
    uint32_t _baudRate;

    PacketHandlerCallback _packetHandler;

    // Parser state machine
    enum class ParserState
    {
        WAIT_START,
        WAIT_MSGTYPE,
        WAIT_LENGTH_1,
        WAIT_LENGTH_2,
        WAIT_PAYLOAD,
        WAIT_CRC_1,
        WAIT_CRC_2,
        WAIT_END
    };
    ParserState _state;

    static const uint8_t START_BYTE = 0xAA;
    static const uint8_t END_BYTE = 0x55;

    // We'll store partial data
    UARTMsgType _currentMsgType;
    uint16_t _currentLength;
    uint16_t _bytesReadForPayload;
    uint16_t _currentCRC;
    uint8_t *_payloadBuf;
    uint16_t _payloadBufSize;

    // We store the data that we feed into CRC => (msgType + length + payload).
    // We'll accumulate it in a small temp buffer to compute the final CRC.
    uint8_t *_crcBuffer;
    uint16_t _crcBufferPos;

    // private methods
    void resetParser();
    void handleByte(uint8_t b);
    void expandPayloadBuffer(uint16_t neededSize);
    void computeCRCByte(uint8_t b);
};

#endif
