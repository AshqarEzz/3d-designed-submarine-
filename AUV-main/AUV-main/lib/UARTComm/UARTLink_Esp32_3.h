#ifndef UART_LINK_ESP32_3_H
#define UART_LINK_ESP32_3_H

#include <Arduino.h>
#include "UARTComm.h"
#include "DataTypes.h"
#include <functional>
typedef std::function<void(const Esp32_3_Command &)> CommandCallback;

class UARTLink_Esp32_3
{
public:
    UARTLink_Esp32_3(HardwareSerial &serial, uint32_t baud);
    void begin();
    void update();

    // Send the command structure
    void sendCommand(const Esp32_3_Command &cmd);
    void onCommandReceived(CommandCallback cb);

private:
    UARTComm _comm;
    CommandCallback _cmdCallback;

    // If we want an ack, we can handle packets here. For now, we ignore.
    static void handlePacket(const UARTPacket &pkt);
    void onPacket(const UARTPacket &pkt);
    static UARTLink_Esp32_3 *_instance;
};

#endif
