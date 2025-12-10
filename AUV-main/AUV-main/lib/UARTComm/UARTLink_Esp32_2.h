#ifndef UART_LINK_ESP32_2_H
#define UART_LINK_ESP32_2_H

#include <Arduino.h>
#include "UARTComm.h"
#include "DataTypes.h"

class UARTLink_Esp32_2
{
public:
    UARTLink_Esp32_2(HardwareSerial &serial, uint32_t baud);
    void begin();
    void update();

    bool getData(Esp32_2_Data &data);
    void sendData(const Esp32_2_Data &data)
    {
        // direct approach:
        _comm().sendPacket(UARTMsgType::DOPPLER_PRESS_FLOW,
                           (uint8_t *)&data,
                           sizeof(Esp32_2_Data));
    }

    // also let's add an accessor to _comm if we want:
    UARTComm &_comm() { return _commBase; }

private:
    UARTComm _commBase;
    Esp32_2_Data _lastData;
    bool _newData;

    static void handlePacket(const UARTPacket &pkt);
    void onPacket(const UARTPacket &pkt);
    static UARTLink_Esp32_2 *_instance;
};

#endif
