// UARTLink_Esp32_3.cpp
#include "UARTLink_Esp32_3.h"

UARTLink_Esp32_3 *UARTLink_Esp32_3::_instance = nullptr;

UARTLink_Esp32_3::UARTLink_Esp32_3(HardwareSerial &serial, uint32_t baud)
    : _comm(serial, baud), _cmdCallback(nullptr)
{
    _instance = this;
}

void UARTLink_Esp32_3::begin()
{
    _comm.begin();
    _comm.onPacketReceived(handlePacket);
}

void UARTLink_Esp32_3::update()
{
    _comm.update();
}

void UARTLink_Esp32_3::onCommandReceived(CommandCallback cb)
{
    _cmdCallback = cb;
}

void UARTLink_Esp32_3::handlePacket(const UARTPacket &pkt)
{
    if (_instance)
    {
        _instance->onPacket(pkt);
    }
}

void UARTLink_Esp32_3::onPacket(const UARTPacket &pkt)
{
    if (pkt.msgType == UARTMsgType::COMMAND_ESC_PUMP)
    {
        if (pkt.length == sizeof(Esp32_3_Command))
        {
            Esp32_3_Command cmd;
            memcpy(&cmd, pkt.payload, sizeof(Esp32_3_Command));
            if (_cmdCallback)
            {
                _cmdCallback(cmd);
            }
        }
    }
    // else ignore
}
