#include "UARTLink_Esp32_2.h"

UARTLink_Esp32_2 *UARTLink_Esp32_2::_instance = nullptr;

UARTLink_Esp32_2::UARTLink_Esp32_2(HardwareSerial &serial, uint32_t baud)
    : _commBase(serial, baud), _newData(false)
{
    _instance = this;
}

void UARTLink_Esp32_2::begin()
{
    _commBase.begin();
    _commBase.onPacketReceived(handlePacket);
}

void UARTLink_Esp32_2::update()
{
    _commBase.update();
}

bool UARTLink_Esp32_2::getData(Esp32_2_Data &data)
{
    if (_newData)
    {
        data = _lastData;
        _newData = false;
        return true;
    }
    return false;
}

void UARTLink_Esp32_2::handlePacket(const UARTPacket &pkt)
{
    if (_instance)
    {
        _instance->onPacket(pkt);
    }
}

void UARTLink_Esp32_2::onPacket(const UARTPacket &pkt)
{
    if (pkt.msgType == UARTMsgType::DOPPLER_PRESS_FLOW)
    {
        if (pkt.length == sizeof(Esp32_2_Data))
        {
            memcpy(&_lastData, pkt.payload, sizeof(Esp32_2_Data));
            _newData = true;
        }
    }
}
