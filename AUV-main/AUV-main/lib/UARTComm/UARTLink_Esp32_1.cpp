#include "UARTLink_Esp32_1.h"

UARTLink_Esp32_1 *UARTLink_Esp32_1::_instance = nullptr;

UARTLink_Esp32_1::UARTLink_Esp32_1(HardwareSerial &serial, uint32_t baud)
    : _commBase(serial, baud), _newImu(false), _newSonar(false)
{
    // store instance pointer so static callback can refer to us
    _instance = this;
}

void UARTLink_Esp32_1::begin()
{
    _commBase.begin();
    _commBase.onPacketReceived(handlePacket);
}
void UARTLink_Esp32_1::update()
{
    _commBase.update();
}

bool UARTLink_Esp32_1::getImuData(IMU_MPU6050_Data &imuData)
{
    if (_newImu)
    {
        imuData = _lastImu;
        _newImu = false; // consume
        return true;
    }
    return false;
}

bool UARTLink_Esp32_1::getSonarData(SonarScanData &sonarData)
{
    if (_newSonar)
    {
        sonarData = _lastSonar;
        _newSonar = false; // consume
        return true;
    }
    return false;
}

void UARTLink_Esp32_1::handlePacket(const UARTPacket &pkt)
{
    if (_instance)
    {
        _instance->onPacket(pkt);
    }
}

void UARTLink_Esp32_1::onPacket(const UARTPacket &pkt)
{
    // we decode based on msgType
    switch (pkt.msgType)
    {
    case UARTMsgType::IMU_DATA:
    {
        if (pkt.length == sizeof(IMU_MPU6050_Data))
        {
            memcpy(&_lastImu, pkt.payload, sizeof(IMU_MPU6050_Data));
            _newImu = true;
        }
    }
    break;

    case UARTMsgType::SONAR_SWEEP_DATA:
    {
        if (pkt.length == sizeof(SonarScanData))
        {
            memcpy(&_lastSonar, pkt.payload, sizeof(SonarScanData));
            _newSonar = true;
        }
    }
    break;

    default:
        // ignore or handle error
        break;
    }
}
