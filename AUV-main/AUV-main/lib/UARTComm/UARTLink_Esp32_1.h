#ifndef UART_LINK_ESP32_1_H
#define UART_LINK_ESP32_1_H

#include <Arduino.h>
#include "UARTComm.h"
#include "DataTypes.h" // for IMU_MPU6050_Data, SonarScanData, etc.

class UARTLink_Esp32_1
{
public:
    UARTLink_Esp32_1(HardwareSerial &serial, uint32_t baud);
    void begin();

    // Called in loop
    void update();

    // Return true if a new IMU data is available, fill 'imuData'
    bool getImuData(IMU_MPU6050_Data &imuData);

    // Return true if a new sonar sweep is available, fill 'sonarData'
    bool getSonarData(SonarScanData &sonarData);

    UARTComm &_comm() { return _commBase; }

private:
    UARTComm _commBase;
    // We store the last IMU data and a flag
    IMU_MPU6050_Data _lastImu;
    bool _newImu;

    // last sonar
    SonarScanData _lastSonar;
    bool _newSonar;

    // static callback for packet handling
    static void handlePacket(const UARTPacket &pkt);
    // but we need to route to instance => we do an instance pointer
    void onPacket(const UARTPacket &pkt);
    static UARTLink_Esp32_1 *_instance;
};

#endif
