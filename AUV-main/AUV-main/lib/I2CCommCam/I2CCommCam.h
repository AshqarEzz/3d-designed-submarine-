#ifndef I2C_COMM_CAM_H
#define I2C_COMM_CAM_H

#include <Arduino.h>
#include <Wire.h>
#include "DataTypes.h"

class I2CCommCam
{
public:
    I2CCommCam(uint8_t camI2CAddress = 0x30);

    void begin(int sdaPin = 21, int sclPin = 22, uint32_t frequency = 100000);

    // Send submarine+env data to cam
    bool sendMainToCam(const MainToCamData &data);

    // Check if there's a new path. If so, read it
    bool getPathData(PathData &path);

private:
    uint8_t _addr;

    // Helper to serialize MainToCamData
    bool writeMainToCamData(const MainToCamData &data);

    // Helper to parse PathData from an I2C read
    bool readPathData(PathData &path);
};

#endif
