#include "IMU_MPU6050.h"

IMU_MPU6050::IMU_MPU6050() : _initialized(false) {}

bool IMU_MPU6050::begin(const IMU_MPU6050_Config &config)
{
    _config = config;
    Wire.begin(_config.sdaPin, _config.sclPin);

    if (!mpu.begin(_config.i2cAddress, &Wire))
    {
        // Could not find MPU6050 chip
        _initialized = false;
        return false;
    }

    // Configure ranges
    mpu.setAccelerometerRange(_config.accelRange);
    mpu.setGyroRange(_config.gyroRange);
    mpu.setFilterBandwidth(_config.filterBandwidth);

    _initialized = true;
    return true;
}

bool IMU_MPU6050::update()
{
    if (!_initialized)
        return false;

    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    _data.accelX = a.acceleration.x;
    _data.accelY = a.acceleration.y;
    _data.accelZ = a.acceleration.z;
    _data.gyroX = g.gyro.x;
    _data.gyroY = g.gyro.y;
    _data.gyroZ = g.gyro.z;
    _data.temperature = temp.temperature;

    return true;
}

IMU_MPU6050_Data IMU_MPU6050::getData() const
{
    return _data;
}
