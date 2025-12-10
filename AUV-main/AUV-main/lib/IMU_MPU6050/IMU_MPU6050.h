#ifndef IMU_MPU6050_H
#define IMU_MPU6050_H

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

struct IMU_MPU6050_Config
{
    // I2C pins can be set if using custom Wire instance on ESP32
    // Typically, SDA=21, SCL=22 (default)
    int sdaPin = 21;
    int sclPin = 22;

    // I2C address, rarely changed from 0x68, but let's keep it configurable
    uint8_t i2cAddress = 0x68;

    // Accelerometer range: choose from {2,4,8,16} G
    // We'll store the Adafruit enumerations for convenience
    mpu6050_accel_range_t accelRange = MPU6050_RANGE_8_G;
    // Gyro range: choose from {250, 500, 1000, 2000} deg/s
    mpu6050_gyro_range_t gyroRange = MPU6050_RANGE_500_DEG;

    // Filter bandwidth
    mpu6050_bandwidth_t filterBandwidth = MPU6050_BAND_5_HZ;
};

struct IMU_MPU6050_Data
{
    float accelX;
    float accelY;
    float accelZ;
    float gyroX;
    float gyroY;
    float gyroZ;
    float temperature;
};

class IMU_MPU6050
{
public:
    IMU_MPU6050();
    bool begin(const IMU_MPU6050_Config &config);
    bool update(); // Read new data from the sensor
    IMU_MPU6050_Data getData() const;

private:
    Adafruit_MPU6050 mpu;
    IMU_MPU6050_Config _config;
    IMU_MPU6050_Data _data;
    bool _initialized;
};

#endif
