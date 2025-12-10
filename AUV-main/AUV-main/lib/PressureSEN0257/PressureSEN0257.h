#ifndef PRESSURE_SEN0257_H
#define PRESSURE_SEN0257_H

#include <Arduino.h>

struct PressureSEN0257_Config
{
    int analogPin;         // The ADC pin connected to the sensor output
    float supplyVoltage;   // e.g. 5.0f
    int adcResolution;     // e.g. 4095 for 12-bit ADC on ESP32
    float offsetVoltage;   // e.g. 0.483f (the no-load voltage offset)
    float scaleKPaPerVolt; // e.g. 400 KPa per 1.0 Volt above offset
};

struct PressureSEN0257_Data
{
    float voltage;  // last raw voltage reading
    float pressure; // in KPa
};

class PressureSEN0257
{
public:
    PressureSEN0257();
    bool begin(const PressureSEN0257_Config &config);
    bool update();
    PressureSEN0257_Data getData() const;

private:
    PressureSEN0257_Config _config;
    PressureSEN0257_Data _data;
    bool _initialized;
};

#endif
