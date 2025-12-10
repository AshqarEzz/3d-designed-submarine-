#ifndef FLOW_YF_S201_H
#define FLOW_YF_S201_H

#include <Arduino.h>

struct FlowYF_S201_Config
{
    int sensorPin;           // Digital pin receiving pulses
    float calibrationFactor; // pulses per second per (desired flow unit)
};

struct FlowYF_S201_Data
{
    float flowRate;        // e.g. in L/min
    unsigned long totalMl; // total milliliters
};

class FlowYF_S201
{
public:
    FlowYF_S201();
    bool begin(const FlowYF_S201_Config &config);
    bool update(); // call at some interval (e.g., once per second)
    FlowYF_S201_Data getData() const;

    // Must be called from an ISR: attachInterrupt(digitalPinToInterrupt(sensorPin), <staticWrapper>, FALLING);
    static void IRAM_ATTR pulseISR();

private:
    static volatile unsigned long _pulseCount;

    FlowYF_S201_Config _config;
    FlowYF_S201_Data _data;
    bool _initialized;

    unsigned long _previousMillis;
    unsigned long _intervalMs;
};

#endif
