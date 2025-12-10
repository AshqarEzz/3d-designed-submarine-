#ifndef ESC_CONTROLLER_H
#define ESC_CONTROLLER_H

#include <Arduino.h>

struct ESCController_Config
{
    int signalPin;      // The pin that controls the ESC
    int ledcChannel;    // 0..15
    int ledcTimer;      // 0..3
    uint32_t frequency; // typically 50 for servo/ESC
    uint8_t resolution; // typically 16 bits
    // Pulse width bounds (microseconds)
    uint32_t minPulseUs;     // e.g. 1000
    uint32_t maxPulseUs;     // e.g. 2000
    uint32_t neutralPulseUs; // e.g. 1500
};

class ESCController
{
public:
    ESCController();
    bool begin(const ESCController_Config &config);

    // Writes a pulse in microseconds. For example:
    //  - 1000 => full speed reverse
    //  - 1500 => neutral
    //  - 2000 => full speed forward
    // (depending on your ESC calibration)
    void writeMicroseconds(uint32_t us);

private:
    ESCController_Config _config;
    bool _initialized;

    // Helper to convert microseconds to LEDC duty
    uint32_t computeDutyUS(uint32_t us);
};

#endif
