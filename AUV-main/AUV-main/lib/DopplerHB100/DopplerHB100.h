#ifndef DOPPLER_HB100_H
#define DOPPLER_HB100_H

#include <Arduino.h>

struct DopplerHB100_Config
{
    int analogPin;        // The pin reading the Doppler signal
    uint16_t midpoint;    // The approximate DC offset for the signal
    uint16_t sensitivity; // The threshold
    // For converting frequency to speed. For example:
    //   speed(km/h) = freq / freqToKmhFactor
    // or
    //   speed(m/s) = freq / freqToMsFactor
    // We'll keep it flexible:
    float freqToKmhFactor = 19.49f; // from example
    float freqToMphFactor = 31.36f; // from example
};

struct DopplerHB100_Data
{
    float frequency;
    float speedKmh;
    float speedMph;
};

class DopplerHB100
{
public:
    DopplerHB100();
    bool begin(const DopplerHB100_Config &config);
    bool update();
    DopplerHB100_Data getData() const;

private:
    DopplerHB100_Config _config;
    DopplerHB100_Data _data;
    bool _initialized;

    // Internal method to measure frequency.
    // Implement your logic (zero-cross, peak detection, or a library).
    float measureFrequency();
};

#endif
