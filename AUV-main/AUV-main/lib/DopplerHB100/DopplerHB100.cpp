#include "DopplerHB100.h"

// For a more sophisticated measurement, you could do interrupt-based zero-cross detection,
// or an FFT approach, etc. We keep it in a single function here for demonstration.

DopplerHB100::DopplerHB100() : _initialized(false)
{
    _data.frequency = 0.0f;
    _data.speedKmh = 0.0f;
    _data.speedMph = 0.0f;
}

bool DopplerHB100::begin(const DopplerHB100_Config &config)
{
    _config = config;
    pinMode(_config.analogPin, INPUT);
    _initialized = true;
    return true;
}

bool DopplerHB100::update()
{
    if (!_initialized)
        return false;

    // Acquire new frequency measurement
    float freq = measureFrequency();
    _data.frequency = freq;

    // Convert to speeds
    _data.speedKmh = freq / _config.freqToKmhFactor;
    _data.speedMph = freq / _config.freqToMphFactor;

    return true;
}

DopplerHB100_Data DopplerHB100::getData() const
{
    return _data;
}

/**
 *  measureFrequency()
 *  -> Pseudocode approach:
 *     1. Over a small sample time, measure analog values
 *     2. Count zero-cross events or peaks
 *     3. Convert to frequency
 */
float DopplerHB100::measureFrequency()
{
    const unsigned long measureWindowMs = 100; // measure for 100ms
    unsigned long startTime = millis();
    unsigned long zeroCrossCount = 0;

    int lastSample = analogRead(_config.analogPin);
    bool wasAbove = (lastSample > (int)_config.midpoint);

    while (millis() - startTime < measureWindowMs)
    {
        int sample = analogRead(_config.analogPin);
        bool isAbove = (sample > (int)_config.midpoint + (int)_config.sensitivity);
        if (isAbove != wasAbove)
        {
            zeroCrossCount++;
            wasAbove = isAbove;
        }
    }

    // one full cycle is 2 zero-cross events, so freq = zeroCrossCount / 2 / (windowSec)
    float windowSec = (float)measureWindowMs / 1000.0f;
    float freq = (zeroCrossCount / 2.0f) / windowSec;

    return freq;
}
