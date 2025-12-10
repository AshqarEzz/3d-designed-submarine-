#include "FlowYF_S201.h"

volatile unsigned long FlowYF_S201::_pulseCount = 0;

FlowYF_S201::FlowYF_S201()
    : _initialized(false), _previousMillis(0), _intervalMs(1000)
{
    _data.flowRate = 0.0f;
    _data.totalMl = 0;
}

bool FlowYF_S201::begin(const FlowYF_S201_Config &config)
{
    _config = config;
    pinMode(_config.sensorPin, INPUT_PULLUP);

    // Reset counts
    _pulseCount = 0;
    _data.flowRate = 0.0f;
    _data.totalMl = 0;

    // Attach the ISR
    attachInterrupt(digitalPinToInterrupt(_config.sensorPin), pulseISR, FALLING);

    _initialized = true;
    _previousMillis = millis();
    return true;
}

bool FlowYF_S201::update()
{
    if (!_initialized)
        return false;

    unsigned long currentMillis = millis();
    if (currentMillis - _previousMillis >= _intervalMs)
    {
        // Calculate flow
        unsigned long pCount = _pulseCount;
        _pulseCount = 0; // reset for next interval
        _previousMillis = currentMillis;

        // flowRate = pulses/sec / calibrationFactor
        // We measured pulses in 1 second => pCount pulses/s
        float pulsesPerSecond = (float)pCount;
        _data.flowRate = pulsesPerSecond / _config.calibrationFactor;

        // For example, if flowRate is L/min:
        //   L/min = (pulses/s)/calFactor,
        // Then we convert that to ml for total:
        //   flowRate(L/min) = flowRate / 60 (L/s)
        //   => ml/s = flowRate(L/s) * 1000
        //   => ml/s = (_data.flowRate / 60.0f) * 1000
        float mlPerSecond = (_data.flowRate / 60.0f) * 1000.0f;
        // Over the last second:
        _data.totalMl += (unsigned long)(mlPerSecond);
    }

    return true;
}

FlowYF_S201_Data FlowYF_S201::getData() const
{
    return _data;
}

void IRAM_ATTR FlowYF_S201::pulseISR()
{
    _pulseCount++;
}
