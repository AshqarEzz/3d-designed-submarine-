#include "PressureSEN0257.h"

PressureSEN0257::PressureSEN0257() : _initialized(false)
{
    _data.voltage = 0.0f;
    _data.pressure = 0.0f;
}

bool PressureSEN0257::begin(const PressureSEN0257_Config &config)
{
    _config = config;
    pinMode(_config.analogPin, INPUT);
    _initialized = true;
    return true;
}

bool PressureSEN0257::update()
{
    if (!_initialized)
        return false;

    int raw = analogRead(_config.analogPin);
    float volts = ((float)raw / (float)_config.adcResolution) * _config.supplyVoltage;

    _data.voltage = volts;
    // pressure = (voltage - offset) * scale
    float diff = volts - _config.offsetVoltage;
    if (diff < 0)
        diff = 0; // clamp negative
    _data.pressure = diff * _config.scaleKPaPerVolt;

    return true;
}

PressureSEN0257_Data PressureSEN0257::getData() const
{
    return _data;
}
