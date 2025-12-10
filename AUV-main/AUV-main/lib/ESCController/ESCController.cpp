#include "ESCController.h"

ESCController::ESCController() : _initialized(false) {}

bool ESCController::begin(const ESCController_Config &config)
{
    _config = config;

    ledcSetup(_config.ledcChannel, _config.frequency, _config.resolution);
    ledcAttachPin(_config.signalPin, _config.ledcChannel);

    // Initialize at neutral
    writeMicroseconds(_config.neutralPulseUs);

    _initialized = true;
    return true;
}

void ESCController::writeMicroseconds(uint32_t us)
{
    if (!_initialized)
        return;

    // Constrain to min/max
    if (us < _config.minPulseUs)
        us = _config.minPulseUs;
    if (us > _config.maxPulseUs)
        us = _config.maxPulseUs;

    uint32_t duty = computeDutyUS(us);
    ledcWrite(_config.ledcChannel, duty);
}

uint32_t ESCController::computeDutyUS(uint32_t us)
{
    // period = 1/_config.frequency (in s) => in microseconds => 1e6 / freq
    double periodUs = 1e6 / (double)_config.frequency;
    double fraction = (double)us / periodUs;
    uint32_t maxDuty = (1 << _config.resolution) - 1;
    return (uint32_t)(fraction * maxDuty);
}
