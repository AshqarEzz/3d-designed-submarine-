#include "L298NController.h"

L298NController::L298NController() : _initialized(false) {}

bool L298NController::begin(const L298NController_Config &config)
{
    _config = config;

    pinMode(_config.in1Pin, OUTPUT);
    pinMode(_config.in2Pin, OUTPUT);

    if (_config.usePwm)
    {
        ledcSetup(_config.pwmChannel, _config.pwmFrequency, _config.pwmResolution);
        ledcAttachPin(_config.enPin, _config.pwmChannel);
    }
    else
    {
        pinMode(_config.enPin, OUTPUT);
        digitalWrite(_config.enPin, HIGH); // default on
    }

    // default off
    digitalWrite(_config.in1Pin, LOW);
    digitalWrite(_config.in2Pin, LOW);

    _initialized = true;
    return true;
}

void L298NController::setFlow(L298N_FlowCmd flow, float speedPercent)
{
    if (!_initialized)
        return;

    // Constrain speed
    if (speedPercent < 0.0f)
        speedPercent = 0.0f;
    if (speedPercent > 100.0f)
        speedPercent = 100.0f;

    // Convert to duty
    float fraction = speedPercent / 100.0f;
    int duty = (int)(_config.pwmDutyMax * fraction);

    // Set direction
    switch (flow)
    {
    case L298N_FlowCmd::In:
        digitalWrite(_config.in1Pin, LOW);
        digitalWrite(_config.in2Pin, HIGH);
        break;
    case L298N_FlowCmd::Off:
        digitalWrite(_config.in1Pin, LOW);
        digitalWrite(_config.in2Pin, LOW);
        break;
    case L298N_FlowCmd::Out:
        digitalWrite(_config.in1Pin, HIGH);
        digitalWrite(_config.in2Pin, LOW);
        break;
    }

    if (_config.usePwm)
    {
        ledcWrite(_config.pwmChannel, duty);
    }
    else
    {
        // If not using PWM, either ON or OFF
        digitalWrite(_config.enPin, (speedPercent > 0.0f) ? HIGH : LOW);
    }
}
