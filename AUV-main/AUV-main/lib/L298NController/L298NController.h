#ifndef L298N_CONTROLLER_H
#define L298N_CONTROLLER_H

#include <Arduino.h>

// The flow command: -1 (reverse), 0 (stop), +1 (forward)
enum class L298N_FlowCmd
{
    In = -1,
    Off = 0,
    Out = 1
};

struct L298NController_Config
{
    int enPin; // PWM pin controlling speed (if needed).
               // Or you can tie it to 5V if you want full speed always.
    int in1Pin;
    int in2Pin;
    // For optional PWM:
    bool usePwm = false;
    int pwmChannel = 0;
    int pwmFrequency = 1000;
    int pwmResolution = 8;
    int pwmDutyMax = 255; // if 8-bit resolution
};

class L298NController
{
public:
    L298NController();
    bool begin(const L298NController_Config &config);

    // Set flow direction/speed. For your submarine:
    //   flow = In  => in1=LOW,  in2=HIGH
    //   flow = Off => in1=LOW,  in2=LOW
    //   flow = Out => in1=HIGH, in2=LOW
    // speed is a value [0..100%], if using PWM, else it’s ignored
    void setFlow(L298N_FlowCmd flow, float speedPercent = 100.0f);

private:
    L298NController_Config _config;
    bool _initialized;
};

#endif
