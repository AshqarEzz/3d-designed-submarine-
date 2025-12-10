#include <Arduino.h>

// Actuator libraries
#include "ESCController.h"
#include "L298NController.h"

// Communication
#include "UARTComm.h"
#include "UARTLink_Esp32_3.h"

// Data types
#include "DataTypes.h"

//-------------------------------------------------------------------
// Hardware pins (adjust to real wiring!)
//-------------------------------------------------------------------

// ESC #1
static const int PIN_ESC1 = 25;
// ESC #2
static const int PIN_ESC2 = 26;
// We might create 2 separate ESCController or one if we prefer

ESCController esc1;
ESCController esc2;

// L298N pump
L298NController pump;
L298NController_Config pumpConfig = {
    .enPin = 14, // optional PWM for pump speed, or tie to 5V
    .in1Pin = 12,
    .in2Pin = 13,
    .usePwm = false, // we just do ON/OFF
    .pwmChannel = 0,
    .pwmFrequency = 1000,
    .pwmResolution = 8,
    .pwmDutyMax = 255};

// We also need config for ESCs
ESCController_Config esc1Config = {
    .signalPin = PIN_ESC1,
    .ledcChannel = 0,
    .ledcTimer = 0,
    .frequency = 50,
    .resolution = 16,
    .minPulseUs = 1000,
    .maxPulseUs = 2000,
    .neutralPulseUs = 1500};

ESCController_Config esc2Config = {
    .signalPin = PIN_ESC2,
    .ledcChannel = 1,
    .ledcTimer = 0,
    .frequency = 50,
    .resolution = 16,
    .minPulseUs = 1000,
    .maxPulseUs = 2000,
    .neutralPulseUs = 1500};

//-------------------------------------------------------------------
// Communication
//-------------------------------------------------------------------
// We'll assume we use e.g. Serial1 for receiving from Main
UARTLink_Esp32_3 link3(Serial1, 115200);

//-------------------------------------------------------------------
// We want a callback approach to parse the incoming command.
// But in our design from Message 3.2, we used a "sendCommand" from the Main side.
// On the receiving side, we need to handle the packet.
// We'll do so in "UARTLink_Esp32_3" but that code was minimal.
// Let's expand it to call a user callback on "COMMAND_ESC_PUMP" arrival.
//-------------------------------------------------------------------

// We'll define a function that sets the hardware when we get the command
void applyEscPumpCommand(const Esp32_3_Command &cmd)
{
    // 1) ESC signals
    esc1.writeMicroseconds(cmd.esc1_us);
    esc2.writeMicroseconds(cmd.esc2_us);

    // 2) Pump
    pump.setFlow((L298N_FlowCmd)cmd.pumpCmd);
}

void setup()
{
    Serial.begin(115200);
    delay(1000);
    Serial.println("ESP32-3 firmware starting...");

    // init ESCs
    esc1.begin(esc1Config);
    esc2.begin(esc2Config);

    // init pump
    pump.begin(pumpConfig);

    // link
    link3.begin();
    link3.onCommandReceived(applyEscPumpCommand);

    // (We must ensure link3 calls a callback to parse the incoming command
    //  and call `applyEscPumpCommand(...)`. We'll do it with a static approach
    //  or a "global" approach. We'll implement that next in link3's code.)

    Serial.println("ESP32-3 setup complete.");
}

void loop()
{
    link3.update();
    // that's it. We'll rely on the incoming messages to set the hardware

    delay(10);
}
