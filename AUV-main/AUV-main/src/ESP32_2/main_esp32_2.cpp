#include <Arduino.h>

// Our sensor libraries
#include "DopplerHB100.h"
#include "PressureSEN0257.h"
#include "FlowYF_S201.h"

// Communication
#include "UARTComm.h"
#include "UARTLink_Esp32_2.h"

// Data types
#include "DataTypes.h"

//-------------------------------------------------------------------
// Pin definitions (adjust to real wiring)
//-------------------------------------------------------------------
static const int PIN_DOPPLER = 34;  // analog input
static const int PIN_PRESSURE = 35; // analog input
static const int PIN_FLOW = 27;     // digital input (pulses)

//-------------------------------------------------------------------
// Doppler config
//-------------------------------------------------------------------
DopplerHB100 doppler;
DopplerHB100_Config dopplerConfig = {
    .analogPin = PIN_DOPPLER,
    .midpoint = 512,   // depends on your ADC offset
    .sensitivity = 10, // threshold
    .freqToKmhFactor = 19.49f,
    .freqToMphFactor = 31.36f};

// Pressure config
PressureSEN0257 pressure;
PressureSEN0257_Config pressConfig = {
    .analogPin = PIN_PRESSURE,
    .supplyVoltage = 5.0f,
    .adcResolution = 4095, // for 12-bit ADC on ESP32
    .offsetVoltage = 0.483f,
    .scaleKPaPerVolt = 400.0f};

// Flow config
FlowYF_S201 flow;
FlowYF_S201_Config flowConfig = {
    .sensorPin = PIN_FLOW,
    .calibrationFactor = 4.5f // adjust for your sensor
};

//-------------------------------------------------------------------
// Communication
//-------------------------------------------------------------------
UARTLink_Esp32_2 link2(Serial1, 115200); // or whichever Serial port
//-------------------------------------------------------------------

// We will collect data e.g. every 500ms
unsigned long lastSendMs = 0;
static const unsigned long SEND_INTERVAL_MS = 500;

void setup()
{
    Serial.begin(115200);
    delay(1000);
    Serial.println("ESP32-2 firmware starting...");

    // 1) Setup sensors
    doppler.begin(dopplerConfig);
    pressure.begin(pressConfig);
    flow.begin(flowConfig);

    // 2) Communication
    link2.begin();

    Serial.println("ESP32-2 setup complete.");
}

void loop()
{
    // 1) update comm
    link2.update();

    // 2) update sensors
    // doppler => we call doppler.update() to measure frequency
    doppler.update();
    pressure.update();
    flow.update();

    // 3) every 500ms, send data
    unsigned long now = millis();
    if (now - lastSendMs >= SEND_INTERVAL_MS)
    {
        lastSendMs = now;

        // collect
        DopplerHB100_Data dopData = doppler.getData();
        PressureSEN0257_Data pressData = pressure.getData();
        FlowYF_S201_Data flowData = flow.getData();

        Esp32_2_Data out;
        out.frequency = dopData.frequency;
        out.speedKmh = dopData.speedKmh;
        out.speedMph = dopData.speedMph;
        out.voltage = pressData.voltage;
        out.pressureKPa = pressData.pressure;
        out.flowRate = flowData.flowRate;
        out.totalMl = flowData.totalMl;

        // Now we want to send it to Main
        // We'll do something like:
        link2.sendData(out);
    }

    delay(5);
}
