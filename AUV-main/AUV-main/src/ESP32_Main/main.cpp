#include <Arduino.h>
#include "MainController.h"

// We have 3 hardware serial ports on some ESP32 boards: Serial, Serial1, Serial2, etc.
// We'll decide:
//   - ESP32-1 on Serial1
//   - ESP32-2 on Serial2
//   - ESP32-3 on Serial2 (actually we might need Serial2 & Serial1, depends on pins).
// For demonstration, let's assume we have enough. We'll do:
//   Serial1 -> Esp32-1
//   Serial2 -> Esp32-2
//   We create a SoftwareSerial or use a third hardware, etc. for Esp32-3

MainController mainCtrl;

// We'll define them globally:
UARTLink_Esp32_1 link1(Serial1, 115200);
UARTLink_Esp32_2 link2(Serial2, 115200);
UARTLink_Esp32_3 link3(Serial2, 115200); // or maybe Serial3 if we have it
I2CCommCam camComm(0x30);

void setup()
{
    Serial.begin(115200);

    // init links
    link1.begin();
    link2.begin();
    link3.begin();
    camComm.begin(/*sdaPin=*/21, /*sclPin=*/22, /*freq=*/100000);

    mainCtrl.begin();
}

void loop()
{
    // update comm links
    link1.update();
    link2.update();
    link3.update();
    // no explicit update for I2C, we do on demand

    mainCtrl.update();
    delay(10);
}
