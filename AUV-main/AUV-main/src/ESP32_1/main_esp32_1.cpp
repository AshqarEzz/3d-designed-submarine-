#include <Arduino.h>
#include <Wire.h>
#include <Servo.h>

// Sensor libraries
#include "IMU_MPU6050.h"
#include "UltrasonicAJ_SR04M.h"

// Communication library
#include "UARTComm.h"
#include "UARTLink_Esp32_1.h"

// Data types
#include "DataTypes.h"

//-------------------------------------------------------------------
// Hardware Pins (configurable)
//-------------------------------------------------------------------
static const int PIN_SDA = 21;
static const int PIN_SCL = 22;

static const int PIN_SERVO = 18; // servo control pin
static const int PIN_TRIG = 5;   // ultrasonic trig
static const int PIN_ECHO = 4;   // ultrasonic echo

//-------------------------------------------------------------------
// We create the sensor instances
//-------------------------------------------------------------------
IMU_MPU6050 imu;
UltrasonicAJ_SR04M ultrasonic;

// Config objects
IMU_MPU6050_Config imuConfig;
UltrasonicAJ_SR04M_Config sonarConfig;

// The servo object
Servo sonarServo;

//-------------------------------------------------------------------
// Communication
//-------------------------------------------------------------------
// We'll assume we use HardwareSerial1 on pins GPIO9(RX1) / GPIO10(TX1),
// or whichever is valid on your board. Adjust as needed.
UARTLink_Esp32_1 link1(Serial1, 115200);

//-------------------------------------------------------------------
// Global variables controlling the servo sweep
//-------------------------------------------------------------------
static const float SWEEP_MIN_ANGLE = -60.0f; // deg
static const float SWEEP_MAX_ANGLE = 60.0f;  // deg
static const float SWEEP_STEP = 2.0f;        // deg step
// If we do smaller steps (e.g. 1 deg), the array can hold up to 120 readings.
// Our SonarScanData can hold up to 240. We'll use 2 deg for demonstration.

static float currentAngle = SWEEP_MIN_ANGLE;
static bool sweepingRight = true;

SonarScanData sonarScan;
bool newSweepReady = false;

//-------------------------------------------------------------------
// Timer controlling IMU read rate
//-------------------------------------------------------------------
unsigned long lastImuReadMs = 0;
static const unsigned long IMU_READ_INTERVAL_MS = 20; // ~50 Hz

//-------------------------------------------------------------------
// Setup
//-------------------------------------------------------------------
void setup()
{
    Serial.begin(115200);
    delay(1000);
    Serial.println("ESP32-1 firmware starting...");

    // 1) IMU
    imuConfig.sdaPin = PIN_SDA;
    imuConfig.sclPin = PIN_SCL;
    imuConfig.i2cAddress = 0x68;
    imuConfig.accelRange = MPU6050_RANGE_8_G;
    imuConfig.gyroRange = MPU6050_RANGE_500_DEG;
    imuConfig.filterBandwidth = MPU6050_BAND_5_HZ;

    bool imuOk = imu.begin(imuConfig);
    if (!imuOk)
    {
        Serial.println("ERROR: IMU init failed!");
    }
    else
    {
        Serial.println("IMU init OK");
    }

    // 2) Ultrasonic
    sonarConfig.trigPin = PIN_TRIG;
    sonarConfig.echoPin = PIN_ECHO;
    sonarConfig.speedOfSoundCmPerUs = 0.0343f; // typical
    sonarConfig.maxDistanceCm = 400;
    ultrasonic.begin(sonarConfig);

    // 3) Servo
    sonarServo.attach(PIN_SERVO);
    // Move servo to start
    sonarServo.write(map((int)SWEEP_MIN_ANGLE, -90, 90, 0, 180));

    // Prepare sonarScan data structure
    memset(&sonarScan, 0, sizeof(sonarScan));
    sonarScan.angleResolution = SWEEP_STEP;
    sonarScan.numMeasurements = 0;

    // 4) UART link
    link1.begin();

    // Ready
    Serial.println("ESP32-1 setup complete.");
}

void loop()
{
    // 1) Update communication
    link1.update();
    // (Though this device mostly sends data out,
    //  we might still read if we had servo config commands from Main, etc.)

    // 2) Read IMU at fixed interval
    unsigned long now = millis();
    if (now - lastImuReadMs >= IMU_READ_INTERVAL_MS)
    {
        lastImuReadMs = now;
        // update
        imu.update();
        IMU_MPU6050_Data data = imu.getData();

        // Send IMU data via link1
        // We call the underlying UARTComm "sendPacket(...)"
        // But in our design, link1 is from the "Main" perspective.
        // We can do a direct call to link1's "comm" or replicate a "sendImuData()" method.
        // We'll do a direct approach here for clarity.

        // Let's do a minimal "sendImuData" approach in link1 for cleanliness:
        sendImuData(data);
    }

    // 3) Sweep servo logic
    //    We'll do this continuously, stepping a little each loop iteration.
    servoSweepAndMeasure();

    // 4) If a sweep is completed, send the sonarScan
    if (newSweepReady)
    {
        // Pack into a binary packet and send
        sendSonarData(sonarScan);

        // Prepare for next sweep
        newSweepReady = false;
        sonarScan.numMeasurements = 0;
    }

    // Some small delay
    delay(5);
}

//-------------------------------------------------------------------
// Helper: read IMU data => send to Main
// Rather than overcomplicating, let's define a function:
void sendImuData(const IMU_MPU6050_Data &imuData)
{
    // We can do a direct approach:
    // link1._comm.sendPacket(UARTMsgType::IMU_DATA, (uint8_t*)&imuData, sizeof(IMU_MPU6050_Data));
    // But _comm is private in the class. So let's add a "sendCustomPacket" approach in our link
    // or just do a partial approach. We'll define it inline:

    // We'll forcibly reinterpret link1 as a friend approach or add a small function in the link1:
    // We'll do the latter: define "sendImuPacket(...)"

    // Implementation is below.
    link1._comm().sendPacket(UARTMsgType::IMU_DATA,
                             (uint8_t *)&imuData,
                             sizeof(IMU_MPU6050_Data));
}

void sendSonarData(const SonarScanData &scan)
{
    link1._comm().sendPacket(UARTMsgType::SONAR_SWEEP_DATA,
                             (uint8_t *)&scan,
                             sizeof(SonarScanData));
}

//-------------------------------------------------------------------
// The servo sweep function
//-------------------------------------------------------------------
void servoSweepAndMeasure()
{
    // We'll move the servo in small increments each loop
    // then do a measurement, store in sonarScan array.

    // Convert angle in [-60..60] to servo [0..180].
    int servoAngle = map((int)currentAngle, -90, 90, 0, 180);
    sonarServo.write(servoAngle);

    // Wait a bit to let servo settle. Let's do a short delay:
    delay(10);

    // Now read distance
    ultrasonic.readDistance();
    float dist = ultrasonic.getData().distanceCm;
    // Store in sonarScan
    uint16_t idx = sonarScan.numMeasurements;
    if (idx < SonarScanData::MAX_NUM_MEASUREMENTS)
    {
        // If we want the angle to match the array index as well:
        //   angle = -60 + idx * resolution
        // But we'll store only the measured distance
        if (dist >= sonarConfig.maxDistanceCm)
        {
            // means out of range => let's store -1
            sonarScan.distances[idx] = -1.0f;
        }
        else
        {
            sonarScan.distances[idx] = dist;
        }
        sonarScan.numMeasurements++;
    }

    // Move angle
    if (sweepingRight)
    {
        currentAngle += SWEEP_STEP;
        if (currentAngle >= SWEEP_MAX_ANGLE)
        {
            // done
            sweepingRight = false;
            newSweepReady = true;
        }
    }
    else
    {
        currentAngle -= SWEEP_STEP;
        if (currentAngle <= SWEEP_MIN_ANGLE)
        {
            // done
            sweepingRight = true;
            newSweepReady = true;
        }
    }
}

//-------------------------------------------------------------------
// Minimal extension: let's make the link1 give us a direct handle
// to the underlying _comm. We add a friend or accessor in `UARTLink_Esp32_1`.
// We'll just do a quick accessor approach.
//-------------------------------------------------------------------
