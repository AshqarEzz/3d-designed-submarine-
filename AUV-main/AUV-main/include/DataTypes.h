#ifndef DATA_TYPES_H
#define DATA_TYPES_H

#include <Arduino.h>
#include <vector>

//----------------------------------
// 1) Data from ESP32-1
//----------------------------------
enum class Esp32_1_MessageType : uint8_t
{
    IMU_DATA = 0,
    SONAR_SWEEP_DATA
};

// IMU_MPU6050_Data is already in IMU library, but let's re-declare a reference here
struct IMU_MPU6050_Data
{
    float accelX;
    float accelY;
    float accelZ;
    float gyroX;
    float gyroY;
    float gyroZ;
    float temperature;
};

struct SonarScanData
{
    // If resolution=1 degree, we might have up to 121 measurements for -60..+60 inclusive
    // But to keep it generic, we store them in a fixed-size array.
    // The actual "used" length is in 'numMeasurements'.
    static const uint16_t MAX_NUM_MEASUREMENTS = 240;
    uint16_t numMeasurements;
    // distances in cm. -1 means no obstacle detected in that direction.
    // We store them as float for clarity, or int16_t if we prefer fixed precision.
    float distances[MAX_NUM_MEASUREMENTS];
    // The angle step in degrees. E.g. 1.0 => 120 steps, 5.0 => 24 steps, etc.
    float angleResolution;
};

//----------------------------------
// 2) Data from ESP32-2
//----------------------------------
struct Esp32_2_Data
{
    // Doppler
    float frequency;
    float speedKmh;
    float speedMph;

    // Pressure
    float voltage;
    float pressureKPa;

    // Flow
    float flowRate; // e.g. L/min
    unsigned long totalMl;
};

//----------------------------------
// 3) Command to ESP32-3
//----------------------------------

// PumpFlow = -1 => In, 0 => Off, +1 => Out
// We'll store them in an enum or just use int8_t
enum class PumpFlowCmd : int8_t
{
    IN = -1,
    OFF = 0,
    OUT = 1
};

struct Esp32_3_Command
{
    // Two ESC signals in microseconds
    uint16_t esc1_us;
    uint16_t esc2_us;
    // Pump command
    PumpFlowCmd pumpCmd;
};

//----------------------------------
// 4) Submarine State
//----------------------------------
struct Submarine_State
{
    // linear
    float posX, posY, posZ;       // position
    float velX, velY, velZ;       // velocity
    float accelX, accelY, accelZ; // from IMU
    // rotational
    float roll, pitch, yaw;    // euler angles (we'll keep it simple)
    float gyroX, gyroY, gyroZ; // rad/s from IMU
};

//----------------------------------
// 5) Occupancy Grid
//----------------------------------
const int GRID_SIZE = 25; // each dimension
struct Environment_State
{
    bool occupancyGrid[GRID_SIZE][GRID_SIZE][GRID_SIZE];
    float resolution; // cm per cell
    // The absolute world coordinates of the "center cell"
    // i.e. occupancyGrid[12,12,12], or [11,11,11], whichever reference we pick
    // We'll pick (11,11,11) as "center" so that we can go from 0..24 => ±12
    float centerX, centerY, centerZ;
};

//----------------------------------
// 6) Path
//----------------------------------
struct Waypoint
{
    float x, y, z;
};

struct PathData
{
    std::vector<Waypoint> waypoints;
    bool isNew;
};

//----------------------------------
// 7) Sub->Cam data
//----------------------------------
// Possibly a subset for sending to the camera
// We'll keep it minimal, or store the entire state
struct MainToCamData
{
    Submarine_State subState;
    // We might not send the entire 25x25x25 due to bandwidth concerns
    // but let's say we do for completeness
    Environment_State envState;
};

#endif
