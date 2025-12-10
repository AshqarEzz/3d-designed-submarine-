#ifndef MAIN_CONTROLLER_H
#define MAIN_CONTROLLER_H

#include <Arduino.h>
#include <vector>
#include "DataTypes.h"
#include "UARTLink_Esp32_1.h"
#include "UARTLink_Esp32_2.h"
#include "UARTLink_Esp32_3.h"
#include "I2CCommCam.h"

// We'll define a minimal PID struct for x-y.
struct PID_Controller
{
    float kp, ki, kd;
    float integratorX, integratorY;
    float prevErrorX, prevErrorY;
    unsigned long prevTimeMs;
};

class MainController
{
public:
    MainController();

    void begin();
    void update();

private:
    //===========================
    // Internal State
    //===========================
    Submarine_State _subState;
    Environment_State _envState;
    PathData _currentPath;
    PID_Controller _xyPid;

    // For delta time calculations
    unsigned long _lastUpdateMs;

    // Some config we might want
    float _timeStepSec; // e.g. 0.02 if we update at 50 Hz, etc.
    // For occupancy grid indexing
    // We'll define grid center index = 12,12,12 (or 11,11,11).
    static const int GRID_CENTER = 11;

    //===========================
    // Internal methods
    //===========================
    // 1) Communication stubs
    bool readDataFromESP32_1_Imu(IMU_MPU6050_Data &imuData);
    bool readDataFromESP32_1_Sonar(SonarScanData &sonarData);
    bool readDataFromESP32_2(Esp32_2_Data &data);
    bool writeCommandToESP32_3(const Esp32_3_Command &cmd);
    bool readPathFromCam(PathData &path);
    bool writeDataToCam(const MainToCamData &msg);

    // 2) Update submarine state
    void updateSubmarineStateFromIMU(const IMU_MPU6050_Data &imu, float dt);

    // 3) Occupancy grid
    void updateOccupancyGridFromSonar(const SonarScanData &sonar);
    void setOccupancyByWorldPos(float worldX, float worldY, float worldZ, bool occupied);
    bool shiftGridIfNeeded();
    void shiftGrid(int dx, int dy, int dz);

    // 4) Path logic
    void adjustPath(PathData &path);
    void followPathAndProduceCommands(float dt);

    // 5) XY PID
    void initPID(PID_Controller &pid, float kp, float ki, float kd);
    void resetPID(PID_Controller &pid);
    void computeXYControl(float desiredX, float desiredY, float dt, float &esc1_us, float &esc2_us);

    // 6) Depth logic
    PumpFlowCmd computeDepthCommand(float desiredZ, float currentZ, float threshold);

    // Utility
    float clamp(float val, float minVal, float maxVal);
};

#endif
