#include "MainController.h"
#include "DataTypes.h"

extern UARTLink_Esp32_1 link1;
extern UARTLink_Esp32_2 link2;
extern UARTLink_Esp32_3 link3;
extern I2CCommCam camComm;

MainController::MainController()
    : _lastUpdateMs(0), _timeStepSec(0.01f) // default 10ms
{
    // Initialize submarine state to 0
    memset(&_subState, 0, sizeof(Submarine_State));

    // Initialize environment state
    memset(_envState.occupancyGrid, false, sizeof(_envState.occupancyGrid));
    _envState.resolution = 10.0f; // cm per cell
    _envState.centerX = 0.0f;     // initial world center
    _envState.centerY = 0.0f;
    _envState.centerZ = 0.0f;

    // Initialize path
    _currentPath.isNew = false;

    // Initialize XY PID
    initPID(_xyPid, /*kp=*/1.0f, /*ki=*/0.0f, /*kd=*/0.0f);
}

void MainController::begin()
{
    _lastUpdateMs = millis();
}

void MainController::update()
{
    // 1) Calculate delta time
    unsigned long now = millis();
    float dt = (now - _lastUpdateMs) / 1000.0f;
    if (dt <= 0.0f)
        dt = _timeStepSec; // fallback if something weird
    _lastUpdateMs = now;

    // 2) Read from ESP32-1: IMU data
    IMU_MPU6050_Data imuData;
    if (readDataFromESP32_1_Imu(imuData))
    {
        updateSubmarineStateFromIMU(imuData, dt);
    }

    // 3) Possibly read from ESP32-1: Sonar sweep data
    //    If available, update occupancy grid
    SonarScanData sonarData;
    if (readDataFromESP32_1_Sonar(sonarData))
    {
        updateOccupancyGridFromSonar(sonarData);
    }

    // 4) Read from ESP32-2: doppler, pressure, flow
    Esp32_2_Data data2;
    if (readDataFromESP32_2(data2))
    {
        // We can do something with doppler speed, pressure, flow here if needed
        // e.g. doppler speed might refine velocity measure, etc.
        // We keep it simpler.
    }

    // 5) Check if a new path arrived from ESP32-CAM
    PathData newPath;
    if (readPathFromCam(newPath))
    {
        // If there's a new path, store it
        if (newPath.isNew)
        {
            // Potentially adjust or re-calc
            adjustPath(newPath);
            _currentPath = newPath;
            _currentPath.isNew = false;
            // Also reset XY PID
            resetPID(_xyPid);
        }
    }

    // 6) If shifting grid is needed because sub is out of boundary, do it
    shiftGridIfNeeded();

    // 7) Follow path logic => produce commands to motors/pumps
    followPathAndProduceCommands(dt);

    // 8) Send Submarine_State + Env_State to CAM
    MainToCamData camData;
    camData.subState = _subState;
    camData.envState = _envState;
    writeDataToCam(camData);
}

/*=============================================================================
 * 1) Communication Stubs
 *============================================================================*/
bool MainController::readDataFromESP32_1_Imu(IMU_MPU6050_Data &imuData)
{
    // ask link1 if it has new IMU
    return link1.getImuData(imuData);
}

bool MainController::readDataFromESP32_1_Sonar(SonarScanData &sonarData)
{
    // ask link1 if it has new sonar
    return link1.getSonarData(sonarData);
}

bool MainController::readDataFromESP32_2(Esp32_2_Data &data)
{
    return link2.getData(data);
}

bool MainController::writeCommandToESP32_3(const Esp32_3_Command &cmd)
{
    link3.sendCommand(cmd);
    return true;
}

bool MainController::readPathFromCam(PathData &path)
{
    // Attempt to read from camComm
    // returns true if we got new data
    return camComm.getPathData(path);
}

bool MainController::writeDataToCam(const MainToCamData &msg)
{
    return camComm.sendMainToCam(msg);
}

/*=============================================================================
 * 2) Update Submarine State from IMU
 *    This merges the linear accel with velocity, position, etc.
 *============================================================================*/
void MainController::updateSubmarineStateFromIMU(const IMU_MPU6050_Data &imu, float dt)
{
    // Update submarine acceleration from IMU
    _subState.accelX = imu.accelX;
    _subState.accelY = imu.accelY;
    _subState.accelZ = imu.accelZ;
    // Integrate velocity
    _subState.velX += _subState.accelX * dt;
    _subState.velY += _subState.accelY * dt;
    _subState.velZ += _subState.accelZ * dt;
    // Integrate position
    _subState.posX += _subState.velX * dt + 0.5f * _subState.accelX * dt * dt;
    _subState.posY += _subState.velY * dt + 0.5f * _subState.accelY * dt * dt;
    _subState.posZ += _subState.velZ * dt + 0.5f * _subState.accelZ * dt * dt;

    // Update gyro
    _subState.gyroX = imu.gyroX;
    _subState.gyroY = imu.gyroY;
    _subState.gyroZ = imu.gyroZ;

    // For roll/pitch/yaw, we do a naive integration (assuming gyro in rad/s):
    _subState.roll += _subState.gyroX * dt;
    _subState.pitch += _subState.gyroY * dt;
    _subState.yaw += _subState.gyroZ * dt;
}

/*=============================================================================
 * 3) Occupancy Grid from Sonar
 *============================================================================*/
void MainController::updateOccupancyGridFromSonar(const SonarScanData &sonar)
{
    // We have angles from -60 to +60, step=sonar.angleResolution
    // For each measurement i:
    //   angle_i = -60 + i*angleResolution
    //   distance_i = sonar.distances[i]
    // Convert (angle, distance) to a world coordinate (assuming heading = yaw)
    // then set occupancyGrid[...] = 1 for that cell.

    float headingRad = _subState.yaw; // submarine's yaw in radians

    for (uint16_t i = 0; i < sonar.numMeasurements && i < SonarScanData::MAX_NUM_MEASUREMENTS; i++)
    {
        float dist = sonar.distances[i];
        float angleDeg = -60.0f + i * sonar.angleResolution; // example
        float angleRad = angleDeg * 3.14159f / 180.0f;

        if (dist < 0)
        {
            // Means no obstacle => we can set all cells up to max range as free
            // For now, we do nothing or we can mark them free
            // if we want to 'clear' that sector. Implementation choice.
            continue;
        }

        // Convert local polar (angleRad, dist) -> local X/Y
        float localX = dist * cos(angleRad);
        float localY = dist * sin(angleRad);
        // We ignore Z for now, or we can consider a horizontal plane.
        // For advanced 3D, you'd need pitch as well, or some known sensor orientation.

        // Then rotate by submarine's heading
        float globalX = _subState.posX + (localX * cos(headingRad) - localY * sin(headingRad));
        float globalY = _subState.posY + (localX * sin(headingRad) + localY * cos(headingRad));
        float globalZ = _subState.posZ; // we do not track vertical angle for now, or we can do if we have tilt data

        // Mark that position as occupied
        setOccupancyByWorldPos(globalX, globalY, globalZ, true);
    }
}

void MainController::setOccupancyByWorldPos(float worldX, float worldY, float worldZ, bool occupied)
{
    // 1) Convert from world coords => grid index
    // delta = worldPos - envState.center
    float dx = worldX - _envState.centerX;
    float dy = worldY - _envState.centerY;
    float dz = worldZ - _envState.centerZ;

    // in "cell units"
    float cellX = dx / _envState.resolution + GRID_CENTER;
    float cellY = dy / _envState.resolution + GRID_CENTER;
    float cellZ = dz / _envState.resolution + GRID_CENTER;

    int ix = (int)roundf(cellX);
    int iy = (int)roundf(cellY);
    int iz = (int)roundf(cellZ);

    // 2) check bounds
    if (ix < 0 || ix >= GRID_SIZE ||
        iy < 0 || iy >= GRID_SIZE ||
        iz < 0 || iz >= GRID_SIZE)
    {
        // out of bounds
        return;
    }

    // 3) set occupancy
    _envState.occupancyGrid[ix][iy][iz] = occupied;
}

/*=============================================================================
 * 3b) Shift the grid if submarine goes out of bounds
 *============================================================================*/
bool MainController::shiftGridIfNeeded()
{
    // find sub's cell index
    float dx = _subState.posX - _envState.centerX;
    float dy = _subState.posY - _envState.centerY;
    float dz = _subState.posZ - _envState.centerZ;

    int ix = (int)roundf(dx / _envState.resolution) + GRID_CENTER;
    int iy = (int)roundf(dy / _envState.resolution) + GRID_CENTER;
    int iz = (int)roundf(dz / _envState.resolution) + GRID_CENTER;

    int shiftX = 0;
    int shiftY = 0;
    int shiftZ = 0;

    // We want sub's cell index to remain within [0..24].
    // If ix < 0 => shift + 1
    // If ix > 24 => shift - 1
    // But it's not just 1; we want to shift enough so that ix is back near center
    // We'll do a simpler approach: if ix < 2 => shift + (some cells)
    // if ix > 22 => shift - (some cells)
    // For example, we might shift 12 cells so sub gets back to the center
    int lowerBoundary = 2;
    int upperBoundary = GRID_SIZE - 3; // 22 if 25

    if (ix < lowerBoundary)
    {
        shiftX = (GRID_CENTER - ix); // so sub is near center again
    }
    else if (ix > upperBoundary)
    {
        shiftX = (GRID_CENTER - ix);
    }
    if (iy < lowerBoundary)
    {
        shiftY = (GRID_CENTER - iy);
    }
    else if (iy > upperBoundary)
    {
        shiftY = (GRID_CENTER - iy);
    }
    if (iz < lowerBoundary)
    {
        shiftZ = (GRID_CENTER - iz);
    }
    else if (iz > upperBoundary)
    {
        shiftZ = (GRID_CENTER - iz);
    }

    if (shiftX == 0 && shiftY == 0 && shiftZ == 0)
    {
        return false; // no shift needed
    }

    shiftGrid(shiftX, shiftY, shiftZ);

    // Also shift center in real-world coords
    float shiftWorldX = -(shiftX * _envState.resolution);
    float shiftWorldY = -(shiftY * _envState.resolution);
    float shiftWorldZ = -(shiftZ * _envState.resolution);

    _envState.centerX += shiftWorldX;
    _envState.centerY += shiftWorldY;
    _envState.centerZ += shiftWorldZ;

    return true;
}

void MainController::shiftGrid(int dx, int dy, int dz)
{
    // Shifting 3D array by dx,dy,dz
    // We'll create a temp array, fill it with false, then copy the old data
    // to the new location in the temp array, then copy back.

    static bool tempGrid[GRID_SIZE][GRID_SIZE][GRID_SIZE];

    memset(tempGrid, false, sizeof(tempGrid));

    for (int x = 0; x < GRID_SIZE; x++)
    {
        for (int y = 0; y < GRID_SIZE; y++)
        {
            for (int z = 0; z < GRID_SIZE; z++)
            {
                int nx = x + dx;
                int ny = y + dy;
                int nz = z + dz;
                if (nx >= 0 && nx < GRID_SIZE &&
                    ny >= 0 && ny < GRID_SIZE &&
                    nz >= 0 && nz < GRID_SIZE)
                {
                    tempGrid[nx][ny][nz] = _envState.occupancyGrid[x][y][z];
                }
            }
        }
    }

    // copy back
    memcpy(_envState.occupancyGrid, tempGrid, sizeof(tempGrid));
}

/*=============================================================================
 * 4) Path Logic
 *============================================================================*/
void MainController::adjustPath(PathData &path)
{
    // This is a placeholder for advanced path “pre-processing”
    // E.g. checking for immediate obstacles, etc.
    // For now, do nothing
}

void MainController::followPathAndProduceCommands(float dt)
{
    if (_currentPath.waypoints.empty())
    {
        // No path => maybe stop motors
        Esp32_3_Command cmd;
        cmd.esc1_us = 1500; // neutral
        cmd.esc2_us = 1500; // neutral
        cmd.pumpCmd = PumpFlowCmd::OFF;
        writeCommandToESP32_3(cmd);
        return;
    }

    // 1) Look at next waypoint
    Waypoint &wp = _currentPath.waypoints.front();
    float distX = wp.x - _subState.posX;
    float distY = wp.y - _subState.posY;
    float distZ = wp.z - _subState.posZ;
    float distSq = distX * distX + distY * distY + distZ * distZ;
    float threshold = 10.0f; // e.g. 10 cm

    if (distSq < (threshold * threshold))
    {
        // We consider we "reached" it
        _currentPath.waypoints.erase(_currentPath.waypoints.begin());
        if (_currentPath.waypoints.empty())
        {
            // done
            Esp32_3_Command cmd;
            cmd.esc1_us = 1500;
            cmd.esc2_us = 1500;
            cmd.pumpCmd = PumpFlowCmd::OFF;
            writeCommandToESP32_3(cmd);
            return;
        }
    }

    // Re-read front if we popped
    if (!_currentPath.waypoints.empty())
    {
        Waypoint &wpNext = _currentPath.waypoints.front();
        // Compute X-Y control
        float esc1_us = 1500, esc2_us = 1500;
        computeXYControl(wpNext.x, wpNext.y, dt, esc1_us, esc2_us);

        // Compute depth
        PumpFlowCmd pumpCmd = computeDepthCommand(wpNext.z, _subState.posZ, /*threshold=*/10.0f);

        // Send
        Esp32_3_Command cmd;
        cmd.esc1_us = (uint16_t)esc1_us;
        cmd.esc2_us = (uint16_t)esc2_us;
        cmd.pumpCmd = pumpCmd;
        writeCommandToESP32_3(cmd);
    }
}

/*=============================================================================
 * 5) XY PID
 *    We'll do a simple 2D approach: if we want to go from (posX,posY) to
 *    (desiredX,desiredY), we treat the difference as error. Then we produce
 *    a differential throttle for ESC1/ESC2.
 *============================================================================*/
void MainController::initPID(PID_Controller &pid, float kp, float ki, float kd)
{
    pid.kp = kp;
    pid.ki = ki;
    pid.kd = kd;
    pid.integratorX = 0.0f;
    pid.integratorY = 0.0f;
    pid.prevErrorX = 0.0f;
    pid.prevErrorY = 0.0f;
    pid.prevTimeMs = millis();
}

void MainController::resetPID(PID_Controller &pid)
{
    pid.integratorX = 0.0f;
    pid.integratorY = 0.0f;
    pid.prevErrorX = 0.0f;
    pid.prevErrorY = 0.0f;
    pid.prevTimeMs = millis();
}

void MainController::computeXYControl(float desiredX, float desiredY, float dt, float &esc1_us, float &esc2_us)
{
    // Error is difference
    float errX = desiredX - _subState.posX;
    float errY = desiredY - _subState.posY;

    // For simplicity, we do the same PID for X and Y (like a 2D approach)
    // ignoring heading. A more advanced approach would be to transform error
    // into forward-lateral relative to sub heading.

    // P
    float pX = _xyPid.kp * errX;
    float pY = _xyPid.kp * errY;

    // I
    _xyPid.integratorX += errX * dt;
    _xyPid.integratorY += errY * dt;
    float iX = _xyPid.ki * _xyPid.integratorX;
    float iY = _xyPid.ki * _xyPid.integratorY;

    // D
    float dX = _xyPid.kd * (errX - _xyPid.prevErrorX) / dt;
    float dY = _xyPid.kd * (errY - _xyPid.prevErrorY) / dt;

    float outX = pX + iX + dX;
    float outY = pY + iY + dY;

    _xyPid.prevErrorX = errX;
    _xyPid.prevErrorY = errY;

    // We'll interpret outX as forward/reverse throttle, outY as left/right differential
    // Then ESC1 = base + diff, ESC2 = base - diff.
    // "base" is 1500 (neutral). We'll do a simple scale so that max is ±500 => [1000..2000].
    float maxOutput = 500.0f;
    outX = clamp(outX, -maxOutput, maxOutput);
    outY = clamp(outY, -maxOutput, maxOutput);

    float base = 1500.0f;
    // We can combine X and Y in some manner
    // forward = outX => add to both ESC
    // rotate  = outY => add to one side, subtract from other

    float left = base + outX + outY;
    float right = base + outX - outY;

    // clamp
    left = clamp(left, 1000.0f, 2000.0f);
    right = clamp(right, 1000.0f, 2000.0f);

    esc1_us = left;
    esc2_us = right;
}

/*=============================================================================
 * 6) Depth Logic
 *============================================================================*/
PumpFlowCmd MainController::computeDepthCommand(float desiredZ, float currentZ, float threshold)
{
    float diff = desiredZ - currentZ;
    if (diff > threshold)
    {
        // sub is above desired => we want to go deeper => pump water in
        return PumpFlowCmd::IN;
    }
    else if (diff < -threshold)
    {
        // sub is below desired => we want to go shallower => pump water out
        return PumpFlowCmd::OUT;
    }
    // within threshold => do nothing
    return PumpFlowCmd::OFF;
}

/*=============================================================================
 * Utility
 *============================================================================*/
float MainController::clamp(float val, float minVal, float maxVal)
{
    if (val < minVal)
        return minVal;
    if (val > maxVal)
        return maxVal;
    return val;
}
