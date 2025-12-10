#include "I2CCommCam.h"

I2CCommCam::I2CCommCam(uint8_t camI2CAddress)
    : _addr(camI2CAddress)
{
}

void I2CCommCam::begin(int sdaPin, int sclPin, uint32_t frequency)
{
    Wire.begin(sdaPin, sclPin, frequency);
}

bool I2CCommCam::sendMainToCam(const MainToCamData &data)
{
    // We do a naive approach:
    // 1) Start transmission
    // 2) Write a small "header" indicating what data we are sending
    // 3) Write the entire structure as raw bytes
    // (In a real system, we'd consider chunking or partial sending if it's too large)

    // For reference, Submarine_State is ~sizeof(float)*14
    // Environment_State is 25*25*25 = 15625 bool => we might store them as bits or as bytes.
    // This is quite large. We do a basic approach here, but in practice we might do chunked transmissions.

    // Let’s do chunking if it's too big. We'll just do a single shot attempt here for demonstration.

    // We'll flatten the data into a buffer. We'll do the booleans as bytes for simplicity.

    // 1) Calculate total size:
    size_t subStateSize = sizeof(Submarine_State);
    size_t envHeaderSize = sizeof(data.envState.resolution) + 3 * sizeof(float);
    // that covers resolution + centerX/Y/Z
    size_t gridSize = (25 * 25 * 25); // store as 1 byte each
    size_t totalSize = 1 + subStateSize + envHeaderSize + gridSize;
    // +1 if we want a 'message ID' or something

    // If total size > I2C buffer limit (esp32 default might be 128~512 bytes), this won't fully work,
    // but we proceed for demonstration.

    // 2) Build the buffer
    uint8_t *buf = (uint8_t *)malloc(totalSize);
    if (!buf)
        return false;

    // We'll define first byte as e.g. 0x01 indicating "MainToCamData"
    buf[0] = 0x01;
    size_t offset = 1;

    // copy subState
    memcpy(buf + offset, &data.subState, subStateSize);
    offset += subStateSize;

    // copy envHeader (resolution, centerX, centerY, centerZ)
    memcpy(buf + offset, &data.envState.resolution, sizeof(float));
    offset += sizeof(float);
    memcpy(buf + offset, &data.envState.centerX, sizeof(float));
    offset += sizeof(float);
    memcpy(buf + offset, &data.envState.centerY, sizeof(float));
    offset += sizeof(float);
    memcpy(buf + offset, &data.envState.centerZ, sizeof(float));
    offset += sizeof(float);

    // copy grid
    for (int x = 0; x < 25; x++)
    {
        for (int y = 0; y < 25; y++)
        {
            for (int z = 0; z < 25; z++)
            {
                // store as 0x00 or 0x01
                buf[offset++] = data.envState.occupancyGrid[x][y][z] ? 0x01 : 0x00;
            }
        }
    }

    // 3) Write via I2C
    Wire.beginTransmission(_addr);
    size_t written = Wire.write(buf, totalSize);
    uint8_t error = Wire.endTransmission();

    free(buf);

    if ((written == totalSize) && (error == 0))
    {
        return true;
    }
    else
    {
        return false;
    }
}

bool I2CCommCam::getPathData(PathData &path)
{
    // We'll do a naive approach: we send a request, then read up to e.g. 128 bytes
    // The cam might store a path with up to e.g. 5 waypoints.
    // We'll define: first byte is "0x02" => PathData, second byte is how many waypoints,
    // then each waypoint is 3 floats, then a bool isNew

    // 1) Request
    Wire.beginTransmission(_addr);
    Wire.write(0x02);            // means "request path"
    Wire.endTransmission(false); // do repeated start

    // 2) Read
    // We guess the maximum possible size, e.g. 1 + 1 + 5*(3 floats) + 1 = 1 +1 +5*12 +1=63
    // We'll do a 64 read
    size_t toRead = 64;
    uint8_t bytesRead = Wire.requestFrom((int)_addr, (int)toRead, (int)true);

    if (bytesRead < 2)
    {
        return false; // not enough data
    }

    // read them into a local buffer
    uint8_t buf[64];
    for (int i = 0; i < bytesRead; i++)
    {
        buf[i] = Wire.read();
    }

    // parse
    if (buf[0] != 0x02)
    {
        // not path data
        return false;
    }
    uint8_t numWaypoints = buf[1];
    if (numWaypoints > 20)
        numWaypoints = 20; // just clamp

    path.waypoints.clear();
    size_t offset = 2;
    for (uint8_t i = 0; i < numWaypoints; i++)
    {
        if (offset + 12 <= bytesRead)
        {
            Waypoint wp;
            memcpy(&wp.x, &buf[offset], 4);
            offset += 4;
            memcpy(&wp.y, &buf[offset], 4);
            offset += 4;
            memcpy(&wp.z, &buf[offset], 4);
            offset += 4;
            path.waypoints.push_back(wp);
        }
        else
        {
            break;
        }
    }
    if (offset < bytesRead)
    {
        // read isNew
        bool isNew = (buf[offset] != 0);
        path.isNew = isNew;
    }
    else
    {
        path.isNew = false;
    }
    return true;
}
