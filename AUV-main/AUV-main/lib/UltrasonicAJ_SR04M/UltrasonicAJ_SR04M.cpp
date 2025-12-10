#include "UltrasonicAJ_SR04M.h"

UltrasonicAJ_SR04M::UltrasonicAJ_SR04M()
    : _initialized(false)
{
    _data.distanceCm = 0.0f;
}

bool UltrasonicAJ_SR04M::begin(const UltrasonicAJ_SR04M_Config &config)
{
    _config = config;
    pinMode(_config.trigPin, OUTPUT);
    pinMode(_config.echoPin, INPUT);
    _initialized = true;
    return true;
}

bool UltrasonicAJ_SR04M::readDistance()
{
    if (!_initialized)
        return false;

    // Trigger the sensor
    digitalWrite(_config.trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(_config.trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(_config.trigPin, LOW);

    // Read echo
    long duration = pulseIn(_config.echoPin, HIGH);

    // Convert duration to distance
    float distance = duration * _config.speedOfSoundCmPerUs / 2.0f;
    if (distance > _config.maxDistanceCm)
    {
        // You can decide to clamp or mark invalid
        distance = _config.maxDistanceCm;
    }

    _data.distanceCm = distance;
    return true;
}

UltrasonicAJ_SR04M_Data UltrasonicAJ_SR04M::getData() const
{
    return _data;
}
