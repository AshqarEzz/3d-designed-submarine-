#ifndef ULTRASONIC_AJ_SR04M_H
#define ULTRASONIC_AJ_SR04M_H

#include <Arduino.h>

struct UltrasonicAJ_SR04M_Config
{
    int trigPin;
    int echoPin;
    // Speed of sound in cm/us (roughly 0.0343 cm/us at sea level), can be adjusted
    float speedOfSoundCmPerUs = 0.0343f;
    // In case you want a max distance cutoff in cm
    unsigned long maxDistanceCm = 400;
};

struct UltrasonicAJ_SR04M_Data
{
    // Measured distance in cm
    float distanceCm;
};

class UltrasonicAJ_SR04M
{
public:
    UltrasonicAJ_SR04M();
    bool begin(const UltrasonicAJ_SR04M_Config &config);
    // readDistance() triggers a measurement and updates internal data
    bool readDistance();
    UltrasonicAJ_SR04M_Data getData() const;

private:
    UltrasonicAJ_SR04M_Config _config;
    UltrasonicAJ_SR04M_Data _data;
    bool _initialized;
};

#endif
