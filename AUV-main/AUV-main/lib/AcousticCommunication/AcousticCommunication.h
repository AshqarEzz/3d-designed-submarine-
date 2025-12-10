#ifndef ACOUSTIC_COMMUNICATION_H
#define ACOUSTIC_COMMUNICATION_H

#include <Arduino.h>
#include <vector>

// We'll define a minimal "packet" approach for demonstration
// In real life you'd have advanced modulation, FSK/OFDM, error correction, etc.

class AcousticCommunication
{
public:
    AcousticCommunication();
    bool begin(int inputPin, int outputPin);

    bool sendData(const uint8_t *data, size_t length);
    // Non-blocking; returns immediately. Real code might generate tones asynchronously.

    // For listening, we might call:
    bool available();                             // returns true if there's a new packet
    size_t readData(uint8_t *dst, size_t maxLen); // read next packet

    // ... more advanced methods as needed

private:
    int _inPin;
    int _outPin;
    bool _initialized;

    // We keep an internal buffer for the sake of demonstration
    std::vector<uint8_t> _rxBuffer;

    // placeholder for advanced DSP logic
    void processInput();
};

#endif
