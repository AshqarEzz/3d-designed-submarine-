#include "UARTComm.h"

static uint16_t crc16_update(uint16_t crc, uint8_t data)
{
    crc ^= (uint16_t)data << 8;
    for (int i = 0; i < 8; i++)
    {
        if (crc & 0x8000)
            crc = (crc << 1) ^ 0x1021;
        else
            crc = (crc << 1);
    }
    return crc;
}
static uint16_t computeCRC(const uint8_t *data, size_t length)
{
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < length; i++)
    {
        crc = crc16_update(crc, data[i]);
    }
    return crc;
}

UARTComm::UARTComm(HardwareSerial &serial, uint32_t baudRate)
    : _serial(serial), _baudRate(baudRate),
      _packetHandler(nullptr), _payloadBuf(nullptr),
      _crcBuffer(nullptr)
{
    _payloadBufSize = 0;
    resetParser();
}

void UARTComm::begin()
{
    _serial.begin(_baudRate);
    // Allocate some default buffer
    expandPayloadBuffer(256);
    _crcBuffer = (uint8_t *)malloc(256);
}

void UARTComm::update()
{
    while (_serial.available() > 0)
    {
        uint8_t b = (uint8_t)_serial.read();
        handleByte(b);
    }
}

bool UARTComm::sendPacket(UARTMsgType msgType, const uint8_t *payload, uint16_t length)
{
    // Construct the framing:
    // [start(1)][msgType(1)][length(2)][payload(n)][crc(2)][end(1)]

    uint8_t start = START_BYTE;
    uint8_t end = END_BYTE;
    uint8_t type = (uint8_t)msgType;

    // compute CRC over (msgType + length + payload)
    // first build a small buffer to pass to computeCRC
    uint8_t *tmpBuf = (uint8_t *)malloc(length + 3);
    tmpBuf[0] = type;
    tmpBuf[1] = (uint8_t)((length >> 8) & 0xFF);
    tmpBuf[2] = (uint8_t)(length & 0xFF);
    memcpy(&tmpBuf[3], payload, length);

    uint16_t crc = computeCRC(tmpBuf, length + 3);
    free(tmpBuf);

    // Now send
    _serial.write(start);
    _serial.write(type);
    // length in big-endian or little-endian (decide). We'll do big-endian here:
    _serial.write((uint8_t)((length >> 8) & 0xFF));
    _serial.write((uint8_t)(length & 0xFF));
    // payload
    _serial.write(payload, length);
    // crc (big-endian or little, must match your parser)
    _serial.write((uint8_t)((crc >> 8) & 0xFF));
    _serial.write((uint8_t)(crc & 0xFF));
    // end
    _serial.write(end);

    return true;
}

void UARTComm::onPacketReceived(PacketHandlerCallback cb)
{
    _packetHandler = cb;
}

//-------------------------------
// Private parser methods
//-------------------------------
void UARTComm::resetParser()
{
    _state = ParserState::WAIT_START;
    _currentMsgType = (UARTMsgType)0;
    _currentLength = 0;
    _bytesReadForPayload = 0;
    _currentCRC = 0xFFFF; // for usage if needed
    _crcBufferPos = 0;
}

void UARTComm::handleByte(uint8_t b)
{
    switch (_state)
    {
    case ParserState::WAIT_START:
        if (b == START_BYTE)
        {
            resetParser(); // ensure clean
            _state = ParserState::WAIT_MSGTYPE;
        }
        break;

    case ParserState::WAIT_MSGTYPE:
        _currentMsgType = (UARTMsgType)b;
        // The first byte for CRC is the msgType
        if (_crcBuffer)
            _crcBuffer[_crcBufferPos++] = b;
        _state = ParserState::WAIT_LENGTH_1;
        break;

    case ParserState::WAIT_LENGTH_1:
        if (_crcBuffer)
            _crcBuffer[_crcBufferPos++] = b;
        _currentLength = ((uint16_t)b << 8) & 0xFF00;
        _state = ParserState::WAIT_LENGTH_2;
        break;

    case ParserState::WAIT_LENGTH_2:
    {
        if (_crcBuffer)
            _crcBuffer[_crcBufferPos++] = b;
        _currentLength |= (uint16_t)b & 0x00FF;
        _bytesReadForPayload = 0;
        // Check buffer sizes
        expandPayloadBuffer(_currentLength);
        if (_crcBuffer && _crcBufferPos + _currentLength > 256)
        {
            // re-allocate bigger if needed
            uint8_t *newBuf = (uint8_t *)realloc(_crcBuffer, _crcBufferPos + _currentLength + 2 /*crc*/);
            if (newBuf)
                _crcBuffer = newBuf;
        }
        _state = (_currentLength == 0) ? ParserState::WAIT_CRC_1 : ParserState::WAIT_PAYLOAD;
    }
    break;

    case ParserState::WAIT_PAYLOAD:
    {
        // store into payload
        _payloadBuf[_bytesReadForPayload] = b;
        // also store into _crcBuffer
        if (_crcBuffer)
            _crcBuffer[_crcBufferPos++] = b;

        _bytesReadForPayload++;
        if (_bytesReadForPayload >= _currentLength)
        {
            _state = ParserState::WAIT_CRC_1;
        }
        break;
    }

    case ParserState::WAIT_CRC_1:
        // We'll store the high byte of CRC in a temporary
        _currentCRC = ((uint16_t)b << 8) & 0xFF00;
        _state = ParserState::WAIT_CRC_2;
        break;

    case ParserState::WAIT_CRC_2:
    {
        _currentCRC |= (uint16_t)b & 0x00FF;
        _state = ParserState::WAIT_END;
    }
    break;

    case ParserState::WAIT_END:
        if (b == END_BYTE)
        {
            // Now we check if the CRC is correct
            uint16_t computed = computeCRC(_crcBuffer, _crcBufferPos);
            if (computed == _currentCRC)
            {
                // valid packet
                if (_packetHandler)
                {
                    UARTPacket pkt;
                    pkt.msgType = _currentMsgType;
                    pkt.length = _currentLength;
                    pkt.payload = _payloadBuf; // pointer
                    _packetHandler(pkt);
                }
            }
        }
        // Whether good or not, reset
        resetParser();
        break;
    }
}

void UARTComm::expandPayloadBuffer(uint16_t neededSize)
{
    if (neededSize <= _payloadBufSize)
        return;
    uint8_t *newBuf = (uint8_t *)realloc(_payloadBuf, neededSize);
    if (newBuf)
    {
        _payloadBuf = newBuf;
        _payloadBufSize = neededSize;
    }
    else
    {
        // Allocation failed => handle error
        // We can keep old buffer, but we won't parse properly
        // In practice, handle gracefully or do a safe fallback
    }
}
