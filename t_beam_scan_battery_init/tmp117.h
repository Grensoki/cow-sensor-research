#pragma once

#include <Arduino.h>
#include <Wire.h>

class TMP117
{
private: 
    static constexpr uint8_t REG_TEMP   = 0x00;
    static constexpr uint8_t REG_CONFIG = 0x01;
    static constexpr uint8_t REG_ID     = 0x0F;

    TwoWire &_wire;
    uint8_t _address;

    bool _initialized;
    uint16_t _deviceID;

    bool writeRegister(uint8_t reg, uint16_t value);
    bool readRegister(uint8_t reg, uint16_t &value);

public:
    explicit TMP117(
        TwoWire &wire = Wire, 
        uint8_t address = 0x48
    );

    bool begin(uint16_t config = 0x0220);
    bool readTemp(float& tempF);
    bool isInitialized() const;
    uint16_t getDeviceID() const;
};