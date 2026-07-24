#include "tmp117.h"

TMP117::TMP117(
    TwoWire &wire,
    uint8_t address
)
    :   _wire(wire),
        _address(address),
        _initialized(false),
        _deviceID(0)
{
}

bool TMP117::writeRegister(uint8_t reg, uint16_t value){
    _wire.beginTransmission(_address);
    _wire.write(reg);
    _wire.write((uint8_t)(value >> 8));        // MSB
    _wire.write((uint8_t)(value & 0xFF));      // LSB

    uint8_t error = _wire.endTransmission();

    if(error != 0){
        Serial.print("TMP117 write failed at register 0x");
        Serial.print(reg, HEX);
        Serial.print(", I2C error: ");
        Serial.println(error);
        return false;
    }

    return true;
}

bool TMP117::readRegister(uint8_t reg, uint16_t& value){
    _wire.beginTransmission(_address);
    _wire.write(reg);

    uint8_t error = _wire.endTransmission(false);

    if (error != 0) {
        Serial.print("TMP117 register pointer failed at 0x");
        Serial.print(reg, HEX);
        Serial.print(", I2C error: ");
        Serial.println(error);
        return false;
    }

    uint8_t received = _wire.requestFrom((uint8_t)_address, (uint8_t)2);

    if(received != 2){
        Serial.print("TMP117 register 0x");
        Serial.print(reg, HEX);
        Serial.print(": expected 2 bytes, received ");
        Serial.println(received);

        return false;
    }

    uint8_t msb = _wire.read();
    uint8_t lsb = _wire.read();

    value = ((uint16_t)msb << 8) | lsb;

    return true;
}

bool TMP117::begin(uint16_t config){
    _initialized = false;
    
    if(!readRegister(REG_ID, _deviceID)){
        Serial.println("Could not read TMP117 device ID.");
        return false;
    }

    Serial.print("TMP117 device ID register: 0x");
    Serial.println(_deviceID, HEX);

    if((_deviceID & 0x0FFF) != 0x0117){
        Serial.println("Unexpected TMP117 device ID.");
        return false;
    }

    if(!writeRegister(REG_CONFIG, config)){
        Serial.println("Failed to configure TMP117.");
        return false;
    }
    //allow first conversion to begin
    delay(1100);  

    float tempF;
    if(!readTemp(tempF)){
        Serial.println("TMP117 initial temperature read failed.");
        return false;
    }

    _initialized = true;
    return true;

}

bool TMP117::readTemp(float &tempF){
    uint16_t value;

    if(!readRegister(REG_TEMP, value)){
        Serial.println("Failed to read temperature at register 0x00");
        return false;
    }

    if (value == 0x8000) {
        Serial.println("TMP117 temperature conversion not ready");
        return false;
    }

    int16_t raw = static_cast<int16_t>(value);
    float tempC = raw / 128.0f;
    tempF = (tempC * 1.8f) + 32.0f;

    return true;
}

bool TMP117::isInitialized() const{
    return _initialized;
}

uint16_t TMP117::getDeviceID() const{
    return _deviceID;
}


