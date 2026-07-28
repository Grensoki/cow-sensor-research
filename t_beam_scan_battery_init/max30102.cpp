#include "max30102.h"
#include "heartRate.h"

HRSensor::HRSensor(TwoWire& wire)
    :   _wire(wire),
        _rateSpot(0),
        _beatCount(0),
        _lastBeat(0),
        _irValue(0),
        _beatsPerMinute(0),
        _beatAverage(0)
{
    for(byte i = 0; i < RATE_SIZE; i++){
        _rates[i] = 0;
    }
}

bool HRSensor::begin(){
    if(!(_sensor.begin(_wire, I2C_SPEED_FAST))){
        return false;
    }
    byte ledBrightness = 0x3F;
    byte sampleAverage = 8;
    byte ledMode = 2;
    int sampleRate = 400;
    int pulseWidth = 411;
    int adcRange = 8192;

    _sensor.setup(
        ledBrightness,
        sampleAverage,
        ledMode,
        sampleRate,
        pulseWidth,
        adcRange);

    _sensor.setPulseAmplitudeIR(0x3F);
    _sensor.setPulseAmplitudeRed(0x0A);
    _sensor.clearFIFO();

    return true;
}

bool HRSensor::update(){
    _irValue = _sensor.getIR();
    if(checkForBeat(_irValue)){
        unsigned long now = millis();

        if(_lastBeat != 0){
            unsigned long delta = now - _lastBeat;
            float newBPM = 60000.0f / delta;

            if(newBPM > 30.0f && newBPM < 255.0f){
                 _beatsPerMinute = newBPM;

                 _rates[_rateSpot++] = static_cast<byte>(_beatsPerMinute);
                 _rateSpot %= RATE_SIZE;

                 if(_beatCount < RATE_SIZE){
                    _beatCount++;
                 }
                 int total = 0;
                 for(byte i = 0; i < _beatCount; i++){
                    total += _rates[i];
                 }
                 _beatAverage = total / _beatCount;
            }
        }
        _lastBeat = now;
    }

    if(_irValue < 5000){
        for(byte i = 0; i < RATE_SIZE; i++){
            _rates[i] = 0;
        }
        _rateSpot = 0;
        _beatCount = 0;
        _lastBeat = 0;
        _beatsPerMinute = 0.0f;
        _beatAverage = 0;
        return false;
    }
    return true;

}

float HRSensor::getBPM() const
{
    return _beatsPerMinute;
}

int HRSensor::getAverageBPM() const
{
    return _beatAverage;
}

long HRSensor::getIRValue() const
{
    return _irValue;
}

bool HRSensor::fingerDetected() const
{
    return _irValue >= 5000;
}