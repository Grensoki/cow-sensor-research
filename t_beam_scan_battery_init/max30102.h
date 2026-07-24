#pragma once

#include <Wire.h>
#include "MAX30105.h"



class HRSensor
{
public:
    explicit HRSensor(TwoWire &wire = Wire);
    bool begin();
    bool update();

    float getBPM() const;
    int getAverageBPM() const;
    long getIRValue() const;
    bool fingerDetected() const;

private: 
    static constexpr byte RATE_SIZE = 10;
    MAX30105 _sensor;
    TwoWire &_wire;

    byte _rates[RATE_SIZE];
    byte _rateSpot;
    byte _rateCount;
    byte _beatCount;

    unsigned long _lastBeat;
    long _irValue;
    float _beatsPerMinute;
    int _beatAverage;
};