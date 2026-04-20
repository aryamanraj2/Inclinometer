// qmc5883p.h
#pragma once
#include <Arduino.h>
#include <Wire.h>

class QMC5883P {
public:
    // Constructor: optional I2C address & bus
    QMC5883P(uint8_t addr = 0x2C, TwoWire &bus = Wire);

    bool begin();                              // Init, true = OK
    bool readXYZ(float *xyz);                  // xyz[3] → µT, true = new data (calibrated)
    float getHeadingDeg(float declDeg = 0.0f); // Heading with internal data caching
    void setHardIronOffsets(float xOff, float yOff, float zOff = 0.0f);
    void setSoftIronScales(float scaleX, float scaleY, float scaleZ = 1.0f);
    bool writeReg(uint8_t reg, uint8_t val);   // Public for low-level config access

private:
    uint8_t _addr;
    TwoWire *_bus;

    // Calibration parameters
    float _offX, _offY, _offZ;
    float _scaleX, _scaleY, _scaleZ;

    // Raw measurement cache
    int16_t _lastRawX, _lastRawY, _lastRawZ;
    unsigned long _lastReadTime;
    static const unsigned long _minInterval = 5; // min ms between reads

    bool readReg(uint8_t reg, uint8_t *buf, uint8_t len);
    bool readRaw();
};
