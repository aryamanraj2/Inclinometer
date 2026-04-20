// qmc5883p.cpp — ESP32-patched version
// Fix: endTransmission(true) instead of endTransmission(false) in readReg()
// to avoid ESP32 Wire repeated-start bug (i2cWriteReadNonStop Error -1)

#include "qmc5883p.h"
#include <Arduino.h>

/* Register addresses */
#define REG_CHIP_ID        0x00
#define REG_DATA_OUT_X_LSB 0x01
#define REG_STATUS         0x09
#define REG_CTL1           0x0A
#define REG_CTL2           0x0B

QMC5883P::QMC5883P(uint8_t addr, TwoWire &bus)
    : _addr(addr), _bus(&bus),
      _offX(0.0f), _offY(0.0f), _offZ(0.0f),
      _scaleX(1.0f), _scaleY(1.0f), _scaleZ(1.0f),
      _lastRawX(0), _lastRawY(0), _lastRawZ(0),
      _lastReadTime(0) {}

bool QMC5883P::begin() {
    // Note: Wire.begin() should already be called in the main sketch
    // with specific SDA/SCL pins before calling this.
    // We do NOT call _bus->begin() here to avoid re-initializing with
    // default pins and losing the user's pin configuration.

    uint8_t id;
    if (!readReg(REG_CHIP_ID, &id, 1) || id != 0x80) return false;

    // Default config: 200 Hz, Continuous, ±2G
    writeReg(0x0D, 0x40); delay(10);
    writeReg(0x29, 0x06); delay(10);
    writeReg(REG_CTL1, 0xCF); delay(10);
    writeReg(REG_CTL2, 0x00); delay(10);

    _lastReadTime = 0;
    return true;
}

bool QMC5883P::readRaw() {
    unsigned long now = millis();
    if (now - _lastReadTime < _minInterval) {
        return false;  // Not enough time elapsed, use cache
    }

    uint8_t status;
    if (!readReg(REG_STATUS, &status, 1) || !(status & 0x01)) {
        return false;  // No new data ready
    }

    uint8_t buf[6];
    if (!readReg(REG_DATA_OUT_X_LSB, buf, 6)) return false;

    _lastRawX = int16_t(buf[1] << 8 | buf[0]);
    _lastRawY = int16_t(buf[3] << 8 | buf[2]);
    _lastRawZ = int16_t(buf[5] << 8 | buf[4]);
    _lastReadTime = now;
    return true;
}

bool QMC5883P::readXYZ(float *xyz) {
    if (!readRaw()) return false;

    // Raw → µT with calibration (Hard + Soft Iron)
    float x = _lastRawX / 1000.0f;
    float y = _lastRawY / 1000.0f;
    float z = _lastRawZ / 1000.0f;

    xyz[0] = (x - _offX) * _scaleX;
    xyz[1] = (y - _offY) * _scaleY;
    xyz[2] = (z - _offZ) * _scaleZ;
    return true;
}

float QMC5883P::getHeadingDeg(float declDeg) {
    readRaw();  // Try to read new data, otherwise use cache

    float x = (_lastRawX / 1000.0f - _offX) * _scaleX;
    float y = (_lastRawY / 1000.0f - _offY) * _scaleY;

    float hdg = atan2(y, x);
    hdg += declDeg * DEG_TO_RAD;
    if (hdg < 0)        hdg += TWO_PI;
    else if (hdg > TWO_PI) hdg -= TWO_PI;
    return hdg * RAD_TO_DEG;
}

void QMC5883P::setHardIronOffsets(float xOff, float yOff, float zOff) {
    _offX = xOff;
    _offY = yOff;
    _offZ = zOff;
}

void QMC5883P::setSoftIronScales(float scaleX, float scaleY, float scaleZ) {
    _scaleX = scaleX;
    _scaleY = scaleY;
    _scaleZ = scaleZ;
}

// ============================================================
// ESP32 FIX: Use endTransmission(true) to send STOP condition
// before requestFrom(). The original library uses
// endTransmission(false) which triggers a repeated-start,
// causing "i2cWriteReadNonStop Error -1" on ESP32.
// ============================================================
bool QMC5883P::readReg(uint8_t reg, uint8_t *buf, uint8_t len) {
    _bus->beginTransmission(_addr);
    _bus->write(reg);
    if (_bus->endTransmission(true) != 0) return false;  // STOP (not repeated start)
    if (_bus->requestFrom(_addr, len) != len) return false;
    for (uint8_t i = 0; i < len; i++) buf[i] = _bus->read();
    return true;
}

bool QMC5883P::writeReg(uint8_t reg, uint8_t val) {
    _bus->beginTransmission(_addr);
    _bus->write(reg);
    _bus->write(val);
    return (_bus->endTransmission() == 0);
}
