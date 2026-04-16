#include "CLS16D24.h"

CLS16D24::CLS16D24() {}

void CLS16D24::reset() {
    writeRegister(SYSM_CTRL, 0x01); // Software reset
    delay(10);
}

void CLS16D24::enable() {
    writeRegister(SYSM_CTRL, 0x03); // Enable CLS function
}

void CLS16D24::setGain(uint8_t gain, bool doubleSensorArea = true) {
    switch (gain) {
        case 1: GAIN_BYTE = 0x01; break;
        case 4: GAIN_BYTE = 0x02; break;
        case 8: GAIN_BYTE = 0x04; break;
        case 32: GAIN_BYTE = 0x08; break;
        case 96: GAIN_BYTE = 0x10; break;
        default: GAIN_BYTE = 0x10; break;        // defaults to gain of 96
    }
    if (doubleSensorArea) {
        GAIN_BYTE |= 0x80;
    } else {
        GAIN_BYTE &= 0x7F;
    }
    writeRegister(CLS_GAIN, GAIN_BYTE);
}

void CLS16D24::setResolutionAndConversionTime(uint8_t time) {
    CONV_TIME = time;
    INT_TIME = intPow(4, (CONV_TIME & 0x03));
    CLS_CONV = CONV_TIME >> 4;
    writeRegister(CLS_TIME, CONV_TIME);
}

void CLS16D24::readRGBWIR(uint16_t &r, uint16_t &g, uint16_t &b, uint16_t &w, uint16_t &ir) {
    r = readRegister16(RCH_DATA_L);
    g = readRegister16(GCH_DATA_L);
    b = readRegister16(BCH_DATA_L);
    w = readRegister16(WCH_DATA_L);
    ir = readRegister16(IRCH_DATA_L);
}

void CLS16D24::writeRegister(uint8_t reg, uint8_t value) {
    wire->beginTransmission(CLS16D24_ADDRESS);
    wire->write(reg);
    wire->write(value);
    wire->endTransmission();
}

uint16_t CLS16D24::readRegister16(uint8_t reg) {
    wire->beginTransmission(CLS16D24_ADDRESS);
    wire->write(reg);
    wire->endTransmission(false);
    wire->requestFrom(CLS16D24_ADDRESS, (uint8_t)2);
    uint16_t value = wire->read();
    value |= (wire->read() << 8);
    return value;
}

float CLS16D24::getConversionTimeMillis() {       // calculates and returns conversion time in milliseconds for color data reading based on the value set by setResolutionAndConversionTime
    // TCLS= 3.827+ [ INT_TIME x (CLSCONV + 1) ] (ms)
    return 3.827 + ((2.0667 * (float)INT_TIME) * (1 + CLS_CONV));
}


uint16_t CLS16D24::getResolution() {           // calculates and returns data resolution set by setResolutionAndConversionTime
    // minimum of either 65535 or (1024 * ((INT_TIME time units) * (CLS_CONV +1)) - 1    . In this case, INT_TIME is the time units I calculated below (1, 4, 16, or 64).
    return min(65535, (1024 * INT_TIME * (CLS_CONV + 1)) - 1);
}

inline uint16_t CLS16D24::intPow(uint16_t base, uint16_t exp) {
    uint16_t result = 1;
    while (exp > 0) {
        if (exp & 1) {  // If exponent is odd, multiply the result
            result *= base;
        }
        base *= base;   // Square the base
        exp >>= 1;      // Divide exponent by 2
    }
    return result;
}


uint8_t CLS16D24::getGain() {
    switch (GAIN_BYTE & 0x1F) {
        case 0x01: return 1;
        case 0x02: return 4;
        case 0x04: return 8;
        case 0x08: return 32;
        case 0x10: return 96;
        default: return 1; // Default case (failsafe)
    }
}
