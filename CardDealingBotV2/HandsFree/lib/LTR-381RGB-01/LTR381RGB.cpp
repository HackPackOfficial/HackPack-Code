#include "LTR381RGB.h"

// Constructor
LTR381RGB::LTR381RGB(uint8_t address)
{
  _address = address;
  _wire = &Wire; // default to primary I2C bus
}

// Initialize sensor
bool LTR381RGB::begin()
{
  _wire = &Wire;
  _wire->begin();

  // Check PART_ID register (0x06 should return 0xC2)
  uint8_t partID = read8(LTR381RGB_PART_ID);
  if (partID != 0xC2)
  {
    return false; // wrong sensor or not connected
  }

  // Enable CS mode (RGB+IR active)
  enable();

  // Default config: resolution = 18-bit, rate = 100ms (datasheet default 0x22)
  write8(LTR381RGB_MEAS_RATE, 0x22);

  // Default gain = 3x
  write8(LTR381RGB_GAIN, 0x01);

  return true;
}

// Enable the sensor (active mode, CS mode)
void LTR381RGB::enable()
{
  // MAIN_CTRL: 0x00
  // Bit 2 = CS mode (1)
  // Bit 1 = ALS/CS Enable (1)
  // Value = 0b00000110 = 0x06
  write8(LTR381RGB_MAIN_CTRL, 0x06);
}

// Disable the sensor (standby)
void LTR381RGB::disable()
{
  write8(LTR381RGB_MAIN_CTRL, 0x00);
}

// Set analog gain (per datasheet table, 0x00–0x04 valid)
void LTR381RGB::setGain(uint8_t gain)
{
  if (gain > 0x04)
    gain = 0x04;
  write8(LTR381RGB_GAIN, gain);
}

// Set resolution and measurement rate
// resolutionBits = 0–4 (maps to 20,19,18,17,16-bit modes)
// rateCode = 0–6 (maps to 25ms … 2000ms)
void LTR381RGB::setResolutionAndRate(uint8_t resolutionBits, uint8_t rateCode)
{
  uint8_t value = ((resolutionBits & 0x07) << 4) | (rateCode & 0x07);
  write8(LTR381RGB_MEAS_RATE, value);
}

// Get raw RGB + IR data
// "w" will return the IR channel (no true white channel available)
void LTR381RGB::getRawData(uint16_t *r, uint16_t *g, uint16_t *b, uint16_t *w)
{
  uint32_t red = read20(LTR381RGB_DATA_RED);
  uint32_t green = read20(LTR381RGB_DATA_GREEN);
  uint32_t blue = read20(LTR381RGB_DATA_BLUE);
  uint32_t ir = read20(LTR381RGB_DATA_IR);

  // Downscale 20-bit data to 16-bit
  *r = (uint16_t)(red >> 4);
  *g = (uint16_t)(green >> 4);
  *b = (uint16_t)(blue >> 4);

  // Simulate a "white" channel based on luminance
  uint32_t whiteSim = (uint32_t)(0.299f * (*r) + 0.587f * (*g) + 0.114f * (*b));
  if (whiteSim > 65535)
    whiteSim = 65535; // cap for safety
  *w = (uint16_t)whiteSim;
}

// ---- Low-level I2C helpers ----

void LTR381RGB::write8(uint8_t reg, uint8_t value)
{
  _wire->beginTransmission(_address);
  _wire->write(reg);
  _wire->write(value);
  _wire->endTransmission();
}

uint8_t LTR381RGB::read8(uint8_t reg)
{
  _wire->beginTransmission(_address);
  _wire->write(reg);
  _wire->endTransmission();

  _wire->requestFrom(_address, (uint8_t)1);
  if (_wire->available())
  {
    return _wire->read();
  }
  return 0;
}

// Read 20-bit ADC channel (3 registers, little endian)
uint32_t LTR381RGB::read20(uint8_t regBase)
{
  _wire->beginTransmission(_address);
  _wire->write(regBase);
  _wire->endTransmission();

  _wire->requestFrom(_address, (uint8_t)3);

  uint32_t data = 0;
  if (_wire->available() >= 3)
  {
    uint8_t low = _wire->read(); // LSB
    uint8_t mid = _wire->read();
    uint8_t high = _wire->read() & 0x0F; // upper 4 bits valid

    data = ((uint32_t)high << 16) | ((uint32_t)mid << 8) | low;
  }
  return data;
}