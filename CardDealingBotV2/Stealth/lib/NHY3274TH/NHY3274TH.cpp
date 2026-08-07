#include "NHY3274TH.h"

void NHY3274TH::setIntegrationTime(uint8_t integrationTime)
{
  if (integrationTime > 0x3)
  {
    integrationTime = 0x3; // Ensure the value is within bounds (0x0 to 0x3)
  }
  // Write the integration time to the CLS_TIME register (register 0x05)
  write8(0x05, integrationTime);
}

void NHY3274TH::setGain(uint8_t gain)
{
  if (gain > 0x20)
    gain = 0x20;                    // Ensure gain is within valid range
  write8(NHY3274TH_CLS_GAIN, gain); // Write gain value to the gain register
}

// Constructor to initialize the address and Wire object
NHY3274TH::NHY3274TH(uint8_t address)
{
  _address = address;
  _wire = &Wire;
}

// Begin function to initialize I2C communication and sensor
boolean NHY3274TH::begin(TwoWire *theWire)
{
  _wire = theWire;
  _wire->begin();

  // Check if the sensor responds at the given address
  _wire->beginTransmission(_address);
  if (_wire->endTransmission() != 0)
  {
    return false; // Sensor did not respond
  }

  // Enable the sensor by writing to the enable register
  enable();

  return true;
}

// Enable the sensor (power on and enable measurement)
void NHY3274TH::enable()
{
  write8(NHY3274TH_ENABLE, 0x03); // Enable RGB and power on
}

// Disable the sensor (power down)
void NHY3274TH::disable()
{
  write8(NHY3274TH_ENABLE, 0x00); // Power down
}

// Function to get raw RGB and white data
void NHY3274TH::getRawData(uint16_t *r, uint16_t *g, uint16_t *b, uint16_t *w)
{
  *r = read16(NHY3274TH_RDATA_L);
  *g = read16(NHY3274TH_GDATA_L);
  *b = read16(NHY3274TH_BDATA_L);
  *w = read16(NHY3274TH_WDATA_L);
}

// Function to write an 8-bit value to a register
void NHY3274TH::write8(uint8_t reg, uint8_t value)
{
  _wire->beginTransmission(_address);
  _wire->write(reg);
  _wire->write(value);
  _wire->endTransmission();
}

// Function to read an 8-bit value from a register
uint8_t NHY3274TH::read8(uint8_t reg)
{
  _wire->beginTransmission(_address);
  _wire->write(reg);
  _wire->endTransmission();

  _wire->requestFrom(_address, (uint8_t)1);
  return _wire->read();
}

// Function to read a 16-bit value from a register (MSB and LSB)
uint16_t NHY3274TH::read16(uint8_t reg)
{
  uint16_t x;
  x = read8(reg);
  x |= (read8(reg + 1) << 8);
  return x;
}