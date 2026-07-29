#ifndef LTR381RGB_H
#define LTR381RGB_H

#include <Arduino.h>
#include <Wire.h>
#include "ColorSensor.h"

// Default I2C address for LTR-381RGB-01
#define LTR381RGB_ADDRESS 0x53

// Register map (from datasheet)
#define LTR381RGB_MAIN_CTRL       0x00
#define LTR381RGB_MEAS_RATE       0x04
#define LTR381RGB_GAIN            0x05
#define LTR381RGB_PART_ID         0x06
#define LTR381RGB_MAIN_STATUS     0x07

// Data registers (20-bit RGB + IR)
#define LTR381RGB_DATA_IR         0x0A
#define LTR381RGB_DATA_GREEN      0x0D
#define LTR381RGB_DATA_RED        0x10
#define LTR381RGB_DATA_BLUE       0x13

class LTR381RGB : public ColorSensor {
public:
  LTR381RGB(uint8_t address = LTR381RGB_ADDRESS);

  bool begin() override;
  void getRawData(uint16_t* r, uint16_t* g, uint16_t* b, uint16_t* w) override;

  void enable();
  void disable();

  void setGain(uint8_t gain);
  void setResolutionAndRate(uint8_t resolutionBits, uint8_t rateCode);

private:
  uint8_t _address;
  TwoWire* _wire;

  void write8(uint8_t reg, uint8_t value);
  uint8_t read8(uint8_t reg);
  uint32_t read20(uint8_t regBase); // Reads 20-bit channel value
};

#endif // LTR381RGB_H
