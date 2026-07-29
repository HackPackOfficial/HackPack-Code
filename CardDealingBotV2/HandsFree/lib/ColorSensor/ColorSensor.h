#ifndef COLOR_SENSOR_H
#define COLOR_SENSOR_H

#include <Arduino.h>

class ColorSensor {
public:
  virtual bool begin() = 0;
  virtual void getRawData(uint16_t* r, uint16_t* g, uint16_t* b, uint16_t* w) = 0;
};

#endif