/*
  ADCTouch.h - Library for Capacitive touch sensors using only one ADC PIN
  Created by martin2250, April 23, 2014.
  Modified to include rolling average by [Your Name], September 2024.
  Released into the public domain.
*/
#ifndef ADCT_h
#define ADCT_h

#include "Arduino.h"

// Use constexpr for compile-time constant
constexpr int SAMPLES = 16;

class ADCTouchClass
{
  public:
    // Call once during setup to fill initial rolling buffer
    int read(byte ADCChannel);

    // Call repeatedly in loop for continuous averaging
    int readNext(byte ADCChannel);

    // Get the most recent averaged value
    int lastValue();

  private:
    // Array to hold rolling sample values
    int _values[SAMPLES] = {0};

    // Index for the current sample slot in the rolling buffer
    int _currentSlot = 0;

    // Last calculated average value
    int _lastValue = 0;

    // Running sum of values in the rolling array
    int _sum = 0;
};

extern ADCTouchClass ADCTouch;

#endif
