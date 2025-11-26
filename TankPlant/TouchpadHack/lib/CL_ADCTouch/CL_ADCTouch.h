/*
  ADCTouch.h - Library for Capacitive touch sensors using only one ADC PIN
  Original library created by martin2250, April 23, 2014.
  Original library can be found at https://github.com/martin2250/ADCTouch

  Modified to include readNext() function by Evan Barnes for Crunchlabs, September 2024.
  Released into the public domain under the MIT license. 

  The readNext function basically just puts one new ADC reading into the filter and then
  returns the updated filter value, rather than filling completely refilling the sample
  buffer with new readings each time it is called, which is how the read() function
  behaves. readNext() will introduce a larger phase delay into the filtered readings and
  probably be less accurate, but it also takes substantially less time to execute than 
  the full read() function. Both read() and readNext() are blocking functions, and in some
  situations read() blocks for long enough that it can impact other program functions.

  The original library also only exposed a singleton instance of the class. Basically,
  this means that if you tried to use multiple touch sensors with the original, all of the
  different sensor readings would get fed into the same _sum variable and _values array,
  so they would interfere with each other's readings. I modified this to be a proper OOP
  class so that you can instantiate multiple sensors. I've tested this with 3 simultaneous
  sensors with no problem, and it should work with all ADC channels on the ATMega328.

  Thank you martin2250 for putting together the original library!
*/
#ifndef ADCT_h
#define ADCT_h

#include "Arduino.h"

// Use constexpr for compile-time constant
constexpr int SAMPLES = 16;

class ADCTouchClass
{
  public:
    /**
     * @brief Constructor that sets the ADC channel used for sensing.
     * @param ADCChannel the ADC pin to use for capacitive sensing (e.g., A0).
     */
    ADCTouchClass(uint8_t ADCChannel);

    /**
     * @brief Performs a filtered capacitive touch reading on the specified ADC pin.
     * 
     * read() fills an array of length SAMPLES (default value is 16) with capacitive touch readings
     * from the ADC, then returns the average value. This is a blocking function, and can take a
     * substantial amount of time to complete, so if you need similar functionality that takes less
     * execution time, consider using readNext() instead.
     * 
     * @param ADCChannel the ADC pin to use for the reading (e.g., A0).
     * @return the average value of the sampled ADC readings.
     */
    int read();

    /**
     * @brief A variant of read() that only takes a single ADC reading before returning the average value.
     * 
     * readNext() still fills an array of length SAMPLES (default value is 16) with capacitive touch readings
     * from the ADC, then returns the average value. However, unlike read(), readNext() only takes one ADC 
     * reading each time the function is called. It puts that reading in the sample array and then returns
     * the average value of the sample array. It will take 16 calls to readNext() to completely fill the sample
     * buffer. This trades execution time for filter response time. readNext() is blocking, but takes ~1/16th
     * as much time to execute as a single call to read(). The downside is that it will take 16x more calls
     * to readNext() for the filtered sample readings to stabilize than it does for read(). If you have 
     * processing time to spare and want better sample readings, consider using read() instead of readNext().
     * 
     * @param ADCChannel the ADC pin to use for the reading (e.g., A0).
     * @return the average value of the sampled ADC readings.
     */
    int readNext();

    /**
     * @brief returns the last average value of the ADC sample buffer.
     * @return the average value of the sampled ADC readings.
     */
    int lastValue();

  private:
    // The ADC pin to use for capacitive sensing
    uint8_t _ADCChannel;

    // Array to hold rolling sample values
    int _values[SAMPLES] = {0};

    // Index for the current sample slot in the rolling buffer
    int _currentSlot = 0;

    // Last calculated average value
    int _lastValue = 0;

    // Running sum of values in the rolling array
    int _sum = 0;

};


#endif
