/*
  CL_ADCTouch.h - Library for Capacitive touch sensors using only one ADC PIN
  Original library created by martin2250, April 23, 2014.
  Original library can be found at https://github.com/martin2250/ADCTouch

  THIS IS A MODIFIED VERSION OF THE ORIGINAL LIBRARY.
  Modified to include readNext() function by Evan Barnes for Crunchlabs, September 2024.
  Released into the public domain under the MIT license.

  The original library, ADCTouch, does not include the readNext() function that is present here.

  The readNext function basically just puts one new ADC reading into the filter and then
  returns the updated filter value, rather than filling completely refilling the sample
  buffer with new readings each time it is called, which is how the read() function
  behaves. readNext() takes less time to execute than the read() function, but as a tradeoff
  it is more susceptible to noise. Both read() and readNext() are blocking functions, and in
  some situations read() blocks for long enough that it can impact other program functions.
*/

/*
MIT License

Copyright (c) 2022 Martin Pittermann
Copyright (c) 2025 CrunchLabs

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
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
    int read(byte ADCChannel);

    /**
     * @brief A variant of read() that only takes a single ADC reading before returning the average value.
     * 
     * readNext() still fills an array of length SAMPLES (default value is 16) with capacitive touch readings
     * from the ADC, then returns the average value. However, unlike read(), readNext() only takes one ADC 
     * reading each time the function is called. It puts that reading in the sample array and then returns
     * the average value of the sample array. It will take 16 calls to readNext() to completely fill the sample
     * buffer. This trades execution time for filter response time. readNext() is blocking, but takes ~1/16th
     * as much time to execute as a single call to read(). The downside is that it will take 16x longer for
     * the filtered sample readings to stabilize. If you have processing time to spare and want better sample
     * readings, consider using read() instead of readNext().
     * 
     * @param ADCChannel the ADC pin to use for the reading (e.g., A0).
     * @return the average value of the sampled ADC readings.
     */
    int readNext(byte ADCChannel);

    /**
     * @brief returns the last average value of the ADC sample buffer.
     * @return the average value of the sampled ADC readings.
     */
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
