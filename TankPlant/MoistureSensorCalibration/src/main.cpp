/** Hack Pack: Tank Plant
 * Moisture Sensor Calibration Code
 * 
 * This short program is purely for recalibrating your moisture sensor probe. Remove it from the LECA,
 * make sure that it's dry, and then leave it out of the LECA and suspended in open air. Then you can run
 * this code. The average open air reading gets reported back to you on the Serial monitor. Open 
 * Configuration.h, and set the value of MOISTURE_OFFSET to the average reading you just took. 
 * 
 * This is mostly just here for either diagnosing problems with your moisture sensor probe, or for 
 * recalibrating if you made some kind of change. If the value you measure with this code is close to the
 * original value of 754, your probe is probably fine. However, if you want to squeeze a bit more accuracy
 * out of your moisture probe, running this and recalibrating could be worthwhile. 
 */

#pragma region LICENSE

/*
MIT License

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

#pragma endregion LICENSE


#pragma region LIBRARIES AND CONFIGURATION

#include <Arduino.h>

// Many of the basic behaviors of the robot can be changed with the parameters stored in this header file.
// Open it up to see what you can change!
#include <Configuration.h>

#include <CL_ADCTouch.h>          // Performs capacitive sensing using a single ADC pin        

// The pin definitions are all in a separate file for neatness and clarity. If you want to change
// how the sensors and actuators are connected to the microcontroller, you can change that mapping
// in this file. 
#include <PinDefinitions.h>

#pragma endregion LIBRARIES AND CONFIGURATION

// create the object for managing the moisture sensor
ADCTouchClass moistureSensor(MOISTURE_SENSOR_PIN);



#pragma region SETUP
//********************************************************************************************************
// setup
//********************************************************************************************************


void setup() {
  // start the serial monitor, if we're using serial. 
  // Enable or disable Serial monitoring and printing in Configuration.h
  SERIAL_BEGIN(115200);

  SERIAL_PRINTLN("Sampling the sensor...");
  uint16_t measuredMoistureOffset = 0;
  for (uint8_t i = 0; i < 32; i++) {
    noInterrupts();
    measuredMoistureOffset += moistureSensor.read();
    interrupts();
    delay(10);
  }
  measuredMoistureOffset = measuredMoistureOffset / 32;
  SERIAL_PRINT("Measured moisture offset: ");
  SERIAL_PRINTLN(measuredMoistureOffset);
  while (true);
}
#pragma endregion SETUP

#pragma region LOOP
void loop() {
  
}

#pragma endregion LOOP