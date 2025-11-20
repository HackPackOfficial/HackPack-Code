#ifndef SYSTEM_CONFIG_H
#define SYSTEM_CONFIG_H

/*
  ************************************************************************************
  *
  * SYSTEM_CONFIG.H - Internal System Constants for Stock Synthesizer
  * 
  * This file contains core system constants that define the synthesizer's
  * fundamental behavior. These values are carefully tuned for optimal performance
  * and should generally not be modified unless you understand the implications.
  * 
  * MODIFY WITH CAUTION: Changing these values may cause system instability
  * 
  ************************************************************************************
*/

//////////////////////////////////////////////////
//  SYSTEM CONFIGURATION - MODIFY WITH CAUTION //
//////////////////////////////////////////////////

// === CORE AUDIO ENGINE ===
#define CONTROL_RATE 128          // Control update rate in Hz (MUST be 128 for this sketch)
#define DEFAULT_FREQ 110          // Default oscillator frequency in Hz
#define ADC_MAX_VALUE 1023        // Maximum ADC reading value
#define ADC_RESOLUTION 10         // ADC resolution in bits

// === HARDWARE PIN DEFINITIONS ===
#define POT_PIN1 A5              // Bottom potentiometer (wave selection)
#define POT_PIN2 A6              // Middle potentiometer (second note)
#define POT_PIN3 A7              // Top potentiometer (master volume)
#define PROX_PIN 6               // Proximity sensor pin
#define TOUCHPAD_Y1 A1           // Touchpad Y coordinate pin 1
#define TOUCHPAD_Y2 8            // Touchpad Y coordinate pin 2
#define TOUCHPAD_X1 A0           // Touchpad X coordinate pin 1
#define TOUCHPAD_X2 7            // Touchpad X coordinate pin 2

// === MATHEMATICAL CONSTANTS ===
#define DIV1023 (1.0f / 1023.0f) // Division constant for ADC range conversion
#define OCTAVE_SEMITONES 12       // Semitones per octave

// === WAVEFORM ENUMERATION ===
enum waveState { 
  SINE = 0, 
  TRIANGLE = 1, 
  SAW = 2, 
  SQUARE = 3 
};

#endif // SYSTEM_CONFIG_H
