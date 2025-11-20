#pragma once
/*
  ************************************************************************************
  * WAVETABLES - Oscillator declarations and wavetable includes
  * 
  * Contains all wavetable includes and oscillator object declarations
  * Organized by waveform type for easy management
  *
  * Oscillator Usage:
  * - aSin1-4: Sine wave oscillators
  * - aSaw1-2: Sawtooth wave oscillators  
  * - aTri1-4: Triangle wave oscillators
  * - aSqu1-2: Square wave oscillators
  * - kFilterMod: Control rate envelope for modulation
  *
  ************************************************************************************
*/

//////////////////////////////////////////////////////////////////////////////////////////////////////////
// WAVETABLE INCLUDES
//////////////////////////////////////////////////////////////////////////////////////////////////////////

#include <Oscil.h> // oscillator template
#include <tables/sin2048_int8.h> // sine table for oscillator
#include <tables/envelop2048_uint8.h> // for filter modulation
#include <tables/saw2048_int8.h> // saw table for oscillator
#include <tables/square_no_alias_2048_int8.h> // square table for oscillator
#include <tables/triangle_dist_squared_2048_int8.h> // triangle table for oscillator
#include <tables/triangle_dist_cubed_2048_int8.h> // triangle table for oscillator

//////////////////////////////////////////////////////////////////////////////////////////////////////////
// OSCILLATOR DECLARATIONS
//////////////////////////////////////////////////////////////////////////////////////////////////////////

// Sine Wave Oscillators
Oscil <SIN2048_NUM_CELLS, AUDIO_RATE> aSin1(SIN2048_DATA);
Oscil <SIN2048_NUM_CELLS, AUDIO_RATE> aSin2(SIN2048_DATA);
Oscil <SIN2048_NUM_CELLS, AUDIO_RATE> aSin3(SIN2048_DATA);
Oscil <SIN2048_NUM_CELLS, AUDIO_RATE> aSin4(SIN2048_DATA);

// Sawtooth Wave Oscillators
Oscil <SAW2048_NUM_CELLS, AUDIO_RATE> aSaw1(SAW2048_DATA);
Oscil <SAW2048_NUM_CELLS, AUDIO_RATE> aSaw2(SAW2048_DATA);

// Triangle Wave Oscillators
Oscil <TRIANGLE_DIST_SQUARED_2048_NUM_CELLS, AUDIO_RATE> aTri1(TRIANGLE_DIST_SQUARED_2048_DATA);
Oscil <TRIANGLE_DIST_SQUARED_2048_NUM_CELLS, AUDIO_RATE> aTri2(TRIANGLE_DIST_SQUARED_2048_DATA);

// Square Wave Oscillators
Oscil <SQUARE_NO_ALIAS_2048_NUM_CELLS, AUDIO_RATE> aSqu1(SQUARE_NO_ALIAS_2048_DATA);
Oscil <SQUARE_NO_ALIAS_2048_NUM_CELLS, AUDIO_RATE> aSqu2(SQUARE_NO_ALIAS_2048_DATA);

// Control Rate Oscillators
Oscil<ENVELOP2048_NUM_CELLS, MOZZI_CONTROL_RATE> kFilterMod(ENVELOP2048_DATA);
Oscil<SIN2048_NUM_CELLS, MOZZI_CONTROL_RATE> kLFO(SIN2048_DATA);  // LFO for gain modulation
