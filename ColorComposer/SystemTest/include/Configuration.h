#ifndef CONFIGURATION_H
#define CONFIGURATION_H
#include <Arduino.h>
#include <Mozzi.h>

// Easy way to toggle Serial print statements. See Debug.h to see how this works.
// Set the following value to 1 to enable serial print debugging:

#define USE_SERIAL 0          // options: 0 or 1 (0 default) -- ///{"range":[0,1]}
#include <Debug.h>

// set to 1 to change the PWM clock divisor for pins 3 and 11 (timer 2) - removes PWM noise due to LED dimming from audio
#define USE_LED_PWM 1         // options: 1 or 0 (1 default) -- ///{"range":[1,0],"step":-1}

// set to 1 to change the PWM clock divisor for pins 5 and 6 (timer 0) - removes motor noise from audio
#define USE_FAST_PWM 1        // options: 1 or 0 (1 default) -- ///{"range":[1,0],"step":-1}

// defines how often to update one of the sensors. The sensors get updated one at a time, so if this is left as the default
// 15ms interval, each sensor will be updated every 45ms.
constexpr uint8_t I2C_UPDATE_INTERVAL = 15; // time in milliseconds -- ///{"min":1,"max":255}


// **********************************************************************************
// Mozzi Configuration
// **********************************************************************************

// Control rate in Hz for updateControl() callback. Must be a power of 2. 
// Higher values improve responsiveness to sensor input but reduce processing time available for audio synthesis. 
// Audio glitching may indicate this value should be reduced.

#define MOZZI_CONTROL_RATE 128    // options: 8, 16, 32, 64, 128, 256, 512 (default 128) -- ///{"options":["8","16","32","64","128","256","512"]}

// These are the wavetables that will be used for each oscillator voice. Mozzi offers a lot of them to try out!
// To load a new wavetable that isn't included here already, you first have to include its header file, and then
// create an object from it, as shown below.
#include <tables/saw2048_int8.h>                    // Saw wave. 2048 samples represent 1 complete cycle of the wave. Each sample is 8 bit int.
#include <tables/triangle_dist_cubed_2048_int8.h>
#include <tables/triangle_valve_2_2048_int8.h>

Oscil<TRIANGLE_DIST_CUBED_2048_NUM_CELLS, MOZZI_AUDIO_RATE> osc0(TRIANGLE_DIST_CUBED_2048_DATA);  // oscillator 0
Oscil<SAW2048_NUM_CELLS,                  MOZZI_AUDIO_RATE> osc1(SAW2048_DATA);                   // oscillator 1
Oscil<TRIANGLE_VALVE_2_2048_NUM_CELLS,    MOZZI_AUDIO_RATE> osc2(TRIANGLE_VALVE_2_2048_DATA);     // oscillator 2

/**
 * General example of how to use a new wavetable:
 * 
 * #include <tables/chosen_waveform.h>
 * Oscil<CHOSEN_WAVEFORM_NUM_CELLS, MOZZI_AUDIO_RATE> newOscillator(CHOSEN_WAVEFORM_DATA);
 */

// **********************************************************************************
// Music Things
// **********************************************************************************

// First, define the number of scales we're going to use. This value is referenced in the header that gets included next,
// so it has to be included before that.
constexpr uint8_t NUM_SCALES = 3;

// Then include the MusicTools header, which includes functions that are musically useful for composition and things.
#include <MusicTools.h>

// This header includes a selection of musical scales you can try out! 
// Be sure to choose the ones that don't have the _data tag at the end. e.g.:
// scale_AMinPentatonic       << USE THIS ONE
// scale_AMinPentatonic_data  << NOT THIS ONE
#include <MusicalScales.h>

// I highly recommend trying out the various pentatonic scales. Pentatonic scales are constructed in a way that makes them sound
// good most of the time, since they are never dissonant.

// Put all of your scales into a single container so that we can iterate through them with button B1 on the front panel.
// IMPORTANT: The number of scales in this container must match NUM_SCALES, defined above.
ScaleStorage scaleContainer = 
{
  {
    &scale_EbPentatonicMinor,            // options: any of the scales defined in MusicalScales.h (default scale_EbPentatonicMinor) -- ///{"options":["&scale_EbPentatonicMinor","&scale_CPentatonicMajor","&scale_CHarmonicMajor","&scale_CLydian","&scale_CDorian","&scale_CWholeTone","&scale_CDiminishedHalfWhole","&scale_CBlues","&scale_CMelodicMinor","&scale_CHirajoshi","&scale_CHungarianMinor","&scale_CPhrygianDominant","&scale_FMajPentatonic","&scale_BbMajPentatonic","&scale_AMinPentatonic","&scale_DHirajoshi","&scale_GEgyptian","&scale_EKumoi","&scale_DIwato"]}
    &scale_CLydian,                      // options: any of the scales defined in MusicalScales.h (default scale_CLydian) -- ///{"options":["&scale_CLydian","&scale_CPentatonicMajor","&scale_CHarmonicMajor","&scale_EbPentatonicMinor","&scale_CDorian","&scale_CWholeTone","&scale_CDiminishedHalfWhole","&scale_CBlues","&scale_CMelodicMinor","&scale_CHirajoshi","&scale_CHungarianMinor","&scale_CPhrygianDominant","&scale_FMajPentatonic","&scale_BbMajPentatonic","&scale_AMinPentatonic","&scale_DHirajoshi","&scale_GEgyptian","&scale_EKumoi","&scale_DIwato"]}
    &scale_CPentatonicMajor              // options: any of the scales defined in MusicalScales.h (default scale_CPentatonicMajor) -- ///{"options":["&scale_CPentatonicMajor","&scale_EbPentatonicMinor","&scale_CHarmonicMajor","&scale_CLydian","&scale_CDorian","&scale_CWholeTone","&scale_CDiminishedHalfWhole","&scale_CBlues","&scale_CMelodicMinor","&scale_CHirajoshi","&scale_CHungarianMinor","&scale_CPhrygianDominant","&scale_FMajPentatonic","&scale_BbMajPentatonic","&scale_AMinPentatonic","&scale_DHirajoshi","&scale_GEgyptian","&scale_EKumoi","&scale_DIwato"]}
  }, 
  0 // index of currently selected scale.
};


// **********************************************************************************
// ambienceGenerator() specific configurations:
// **********************************************************************************

// Decrease this value to increase the probability of events that are partially randomly triggered
constexpr uint8_t RANDOMNESS_THRESHOLD = 255;          // range 0 to 255 (255 default) -- ///{"min":0,"max":255}

// these mappings determine which color sensor channels affect various parameters in ambienceGenerator()
#define OCTAVE_SHIFTER_CHANNEL        mappedWhite      // options: mappedWhite, mappedRed, mappedGreen, mappedBlue (default mappedWhite) -- ///{"options":["mappedWhite","mappedRed","mappedGreen","mappedBlue"]}
#define BASE_INTERVAL_CHANNEL         mappedWhite      // options: mappedWhite, mappedRed, mappedGreen, mappedBlue (default mappedWhite) -- ///{"options":["mappedWhite","mappedRed","mappedGreen","mappedBlue"]}

#define OSC_0_ARP_TRIGGER_CHANNEL     mappedRed        // options: mappedWhite, mappedRed, mappedGreen, mappedBlue (default mappedRed) -- ///{"options":["mappedRed","mappedWhite","mappedGreen","mappedBlue"]}
#define OSC_0_NOTE_SELECTOR_CHANNEL   mappedGreen      // options: mappedWhite, mappedRed, mappedGreen, mappedBlue (default mappedGreen) -- ///{"options":["mappedGreen","mappedWhite","mappedRed","mappedBlue"]}
#define OSC_0_BUTTON_2_MODE_TIMER     mappedGreen      // options: mappedWhite, mappedRed, mappedGreen, mappedBlue (default mappedGreen) -- ///{"options":["mappedGreen","mappedWhite","mappedRed","mappedBlue"]}

#define OSC_1_ARP_TRIGGER_CHANNEL     mappedGreen      // options: mappedWhite, mappedRed, mappedGreen, mappedBlue (default mappedGreen) -- ///{"options":["mappedGreen","mappedWhite","mappedRed","mappedBlue"]}
#define OSC_1_NOTE_SELECTOR_CHANNEL   mappedBlue       // options: mappedWhite, mappedRed, mappedGreen, mappedBlue (default mappedBlue) -- ///{"options":["mappedBlue","mappedWhite","mappedRed","mappedGreen"]}
#define OSC_1_BUTTON_2_MODE_TIMER     mappedBlue       // options: mappedWhite, mappedRed, mappedGreen, mappedBlue (default mappedBlue) -- ///{"options":["mappedBlue","mappedWhite","mappedRed","mappedGreen"]}

#define OSC_2_ARP_TRIGGER_CHANNEL     mappedBlue       // options: mappedWhite, mappedRed, mappedGreen, mappedBlue (default mappedBlue) -- ///{"options":["mappedBlue","mappedWhite","mappedRed","mappedGreen"]}
#define OSC_2_NOTE_SELECTOR_CHANNEL   mappedRed        // options: mappedWhite, mappedRed, mappedGreen, mappedBlue (default mappedRed) -- ///{"options":["mappedRed","mappedWhite","mappedGreen","mappedBlue"]}
#define OSC_2_BUTTON_2_MODE_TIMER     mappedRed        // options: mappedWhite, mappedRed, mappedGreen, mappedBlue (default mappedRed) -- ///{"options":["mappedRed","mappedWhite","mappedGreen","mappedBlue"]}

#define GLOBAL_ARP_TRIGGER            mappedWhite      // options: mappedWhite, mappedRed, mappedGreen, mappedBlue (default mappedWhite) -- ///{"options":["mappedWhite","mappedRed","mappedGreen","mappedBlue"]}

#endif