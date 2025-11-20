#ifndef WAVETABLES_H
#define WAVETABLES_H

#include <tables/cos2048_int8.h>           // Cosine wave for kick drum
#include <tables/whitenoise8192_int8.h>    // White noise for hi-hat and snare
#include <tables/saw1024_int8.h>           // Smaller saw wave for bass (memory optimized)
#include <tables/sin1024_int8.h>           // Smaller sine wave for snare tone (memory optimized)

// Oscillators used in the Drum and Bass mode - memory optimized
Oscil<COS2048_NUM_CELLS, AUDIO_RATE> kickOsc(COS2048_DATA);        // Kick drum sine wave
Oscil<WHITENOISE8192_NUM_CELLS, AUDIO_RATE> hihatOsc(WHITENOISE8192_DATA); // Hi-hat white noise
Oscil<WHITENOISE8192_NUM_CELLS, AUDIO_RATE> snareOsc(WHITENOISE8192_DATA); // Snare drum noise
Oscil<SIN1024_NUM_CELLS, AUDIO_RATE> snareTone(SIN1024_DATA);      // Snare tonal component (smaller)
Oscil<SAW1024_NUM_CELLS, AUDIO_RATE> bassOsc(SAW1024_DATA);        // Bass saw wave (smaller)
Oscil<SAW1024_NUM_CELLS, AUDIO_RATE> bassOsc2(SAW1024_DATA);       // Second bass saw wave (smaller)

#endif