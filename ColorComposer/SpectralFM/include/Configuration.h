#ifndef CONFIGURATION_H
#define CONFIGURATION_H
#include <Arduino.h>
#include <Mozzi.h>

// Easy way to toggle Serial print statements. See Debug.h.
// Set to 1 to enable serial print debugging.
#define USE_SERIAL 0          // options: 0 or 1 (0 default)
#include <Debug.h>

// Set to 1 to push the PWM frequency on pins 3 and 11 (timer 2) above the
// audible range, removing the whine that would otherwise come from LED dimming.
#define USE_LED_PWM 1         // options: 1 or 0 (1 default)

// Set to 1 to push the PWM frequency on pins 5 and 6 (timer 0) above the
// audible range, removing motor noise from the audio output.
#define USE_FAST_PWM 1        // options: 1 or 0 (1 default)

// Stagger interval for I2C sensor updates. Three sensors share the bus, each
// getting a read every 3 * I2C_UPDATE_INTERVAL ms.
constexpr uint8_t I2C_UPDATE_INTERVAL = 15; // milliseconds

// **********************************************************************************
// Mozzi Configuration
// **********************************************************************************

// Control rate in Hz for updateControl(). Must be a power of 2.
#define MOZZI_CONTROL_RATE 128    // options: 8, 16, 32, 64, 128, 256, 512 (default 128)

// All three oscillators use cosine waves. FM synthesis is based on phase
// modulation of a carrier by a modulator; cosine is the conventional choice
// because it starts at a peak (phase 0 → maximum amplitude), which produces
// symmetric sideband pairs and avoids a DC offset in the modulator output.
//
// NOTE: Oscil<> requires <Oscil.h> to be included in the translation unit
// before this header is processed. main.cpp satisfies that dependency.

#include <tables/cos2048_int8.h>
#include <tables/triangle_dist_squared_2048_int8.h>

Oscil<COS2048_NUM_CELLS, MOZZI_AUDIO_RATE>   aCarrier(COS2048_DATA);   // FM carrier oscillator
Oscil<TRIANGLE_DIST_SQUARED_2048_NUM_CELLS, MOZZI_AUDIO_RATE>   aModulator(TRIANGLE_DIST_SQUARED_2048_DATA); // FM modulator oscillator
Oscil<COS2048_NUM_CELLS, MOZZI_CONTROL_RATE> kModIndex(COS2048_DATA);  // mod-index LFO (control-rate)

// **********************************************************************************
// Music Things
// **********************************************************************************

// Number of scales loaded into scaleContainer. Must match the initializer below.
constexpr uint8_t NUM_SCALES = 3;

#include <MusicTools.h>
#include <MusicalScales.h>

// The middle button (B1) cycles through these scales. The carrier note is
// always quantized to the active scale, so changing the scale changes the
// set of pitches available to the FM carrier.
ScaleStorage scaleContainer =
{
  {
    &scale_EbPentatonicMinor,   // options: any scale defined in MusicalScales.h
    &scale_CLydian,             // options: any scale defined in MusicalScales.h
    &scale_CPentatonicMajor     // options: any scale defined in MusicalScales.h
  },
  0  // index of the initially selected scale
};

// **********************************************************************************
// FM synthesis channel mappings
// **********************************************************************************
//
// These #defines wire the four mapped color channels to FM synthesis parameters.
// Changing a value here (e.g. swapping mappedRed for mappedBlue on FM_INDEX_CHANNEL)
// remaps that color to a different parameter without touching updateFM().
//
// Green  → carrier note selection within the current scale.
//           Smoothly scans through scale degrees as green light changes.
//
// Blue   → C:M (carrier-to-modulator) frequency ratio.
//           In harmonic mode (default): integer ratios 1–8, keeping all FM
//           sidebands as true harmonics of the carrier — tonal and periodic.
//           In inharmonic mode (button 2): fractional ratios 1.0–4.0, placing
//           sidebands at non-integer multiples — bell-like and metallic.
//
// Red    → modulation index (FM depth / timbral brightness).
//           Low red → near-pure sine tone. High red → dense, metallic spectrum.
//           The mapping is linear: index ≈ mappedRed / 4, so the full 0–255
//           range sweeps from index 0 to ~63.
//
// White  → rate of the mod-index LFO (kModIndex).
//           Slow white → leisurely timbral breathing (~0.05 Hz).
//           Fast white → rapid shimmer approaching vibrato character (~3.9 Hz).

#define FM_CARRIER_CHANNEL    mappedGreen     // options: mappedWhite, mappedRed, mappedGreen, mappedBlue (default mappedGreen)
#define FM_RATIO_CHANNEL      mappedBlue      // options: mappedWhite, mappedRed, mappedGreen, mappedBlue (default mappedBlue)
#define FM_INDEX_CHANNEL      mappedRed       // options: mappedWhite, mappedRed, mappedGreen, mappedBlue (default mappedRed)
#define FM_LFO_CHANNEL        mappedWhite     // options: mappedWhite, mappedRed, mappedGreen, mappedBlue (default mappedWhite)

#endif // CONFIGURATION_H