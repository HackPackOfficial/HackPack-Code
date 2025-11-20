#ifndef CONFIG_H
#define CONFIG_H

/*
  ************************************************************************************
  *
  * CONFIG.H - User Configuration Settings for Stock Synthesizer
  * 
  * This file contains all settings that users can safely modify to customize
  * their synthesizer behavior. All values include suggested ranges in comments.
  * 
  * SAFE TO MODIFY: All settings in this file are designed to be user-adjustable
  * 
  ************************************************************************************
*/

//////////////////////////////////////////////////
//  USER CONFIGURATION - SAFE TO MODIFY //
//////////////////////////////////////////////////

// === DEBUG AND SERIAL OUTPUT ===
#define USE_SERIAL true   // enables serial output for debugging, set to false to disable serial output
// NOTE: Serial plotting can be very helpful for monitoring synth parameters, but it can also slow down audio response time

// === TOUCHPAD CALIBRATION ===
// Adjust these values to match your touchpad's actual touch range
#define TOUCHPAD_X_MIN 130        // Minimum X coordinate (range: 100-200)
#define TOUCHPAD_X_MAX 850        // Maximum X coordinate (range: 700-900)
#define TOUCHPAD_Y_MIN 135        // Minimum Y coordinate (range: 100-200)
#define TOUCHPAD_Y_MAX 925        // Maximum Y coordinate (range: 900-950)
#define TOUCHPAD_EDGE_DETECT 915  // Y/X value for finger-off detection (range: 920-970)

// === MUSICAL RANGE SETTINGS ===
#define MIDI_ROOT_MIN 36          // Minimum root note (range: 24-60, C1-C4)
#define MIDI_ROOT_MAX 96          // Maximum root note (range: 60-108, C4-C8)
#define DEFAULT_SCALE_TYPE 0      // Default scale type on startup (0=Major, 1=Minor, etc.)
#define DEFAULT_ROOT_NOTE 36      // Default root note (C1 = MIDI 36)

// === CONTROL SMOOTHING FACTORS ===
// Lower values = faster response, higher values = smoother but slower
#define SMOOTH_FREQ_NORMAL 0.7f   // Frequency smoothing (range: 0.1-0.9)
#define SMOOTH_GAIN_FAST 0.55f    // Fast gain smoothing (range: 0.3-0.8)
#define SMOOTH_GAIN_SLOW 0.95f    // Slow gain fade-out (range: 0.9-0.99)

// === WAVE SELECTION SETTINGS ===
#define WAVE_COUNT 4              // Number of available waveforms (0-3: Triangle, Square, Saw, Noise)
#define DEFAULT_WAVE 0            // Default wave selection (0=Triangle)

// === GAIN AND VOLUME SETTINGS ===
#define MASTER_GAIN_MAX 48        // Maximum master gain value (range: 24-64)
#define VOLUME_SHIFT_AMOUNT 6     // Bit shift for volume control (range: 4-8, higher = quieter)
#define PROX_GAIN_REDUCTION 2     // Proximity sensor gain reduction factor (range: 2-4, higher = more reduction)

// === LFO SYSTEM SETTINGS ===
#define LFO_MIN_BPM 60.0f         // Minimum LFO BPM when spinner stops (range: 30-120)
#define LFO_MAX_BPM 480.0f        // Maximum LFO BPM (range: 240-600)
#define LFO_SPINNER_NODES 3       // Number of nodes on fidget spinner (range: 2-4)
#define LFO_MIN_NODE_WIDTH 3      // Minimum control cycles for valid node (range: 2-5)
#define LFO_DECAY_TIMEOUT 128     // Timeout cycles before LFO decay (range: 64-256)
#define LFO_BPM_MULTIPLIER 2.0f   // RPM to BPM conversion multiplier (range: 1.0-4.0)
#define LFO_DECAY_RATE 0.998f     // LFO decay rate per cycle (range: 0.99-0.999)

// === AUDIO OUTPUT SETTINGS ===
#define AUDIO_OUTPUT_BITS 15      // Audio output bit depth (range: 13-15, higher = more dynamic range)

// === SCALE SYSTEM SETTINGS ===
#define MAX_SCALE_NOTES 21        // Maximum notes in generated scale (3 octaves * 7 notes)
#define SCALE_OCTAVES 3           // Number of octaves to generate (range: 2-4)
#define NOTE_OFFSET_MIN 4         // Minimum note offset for touchpad mapping (range: 0-8)

// === CONTROL TIMING SETTINGS ===
#define SERIAL_OUTPUT_DIVIDER 64  // How often to output serial data (range: 32-128, higher = less frequent)

#endif // CONFIG_H
