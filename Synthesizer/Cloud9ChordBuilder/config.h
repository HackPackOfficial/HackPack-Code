#ifndef CONFIG_H
#define CONFIG_H

/*
  ************************************************************************************
  *
  * CONFIG.H - User Configuration Settings
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
#define USE_SERIAL false  // enables serial output for debugging, set to false to disable serial output
// NOTE: Serial plotting can be very helpful for monitoring synth parameters, but it can also slow down audio response time
#define SERIAL_OUTPUT_DIVIDER 50  // How often to output serial data (range: 20-100, higher = less frequent)

// === MUSICAL RANGE SETTINGS ===
#define MIDI_ROOT_MIN 84          // Minimum root note (range: 60-84, C4-C6)
#define MIDI_ROOT_MAX 96          // Maximum root note (range: 84-108, C6-C8)
#define DEFAULT_CHORD_TABLE 0     // Default chord table on startup (range: 0-10)

// === CONTROL SMOOTHING FACTORS ===
// Lower values = faster response, higher values = smoother but slower
#define SMOOTH_FREQ_NORMAL 0.75f  // Frequency smoothing (range: 0.1-0.9)
#define SMOOTH_FREQ_SLOW 0.85f    // Slower frequency smoothing (range: 0.7-0.95)
#define SMOOTH_GAIN_FAST 0.35f    // Fast gain fade-in (range: 0.1-0.5)
#define SMOOTH_GAIN_SLOW 0.9999f  // Slow gain fade-out (range: 0.99-0.9999)
#define SMOOTH_GAIN_NORMAL 0.65f  // Normal gain smoothing (range: 0.3-0.8)

// === HYSTERESIS SETTINGS ===
// Higher values = less sensitive to small movements, reduces jitter
#define CHORD_HYSTERESIS 40       // Chord selection dead zone (range: 20-60)
#define INVERSION_HYSTERESIS 30   // Inversion selection dead zone (range: 15-45)
#define POT_HYSTERESIS 15         // Potentiometer change threshold (range: 5-25)

// === LFO SYSTEM SETTINGS ===
#define LFO_MODE_ENABLED false    // Toggle for LFO mode vs direct proximity control
#define LFO_MIN_BPM 60.0f         // Minimum LFO BPM when spinner stops (range: 30-120)
#define LFO_MAX_BPM 480.0f        // Maximum LFO BPM (range: 240-600)
#define LFO_SPINNER_NODES 3       // Number of nodes on fidget spinner (range: 2-4)
#define LFO_MIN_NODE_WIDTH 3      // Minimum control cycles for valid node (range: 2-5)
#define LFO_DECAY_TIMEOUT 128     // Timeout cycles before LFO decay (range: 64-256)

// === AUDIO MIXING SETTINGS ===
#define BASS_GAIN_SHIFT 5         // Bass frequency gain reduction (range: 4-6, higher = quieter bass)
#define TREBLE_GAIN_SHIFT 6       // Treble frequency gain reduction (range: 5-7, higher = quieter treble)
#define AUDIO_OUTPUT_BITS 15      // Audio output bit depth (range: 13-15, higher = more dynamic range)

// === TOUCHPAD CALIBRATION ===
// Adjust these values to match your touchpad's actual touch range
#define TOUCHPAD_X_MIN 150        // Minimum X coordinate (range: 100-200)
#define TOUCHPAD_X_MAX 740        // Maximum X coordinate (range: 700-800)
#define TOUCHPAD_Y_MIN 135        // Minimum Y coordinate (range: 100-200)
#define TOUCHPAD_Y_MAX 930        // Maximum Y coordinate (range: 900-950)
#define TOUCHPAD_EDGE_DETECT 928  // Y value for finger-off detection (range: 920-940)
#define TOUCHPAD_X_EDGE_MIN 100   // X minimum for finger-off detection (range: 80-120)
#define TOUCHPAD_X_EDGE_MAX 800   // X maximum for finger-off detection (range: 780-820)

//////////////////////////////////////////////////
//  HARDWARE PIN ASSIGNMENTS //
//////////////////////////////////////////////////
// Potentiometer pins (analog inputs)
#define POT_PIN1 A5                      // Bottom potentiometer - Bass note length control
#define POT_PIN2 A6                      // Middle potentiometer - Tempo control  
#define POT_PIN3 A7                      // Top potentiometer - Swing control

// Proximity sensor pin (digital input)
#define PROX_PIN 6                       // Proximity sensor for fidget spinner detection

// Touchpad pins (resistive touchpad interface)
#define TOUCHPAD_Y1 A1                   // Touchpad Y coordinate pin 1
#define TOUCHPAD_Y2 8                    // Touchpad Y coordinate pin 2
#define TOUCHPAD_X1 A0                   // Touchpad X coordinate pin 1
#define TOUCHPAD_X2 7                    // Touchpad X coordinate pin 2

#endif // CONFIG_H
