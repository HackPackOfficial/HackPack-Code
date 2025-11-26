#ifndef CONFIG_H
#define CONFIG_H

/*
  ************************************************************************************
  *
  * DRUM SEQUENCER CONFIGURATION - User Customizable Settings
  * 
  * This file contains all user-configurable parameters for the drum sequencer.
  * Modify these values to customize the synthesizer's behavior, control response,
  * and audio characteristics.
  * 
  * SAFE RANGES: Each setting includes safe value ranges in comments.
  * Values outside these ranges may cause instability or poor audio quality.
  * 
  ************************************************************************************
*/

//////////////////////////////////////////////////
//  SERIAL AND DEBUG CONFIGURATION //
//////////////////////////////////////////////////
#define USE_SERIAL true                   // Enable serial output for debugging (true/false)
#define DEBUG_SEQUENCER true             // Enable sequencer timing debug output (true/false)
#define SERIAL_BAUD_RATE 9600            // Serial communication baud rate (9600, 19200, 38400)

//////////////////////////////////////////////////
//  HARDWARE PIN ASSIGNMENTS //
//////////////////////////////////////////////////
// Potentiometer pins (analog inputs)
#define POT_PIN1 A5                      // Bottom potentiometer - Bass note length control
#define POT_PIN2 A6                      // Middle potentiometer - Tempo control  
#define POT_PIN3 A7                      // Top potentiometer - Swing control

// Proximity sensor pin (digital input)
#define PROX_PIN 2                       // Proximity sensor for fidget spinner detection

// Touchpad pins (resistive touchpad interface)
#define TOUCHPAD_Y1 A1                   // Touchpad Y coordinate pin 1
#define TOUCHPAD_Y2 8                    // Touchpad Y coordinate pin 2
#define TOUCHPAD_X1 A0                   // Touchpad X coordinate pin 1
#define TOUCHPAD_X2 7                    // Touchpad X coordinate pin 2

//////////////////////////////////////////////////
//  TOUCHPAD CALIBRATION //
//////////////////////////////////////////////////
#define TOUCHPAD_X_MIN 150               // Minimum X coordinate (range: 100-200)
#define TOUCHPAD_X_MAX 740               // Maximum X coordinate (range: 700-800)
#define TOUCHPAD_Y_MIN 135               // Minimum Y coordinate (range: 100-200)
#define TOUCHPAD_Y_MAX 915               // Maximum Y coordinate (range: 900-950)
#define TOUCHPAD_EDGE_DETECT 950         // Y/X value for finger-off detection (range: 920-970)

//////////////////////////////////////////////////
//  TEMPO AND TIMING CONFIGURATION //
//////////////////////////////////////////////////
#define TEMPO_MIN 30                     // Minimum tempo in BPM (range: 20-60)
#define TEMPO_MAX 160                    // Maximum tempo in BPM (range: 120-180)
#define DEFAULT_TEMPO 120                // Default startup tempo in BPM
#define TEMPO_TRANSITION_LENGTH 4        // Number of steps for tempo transitions (range: 2-8)

// Pattern configuration
#define PATTERN_LENGTH 16                // Steps per pattern (8, 16, or 32)
#define MAX_PATTERN_LENGTH 16            // Maximum pattern length supported

// Bass note length control
#define BASS_NOTE_LENGTH_MIN 20          // Minimum bass note length in ms (range: 10-50)
#define BASS_NOTE_LENGTH_MAX 600         // Maximum bass note length in ms (range: 400-1000)
#define DEFAULT_BASS_NOTE_LENGTH 80      // Default bass note length in ms

// Swing control
#define SWING_MIN -100                   // Minimum swing amount (range: -100 to 0)
#define SWING_MAX 100                    // Maximum swing amount (range: 0 to 100)

//////////////////////////////////////////////////
//  AUDIO MIXING AND GAIN CONTROL //
//////////////////////////////////////////////////
// Audio bit shifting for mixing (higher values = quieter)
#define KICK_GAIN_SHIFT 1                // Kick drum gain bit shift (range: 0-3)
#define HIHAT_GAIN_SHIFT_1 2             // Hi-hat first bit shift (range: 1-4)
#define HIHAT_GAIN_SHIFT_2 3             // Hi-hat second bit shift (range: 2-5)
#define SNARE_NOISE_SHIFT 6              // 808 snare noise component shift (range: 2-4) - minimal noise like 808
#define SNARE_TONAL_SHIFT 1              // 808 snare tonal component shift (range: 3-5) - very loud tone for 808 character
#define BASS_GAIN_SHIFT 3                // Bass signal bit shift (range: 2-4)

// Audio output configuration
#define AUDIO_OUTPUT_BITS 13             // Audio output bit depth (range: 10-15)

//////////////////////////////////////////////////
//  DRUM SOUND PARAMETERS //
//////////////////////////////////////////////////
// Kick drum settings
#define KICK_FREQUENCY 45.0f             // Base kick frequency in Hz (range: 30-80)
#define KICK_ATTACK_TIME 10              // Kick attack time in ms (range: 5-20)
#define KICK_DECAY_TIME 120              // Kick decay time in ms (range: 80-200)

// Hi-hat settings  
#define HIHAT_ATTACK_TIME 7              // Hi-hat attack time in ms (range: 3-15)
#define HIHAT_DECAY_TIME 50              // Hi-hat decay time in ms (range: 30-100)

// Snare settings - Roland TR-808 style snare
#define SNARE_ATTACK_TIME 1              // 808 snare attack time in ms (range: 2-10) - instant attack
#define SNARE_DECAY_TIME 35              // 808 snare decay time in ms (range: 50-150) - very short and punchy
#define SNARE_TONE_FREQUENCY 220.0f      // 808 snare frequency in Hz (range: 150-300) - classic 808 pitch

// Bass settings
#define BASS_ATTACK_TIME 10              // Bass attack time in ms (range: 5-20)
#define BASS_DECAY_TIME 120              // Bass decay time in ms (range: 80-200)

// BASS-CENTRIC MIDI tuning system - Bass is the foundation, drums follow
#define BASS_ROOT_MIDI_NOTE 32           // A#1 - bass root MIDI note (foundation)
#define KICK_INTERVAL_OFFSET -5          // Perfect 4th below bass (kick = bass - 5 semitones = F1)
#define SNARE_TONE_INTERVAL_OFFSET 7     // Perfect 5th above bass (snare tone = bass + 7 semitones = F2)
// Results: Bass=A#1(34), Kick=F1(29), Snare Tone=F2(41)

//////////////////////////////////////////////////
//  CONTROL HYSTERESIS AND SMOOTHING //
//////////////////////////////////////////////////
#define PATTERN_HYSTERESIS_THRESHOLD 5   // Threshold for pattern switching (range: 3-10)
#define TEMPO_CHANGE_THRESHOLD 2         // Minimum tempo change to trigger update (range: 1-5)
#define BASS_LENGTH_CHANGE_THRESHOLD 5   // Minimum bass length change threshold (range: 3-10)

//////////////////////////////////////////////////
//  SCALE INTERVALS FOR BASS SEQUENCES //
//////////////////////////////////////////////////
// Scale interval definitions (semitones above root)
#define MINOR_BLUES_INTERVALS {1.0f, 1.067f, 1.25f, 1.333f, 1.5f, 1.8f}
#define MAJOR_PENTATONIC_INTERVALS {1.0f, 1.125f, 1.25f, 1.5f, 1.688f}
#define DORIAN_INTERVALS {1.0f, 1.125f, 1.25f, 1.333f, 1.5f, 1.688f}
#define PHRYGIAN_INTERVALS {1.0f, 1.067f, 1.25f, 1.333f, 1.5f, 1.6f}

//////////////////////////////////////////////////
//  MEMORY OPTIMIZATION SETTINGS //
//////////////////////////////////////////////////
#define MAX_SCALE_FREQUENCIES 8          // Maximum scale frequencies to store (range: 6-12)
#define BASS_SEQUENCE_MAX_LENGTH 8       // Maximum bass sequence length (range: 6-12)

#endif // CONFIG_H
