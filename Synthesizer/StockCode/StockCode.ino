#pragma region README
/*
  ************************************************************************************
  *
  * STOCK SYNTHESIZER - Flexible scale system with clean architecture!
  * 
  * Features:
  * • Flexible scale system with 12 scale types (Major, Minor, Pentatonic, Blues, Modal, etc.)
  * • Semantic control variables for easy remapping
  * • Clean modular function structure with comprehensive documentation
  * • Multiple waveforms (sin, saw, square, triangle)
  * • Smooth volume control with proximity sensor or LFO modulation
  * • Configurable parameters via config.h for easy customization
  *
  * CONTROLS:
  * • Y-axis (touch): Primary note selection from scale
  * • X-axis (touch): Secondary note volume control
  * • botPot (A5): Wave selection (0-3: Triangle, Square, Saw, Noise)
  * • midPot (A6): Secondary note selection from scale
  * • hiPot (A7): Master volume control 
  * • Proximity sensor: Reduces volume when triggered OR LFO modulation from fidget spinner
  *
  * SCALE SYSTEM:
  * • Root note configurable (default C1 = MIDI 36)
  * • Scale type selectable (0-11: Major, Minor, Dorian, etc.)
  * • Automatic 3-octave scale generation
  * • Easy transposition and scale switching
  * • All settings configurable in config.h
  *
  ************************************************************************************
  */
#pragma endregion README
#pragma region LICENSE
/*
  ************************************************************************************
  * MIT License
  *
  * Copyright (c) 2025 Crunchlabs LLC (Laser Synthesizer Code)

  * Permission is hereby granted, free of charge, to any person obtaining a copy
  * of this software and associated documentation files (the "Software"), to deal
  * in the Software without restriction, including without limitation the rights
  * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  * copies of the Software, and to permit persons to whom the Software is furnished
  * to do so, subject to the following conditions:
  *
  * The above copyright notice and this permission notice shall be included in all
  * copies or substantial portions of the Software.
  *
  * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
  * INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
  * PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
  * HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF
  * CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE
  * OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
  *
  ************************************************************************************
*/
#pragma endregion LICENSE
//////////////////////////////////////////////////
//  INCLUDES AND CONFIGURATION //
//////////////////////////////////////////////////
#pragma region INCLUDES

// Configuration files
#include "config.h"
#include "system_config.h"

// Mozzi audio library setup
#include <MozziConfigValues.h>
#define MOZZI_AUDIO_CHANNELS MOZZI_STEREO
#include <MozziGuts.h>
#include <LowPassFilter.h>
#include <mozzi_rand.h>
#include <mozzi_midi.h>
#include <Smooth.h>

// Project-specific includes
#include "wavetables.h"  // All wavetable includes and oscillator declarations
#include "scale_tables.h"  // All scale table definitions

//////////////////////////////////////////////////////////////////////////////////////////////////////////
// GLOBAL VARIABLES
#pragma region Global Variables

// === INPUT STATE VARIABLES ===
// Raw ADC input variables (0-1023 range)
uint16_t botPot = 0;  // Wave selection potentiometer (POT_PIN1)
uint16_t midPot = 0;  // Second note control potentiometer (POT_PIN2)
uint16_t hiPot = 0;   // Master volume potentiometer (POT_PIN3)


// === AUDIO ENGINE STATE ===
float targetNote1 = 0;  // Target frequency for smooth transitions
float targetNote2 = 0;  // Target frequency for second voice
float frequency1 = 0;   // Current frequency for primary voice
float frequency2 = 0;   // Current frequency for secondary voice
long asig;              // Audio signal output

// === VOLUME AND GAIN CONTROL ===
int volume = 0;         // Master volume level
byte gain = 0;          // Master gain control
byte volume1 = 0;       // Primary voice volume (oscillator 1)
byte volume2 = 0;       // Primary voice volume (oscillator 2)
byte volume3 = 0;       // Secondary voice volume (oscillator 1)
byte volume4 = 0;       // Secondary voice volume (oscillator 2)

// === TOUCHPAD AND CONTROL STATE ===
int xVal, yVal;         // Current touchpad coordinates
int xBuff, yBuff;       // Buffered touchpad coordinates (legacy)
int prox, proxBuff;     // Proximity sensor state
bool axis = false;      // Alternates between X and Y coordinate reading

// === LFO SYSTEM - Fidget spinner RPM to BPM conversion ===
bool lfoMode = false;                    // Toggle for LFO mode vs direct proximity control
uint16_t proxTriggerCount = 0;         // Control rate counter for trigger timing
uint16_t proxInterval = 0;             // Time between proximity triggers (control cycles)
float spinnerRPM = 0;                  // Calculated RPM of fidget spinner
float lfoBPM = LFO_MIN_BPM;            // LFO rate in beats per minute
float lfoFreq = 1.0;                   // LFO frequency in Hz
uint16_t decayCounter = 0;             // Counter for LFO decay timing

// Node hole filtering variables (using config constants)
uint8_t proxLowCount = 0;              // Count consecutive LOW readings
uint8_t proxHighCount = 0;             // Count consecutive HIGH readings  
bool validNodeTrigger = false;         // True when we have a valid node (not hole)

// === SEMANTIC CONTROL VARIABLES (processed from raw inputs) ===
uint8_t waveSet = DEFAULT_WAVE;        // Wave selection (0 to WAVE_COUNT-1)
uint8_t noteSet1 = 0;                  // Primary note index from scale
uint8_t noteSet2 = 0;                  // Secondary note index from scale  
uint16_t gainSet = 0;                  // Gain/volume level (0 to ADC_MAX_VALUE)
uint8_t volumeSet2 = 0;                // Secondary note volume control

// Current wave selection state
waveState wave2 = TRIANGLE;            // Current waveform selection

#pragma endregion Global Variables

//////////////////////////////////////////////////////////////////////////////////////////////////////////
// SMOOTH OBJECTS
#pragma region Smooth Objects
// Smooth objects for frequency and gain control (using config constants)
Smooth<int> kSmoothFreq1(SMOOTH_FREQ_NORMAL);   // Primary frequency smoothing
Smooth<int> kSmoothFreq2(SMOOTH_FREQ_NORMAL);   // Secondary frequency smoothing
Smooth<int> kSmoothGain(SMOOTH_GAIN_FAST);      // Normal gain smoothing
Smooth<int> kSmoothGain2(SMOOTH_GAIN_SLOW);     // Slower smoothing for volume fade-out
#pragma endregion Smooth Objects
#pragma endregion INCLUDES

//////////////////////////////////////////////////
//  S E T U P //
//////////////////////////////////////////////////
#pragma region SETUP
/**
 * @brief System initialization - sets up audio engine, scale system, and serial output
 * 
 * Step-by-step process:
 * 1. Initializes Mozzi audio engine at configured control rate
 * 2. Sets up the scale system with default scale and root note
 * 3. Initializes LFO system for fidget spinner modulation
 * 4. Enables serial output for debugging (if configured)
 * 
 * Why: Proper initialization ensures all systems start in a known good state
 * Configuration: All startup values are defined in config.h for easy customization
 */
void setup(){
  // Start Mozzi audio engine first
  startMozzi(CONTROL_RATE);
  
  // Initialize the scale system using config defaults
  generateScale(DEFAULT_SCALE_TYPE, DEFAULT_ROOT_NOTE);
  
  // Initialize LFO system
  kLFO.setFreq(1.0f);  // Start with 1 Hz (60 BPM)
  
  #if USE_SERIAL
    Serial.begin(9600);
    Serial.println(F("STOCK SYNTHESIZER - FLEXIBLE SCALE SYSTEM"));
    Serial.println(F("Version 2.0 - Configurable and Documented"));
    Serial.println(F("Features: 12 Scales, 4 Waveforms, LFO Modulation"));
    Serial.println(F("Configuration: Edit config.h to customize behavior"));
    Serial.println(F("==========================================="));
  #endif
}
#pragma endregion SETUP


//////////////////////////////////////////////////
//  FUNCTIONS  //
//////////////////////////////////////////////////
#pragma region FUNCTIONS

// Subfunctions to keep updateControl() organized and readable
#pragma region Control Helpers

/**
 * @brief Main control update function - called at CONTROL_RATE (128 Hz)
 * 
 * Step-by-step process:
 * 1. Reads all input sensors (touchpad, potentiometers, proximity)
 * 2. Processes note selection from scale system
 * 3. Handles gain control with LFO or proximity modulation
 * 4. Updates buffered values for next cycle
 * 
 * Why: Coordinates all control processing in the correct order for smooth audio
 * Critical: No delay() calls allowed - they would disrupt audio generation timing
 * Timing: Called automatically by Mozzi at CONTROL_RATE frequency
 */
void updateControl(){
  // Clean, organized control flow using subfunctions
  readInputs();
  selectNotes();
  setGains();
  
  // Buffer proximity value for next cycle
  proxBuff = prox;
}

//////////////////////////////////////////////////
//  L O O P //
//////////////////////////////////////////////////
#pragma region LOOP
/**
 * @brief Main program loop - handles audio generation timing
 * 
 * Step-by-step process:
 * 1. Calls Mozzi's audioHook() function
 * 2. audioHook() automatically calls updateControl() at CONTROL_RATE (128Hz)
 * 3. audioHook() automatically calls updateAudio() at AUDIO_RATE (16384Hz)
 * 4. Maintains precise timing for smooth audio generation
 * 
 * Why: Mozzi handles all audio timing - adding other code here would disrupt audio
 * Warning: Do NOT add delays or other code here - it will cause audio glitches
 * Critical: This function must complete quickly to maintain audio quality
 */
void loop(){
  audioHook(); // Mozzi's main audio engine - handles all timing automatically
}
#pragma endregion LOOP

/**
 * @brief Reads all input sensors and maps them to semantic control variables
 * 
 * Step-by-step process:
 * 1. Alternates between reading X and Y touchpad coordinates (prevents interference)
 * 2. Constrains touchpad values to calibrated touch range from config
 * 3. Reads all potentiometer values using fast Mozzi ADC functions
 * 4. Reads proximity sensor for LFO or direct volume control
 * 5. Updates LFO system if enabled (fidget spinner RPM tracking)
 * 6. Maps all raw inputs to semantic control variables for easy remapping
 * 
 * Touchpad Handling:
 * - Alternating X/Y reads prevents electrical interference between axes
 * - Edge detection allows finger-off sensing for smooth fade-outs
 * - Calibrated ranges from config.h ensure consistent response
 * 
 * Semantic Mapping:
 * - Raw inputs mapped to meaningful ranges (wave selection, note indices, etc.)
 * - Easy to remap controls by changing these assignments
 * - All ranges use config constants for easy customization
 * 
 * Why: Separates raw input reading from musical interpretation for flexibility
 */
void readInputs() {
  // Alternate between X and Y touchpad reading to prevent interference
  if (axis){
    yVal = ycoor();
    yVal = constrain(yVal, TOUCHPAD_Y_MIN, TOUCHPAD_Y_MAX);
    axis = !axis;
  } else {
    int rawXVal = xcoor();
    xVal = constrain(rawXVal, TOUCHPAD_X_MIN, TOUCHPAD_X_MAX);
    // Preserve raw value for edge detection (finger off touchpad)
    if (rawXVal > TOUCHPAD_EDGE_DETECT) {
      xVal = rawXVal;  // Keep raw value for edge detection
    }
    axis = !axis;
  }
  
  // Read potentiometers using fast Mozzi ADC functions
  botPot = mozziAnalogRead<ADC_RESOLUTION>(POT_PIN1);  // Wave selection
  midPot = mozziAnalogRead<ADC_RESOLUTION>(POT_PIN2);  // Secondary note control
  hiPot = mozziAnalogRead<ADC_RESOLUTION>(POT_PIN3);   // Master volume

  // Read proximity sensor for LFO or direct volume control
  prox = digitalRead(PROX_PIN);

  // Update LFO system if in LFO mode (fidget spinner RPM tracking)
  if (lfoMode) {
    updateSpinnerLFO();
  }

  // Map raw inputs to semantic control variables (easy to remap as needed)
  waveSet = map(botPot, 0, ADC_MAX_VALUE, 0, WAVE_COUNT - 1);  // Wave selection
  int totalNotes = scaleLength * SCALE_OCTAVES;                 // Total notes in generated scale
  noteSet1 = map(yVal, TOUCHPAD_Y_MIN, TOUCHPAD_Y_MAX, NOTE_OFFSET_MIN, totalNotes - 1);  // Primary note from Y
  noteSet2 = map(midPot, 0, ADC_MAX_VALUE, 0, totalNotes - 1); // Secondary note from pot
  gainSet = map(hiPot, 0, ADC_MAX_VALUE, 0, MASTER_GAIN_MAX);   // Master gain
  volumeSet2 = xVal >> VOLUME_SHIFT_AMOUNT;                     // Secondary volume from X (bit-shifted)
}

/**
 * @brief Handles note selection and frequency calculation from semantic variables
 * 
 * Step-by-step process:
 * 1. Checks if finger is on touchpad (not at edge detection value)
 * 2. Converts note indices to MIDI note numbers using generated scale
 * 3. Converts MIDI notes to frequencies using Mozzi's mtof() function
 * 4. Applies smoothing to frequencies to prevent audio clicks and pops
 * 5. Updates current waveform selection
 * 6. Sets all oscillator frequencies for the selected waveform
 * 
 * Scale System:
 * - Uses generated scale array (created by generateScale() function)
 * - Supports any scale type with automatic 3-octave generation
 * - Note indices map directly to scale positions for musical consistency
 * 
 * Frequency Smoothing:
 * - Prevents audio artifacts when changing notes rapidly
 * - Smoothing factor configurable in config.h
 * - Separate smoothing for each voice allows independent control
 * 
 * Why: Converts abstract note selections into precise audio frequencies
 * Musical: Ensures all notes stay within the selected scale for harmonic consistency
 */
void selectNotes() {
  // Only process notes when finger is on touchpad
  if (yVal < TOUCHPAD_EDGE_DETECT) {
    // Convert note indices to MIDI notes using generated scale
    targetNote1 = mtof(generatedScale[noteSet1]);
    frequency1 = kSmoothFreq1.next(targetNote1);

    targetNote2 = mtof(generatedScale[noteSet2]);
    frequency2 = kSmoothFreq2.next(targetNote2);
    
    // Update waveform selection and set oscillator frequencies
    wave2 = (waveState)waveSet;
    setFrequencies();
  }
}

/**
 * @brief Handles volume and gain control with proximity sensor or LFO modulation
 * 
 * Step-by-step process:
 * 1. Checks if finger is on touchpad or at edge (finger-off detection)
 * 2. Applies master gain control with optional LFO or proximity modulation
 * 3. Calculates individual voice volumes based on touchpad X position
 * 4. Provides smooth fade-out when finger leaves touchpad
 * 
 * Modulation Modes:
 * - LFO Mode: Fidget spinner RPM converted to musical BPM for smooth modulation
 * - Direct Mode: Proximity sensor directly reduces volume for trilling effect
 * 
 * Volume Distribution:
 * - Primary voices (1&2): Full master gain for strong fundamental
 * - Secondary voices (3&4): Modulated by X touchpad position for expression
 * - Gain adjustment prevents clipping while maintaining dynamic range
 * 
 * Edge Handling:
 * - Smooth fade-out when finger leaves touchpad prevents audio pops
 * - Slower smoothing for fade-out creates natural release envelope
 * 
 * Why: Provides expressive volume control with multiple modulation sources
 * Performance: Optimized calculations using bit shifts for efficiency
 */
void setGains() {
  if (yVal < TOUCHPAD_EDGE_DETECT) {  // Finger is on touchpad
    // Master gain with LFO or proximity sensor modulation
    if (lfoMode) {
      // LFO mode: use spinner-derived LFO for smooth modulation
      float lfoMod = getLFOModulation();  // 0.5 to 1.5 multiplier
      float lfoGain = gainSet * lfoMod;   // Apply LFO modulation to base gain
      gain = kSmoothGain.next(constrain(lfoGain, 0, MASTER_GAIN_MAX));
    } else {
      // Direct proximity mode (original behavior)
      if (prox == LOW){
        gain = kSmoothGain.next(gainSet >> PROX_GAIN_REDUCTION);  // Proximity reduces gain
      } else {
        gain = kSmoothGain.next(gainSet);  // Normal gain scaling
      }
    }

    byte gainAdjusted = gain >> 2;  // Adjust for smaller gain range to prevent clipping

    // Volume assignments using semantic variables
    volume1 = gain;                              // Primary note volume (full gain)
    volume2 = gain;                              // Primary note volume (full gain)
    volume3 = (volumeSet2 * gainAdjusted);       // Secondary note volume from X touchpad
    volume4 = (volumeSet2 * gainAdjusted);       // Secondary note volume from X touchpad
  } else {
    // Smooth fade out when finger leaves touchpad
    volume1 = kSmoothGain2.next(0);
    volume2 = volume1;
    volume3 = volume1;
    volume4 = volume1;
  }
}
#pragma endregion Control Helpers

//////////////////////////////////////////////////
//  TOUCHPAD INTERFACE FUNCTIONS //
//////////////////////////////////////////////////
#pragma region Touchpad Helpers

/**
 * @brief Reads Y-coordinate from resistive touchpad
 * 
 * Step-by-step process:
 * 1. Configures pins for Y-axis measurement (A1/8 as voltage divider, A0/7 as sense)
 * 2. Sets up voltage gradient across Y-axis by driving A1 HIGH and pin 8 LOW
 * 3. Reads voltage at touch point through A0 (proportional to Y position)
 * 4. Returns raw ADC value (0-1023) representing Y coordinate
 * 
 * Electrical Operation:
 * - Creates voltage divider across touchpad Y-axis
 * - Touch point acts as voltage tap, creating position-dependent voltage
 * - ADC converts voltage to digital value representing position
 * 
 * Why: Resistive touchpad requires specific pin configuration for each axis
 * Timing: Must alternate with xcoor() to prevent axis interference
 * 
 * @return Raw Y coordinate (0-1023, higher values = higher on touchpad)
 */
int ycoor(){
  pinMode(TOUCHPAD_Y1, OUTPUT);   // A1 - Y axis drive high
  pinMode(TOUCHPAD_Y2, OUTPUT);   // Pin 8 - Y axis drive low
  pinMode(TOUCHPAD_X1, INPUT);    // A0 - Y axis sense input
  pinMode(TOUCHPAD_X2, INPUT);    // Pin 7 - Y axis sense input
  digitalWrite(TOUCHPAD_Y1, HIGH);
  digitalWrite(TOUCHPAD_Y2, LOW);
  return mozziAnalogRead<ADC_RESOLUTION>(TOUCHPAD_X1);
}

/**
 * @brief Reads X-coordinate from resistive touchpad
 * 
 * Step-by-step process:
 * 1. Configures pins for X-axis measurement (A0/7 as voltage divider, A1/8 as sense)
 * 2. Sets up voltage gradient across X-axis by driving pin 7 HIGH and A0 LOW
 * 3. Reads voltage at touch point through A1 (proportional to X position)
 * 4. Returns raw ADC value (0-1023) representing X coordinate
 * 
 * Electrical Operation:
 * - Creates voltage divider across touchpad X-axis (perpendicular to Y)
 * - Touch point voltage now represents horizontal position
 * - Pin configuration swapped from ycoor() to measure different axis
 * 
 * Why: Resistive touchpad requires axis-specific pin configuration
 * Timing: Must alternate with ycoor() to prevent electrical interference
 * 
 * @return Raw X coordinate (0-1023, higher values = further right on touchpad)
 */
int xcoor(){
  pinMode(TOUCHPAD_X1, OUTPUT);   // A0 - X axis drive low
  pinMode(TOUCHPAD_X2, OUTPUT);   // Pin 7 - X axis drive high
  pinMode(TOUCHPAD_Y1, INPUT);    // A1 - X axis sense input
  pinMode(TOUCHPAD_Y2, INPUT);    // Pin 8 - X axis sense input
  digitalWrite(TOUCHPAD_X1, LOW);
  digitalWrite(TOUCHPAD_X2, HIGH);
  return mozziAnalogRead<ADC_RESOLUTION>(TOUCHPAD_Y1);
}

#pragma endregion Touchpad Helpers

//////////////////////////////////////////////////
//  LFO SYSTEM - FIDGET SPINNER INTEGRATION //
//////////////////////////////////////////////////
#pragma region LFO System

/**
 * @brief Tracks fidget spinner RPM and converts to musical BPM for LFO modulation
 * 
 * Step-by-step process:
 * 1. Filters proximity sensor pulses to detect valid spinner nodes (not holes)
 * 2. Measures time intervals between valid node triggers
 * 3. Calculates spinner RPM from node timing and known node count
 * 4. Converts RPM to musical BPM with configurable multiplier
 * 5. Updates LFO oscillator frequency for smooth gain modulation
 * 6. Implements decay when spinner slows down or stops
 * 
 * Node Filtering:
 * - Requires minimum pulse width to distinguish nodes from holes
 * - Prevents false triggers from spinner construction gaps
 * - Uses consecutive sample counting for reliable detection
 * 
 * RPM Calculation:
 * - Measures time between node triggers in control cycles
 * - Converts to milliseconds using CONTROL_RATE
 * - Calculates RPM: 60000ms/min ÷ (interval × nodes_per_rotation)
 * 
 * Musical Mapping:
 * - RPM multiplied by configurable factor for musical feel
 * - Constrained to reasonable BPM range (60-480 BPM)
 * - Converted to Hz for LFO oscillator
 * 
 * Decay System:
 * - Gradually reduces LFO rate when no new triggers detected
 * - Prevents abrupt stops, creates natural fade-out
 * - Configurable timeout and decay rate
 * 
 * Why: Converts physical spinner motion into expressive musical modulation
 * Innovation: Unique control interface using fidget spinner as musical controller
 */
void updateSpinnerLFO() {
  static bool lastValidNodeState = false;
  static uint16_t lastTriggerTime = 0;
  bool currentProxState = (prox == LOW);
  
  // Increment timing counters every control cycle
  proxTriggerCount++;
  decayCounter++;
  
  // Filter out spinner holes by requiring minimum pulse width
  if (currentProxState) {
    proxLowCount++;
    proxHighCount = 0;
    // Valid node detected if LOW signal persists for minimum width
    if (proxLowCount >= LFO_MIN_NODE_WIDTH) {
      validNodeTrigger = true;
    }
  } else {
    proxHighCount++;
    proxLowCount = 0;
    // End of node when HIGH signal persists (gap between nodes)
    if (proxHighCount >= LFO_MIN_NODE_WIDTH) {
      validNodeTrigger = false;
    }
  }
  
  // Detect rising edge of valid node (new node trigger)
  if (validNodeTrigger && !lastValidNodeState) {
    // Calculate time interval since last valid trigger
    if (lastTriggerTime > 0) {
      proxInterval = proxTriggerCount - lastTriggerTime;
      
      // Convert control cycles to milliseconds
      float intervalMs = (proxInterval * 1000.0f) / CONTROL_RATE;
      
      // Calculate RPM with sanity check for reasonable timing
      if (intervalMs > 50 && intervalMs < 5000) {  // 50ms to 5s per node
        spinnerRPM = 60000.0f / (intervalMs * LFO_SPINNER_NODES);
        
        // Map RPM to musical BPM with configurable multiplier
        lfoBPM = constrain(spinnerRPM * LFO_BPM_MULTIPLIER, LFO_MIN_BPM, LFO_MAX_BPM);
        
        // Convert BPM to Hz and update LFO oscillator
        lfoFreq = lfoBPM / 60.0f;
        kLFO.setFreq(lfoFreq);
        
        // Reset decay counter - we have fresh input
        decayCounter = 0;
      }
    }
    
    // Record trigger time for next interval calculation
    lastTriggerTime = proxTriggerCount;
  }
  
  lastValidNodeState = validNodeTrigger;
  
  // Implement LFO decay when spinner slows down
  if (decayCounter > LFO_DECAY_TIMEOUT) {
    lfoBPM = lfoBPM * LFO_DECAY_RATE;  // Gradual decay
    if (lfoBPM < LFO_MIN_BPM) lfoBPM = LFO_MIN_BPM;
    lfoFreq = lfoBPM / 60.0f;
    kLFO.setFreq(lfoFreq);
    
    // Reset decay counter for next decay cycle
    decayCounter = LFO_DECAY_TIMEOUT - 16;  // Check every 16 cycles
  }
}

/**
 * @brief Gets LFO modulation value for smooth gain control
 * 
 * Step-by-step process:
 * 1. Reads current LFO oscillator output (-127 to +127)
 * 2. Normalizes to floating point range (-1.0 to +1.0)
 * 3. Scales and offsets to create gain multiplier (0.5 to 1.5)
 * 4. Returns modulation value for smooth volume tremolo effect
 * 
 * Modulation Range:
 * - 0.5 = 50% of base gain (minimum modulation)
 * - 1.0 = 100% of base gain (no modulation)
 * - 1.5 = 150% of base gain (maximum modulation)
 * 
 * Musical Effect:
 * - Creates smooth tremolo (volume modulation) synchronized to spinner speed
 * - Faster spinning = faster tremolo rate
 * - Slower spinning = slower, more dramatic volume swells
 * 
 * Why: Converts LFO oscillator output to musically useful gain modulation
 * Range: Carefully chosen to provide expressive modulation without distortion
 * 
 * @return LFO modulation multiplier (0.5 to 1.5 range)
 */
float getLFOModulation() {
  // Get current LFO output and normalize to -1.0 to +1.0 range
  int lfoValue = kLFO.next();
  float lfoNormalized = lfoValue / 127.0f;
  
  // Scale to 0.5-1.5 multiplier range for smooth gain modulation
  return 1.0f + (lfoNormalized * 0.5f);
}
#pragma endregion LFO System

/**
 * @brief Generates a complete scale array from root note and scale type
 * 
 * Step-by-step process:
 * 1. Validates and constrains scale index to available scale types
 * 2. Updates global scale state variables (currentScale, rootNote, scaleLength)
 * 3. Retrieves interval pattern for selected scale type from scale tables
 * 4. Generates multiple octaves of the scale using semitone intervals
 * 5. Populates generatedScale array with MIDI note numbers
 * 6. Outputs debug information if serial is enabled
 * 
 * Scale Generation:
 * - Uses semitone interval patterns from scale_tables.h
 * - Generates SCALE_OCTAVES octaves for wide note range
 * - Each octave adds 12 semitones (one chromatic octave)
 * - Array bounds checking prevents buffer overflow
 * 
 * Musical Theory:
 * - Root note becomes scale degree 1 (tonic)
 * - Intervals define the characteristic sound of each scale
 * - Multiple octaves provide full range for musical expression
 * - MIDI note numbers allow direct frequency conversion
 * 
 * Why: Pre-generates all scale notes for fast real-time lookup during performance
 * Flexibility: Supports any scale type and root note for complete musical freedom
 * 
 * @param scaleIndex Index of scale type (0 to NUM_SCALES-1, see scale_tables.h)
 * @param root Root MIDI note number (typically 24-96 for reasonable range)
 */
void generateScale(uint8_t scaleIndex, uint8_t root) {
  // Validate and constrain scale index to available scales
  scaleIndex = constrain(scaleIndex, 0, NUM_SCALES - 1);
  
  // Update global scale state variables
  currentScale = scaleIndex;
  rootNote = root;
  scaleLength = scaleLengths[scaleIndex];
  
  // Get the semitone interval pattern for this scale type
  const uint8_t* intervals = scaleTypes[scaleIndex];
  
  // Generate multiple octaves of the scale
  uint8_t noteIndex = 0;
  for (uint8_t octave = 0; octave < SCALE_OCTAVES; octave++) {
    for (uint8_t i = 0; i < scaleLength; i++) {
      if (noteIndex < MAX_SCALE_NOTES) {  // Prevent array overflow
        // Calculate MIDI note: root + scale interval + octave offset
        generatedScale[noteIndex] = root + intervals[i] + (octave * OCTAVE_SEMITONES);
        noteIndex++;
      }
    }
  }
  
  // Debug output if serial communication is enabled
  #if USE_SERIAL
    if (Serial) {
      Serial.print(F("Generated "));
      Serial.print(scaleNames[scaleIndex]);
      Serial.print(F(" scale from root MIDI "));
      Serial.print(root);
      Serial.print(F(" ("));
      Serial.print(noteIndex);
      Serial.println(F(" notes)"));
    }
  #endif
}

//////////////////////////////////////////////////
//  A U D I O   O U T P U T //
//////////////////////////////////////////////////
#pragma region Audio Output

/**
 * @brief Updates oscillator frequencies based on current wave selection
 * 
 * Step-by-step process:
 * 1. Checks current waveform selection (wave2)
 * 2. Sets appropriate oscillator frequencies for the selected waveform
 * 3. Applies harmonic relationships (octaves, etc.) for richer sound
 * 
 * Wave Types:
 * - TRIANGLE (0): Sine base + triangle harmonics + octave doubling
 * - SQUARE (1): Sine base + triangle harmonics (smooth square approximation)
 * - SAW (2): Sine base + sawtooth harmonics
 * - NOISE (3): Sine base + square harmonics with octave doubling
 * 
 * Why: Different oscillator combinations create distinct timbres and textures
 * Performance: Only updates oscillators needed for current waveform
 */
void setFrequencies(){
  switch (wave2) {
   case TRIANGLE:  // Square wave approximation with sine + triangle
      aSin1.setFreq(frequency1);
      aSin2.setFreq(frequency2);
      aTri1.setFreq(frequency1);
      aTri2.setFreq(frequency2);
     break;
     
   case SQUARE:  // Noise-like texture with sine + square + octave doubling
      aSin1.setFreq(frequency1);
      aSin2.setFreq(frequency2);
      aSqu1.setFreq(frequency1);
      aSqu2.setFreq(frequency2 * 2);  // Octave doubling for complexity
     break;
     
   case SAW:  // Sawtooth with sine harmonics
      aSin1.setFreq(frequency1);
      aSin2.setFreq(frequency2);
      aSaw1.setFreq(frequency1);
      aSaw2.setFreq(frequency2);
     break;
     
   case SINE:  // Pure triangle with sine base + octave harmonics
   default:
      aSin1.setFreq(frequency1);
      aSin2.setFreq(frequency2);
      aSin3.setFreq(frequency1 * 2);  // Octave harmonic
      aSin4.setFreq(frequency2 * 2);  // Octave harmonic
    break;
 }
}

/**
 * @brief Audio generation function - called at AUDIO_RATE (16384 Hz)
 * 
 * Step-by-step process:
 * 1. Generates audio samples from active oscillators based on waveform selection
 * 2. Applies individual volume controls to each oscillator
 * 3. Mixes all oscillator outputs into final audio signal
 * 4. Converts to stereo output with configured bit depth and clipping
 * 
 * Wave Mixing:
 * - Each waveform uses different oscillator combinations for unique timbre
 * - Volume controls allow independent level adjustment of each voice
 * - Bit shifting (>>1) reduces volume of harmonic oscillators to prevent clipping
 * 
 * Why: Real-time audio generation requires precise sample-by-sample mixing
 * Performance: Critical function - must complete within audio sample period
 * 
 * @return Stereo audio output with AUDIO_OUTPUT_BITS resolution
 */
AudioOutput_t updateAudio(){
  switch (wave2) {
   case TRIANGLE:  // Square wave approximation: sine + triangle blend
    asig = (long)
      aSin1.next() * volume1 +
      aSin2.next() * volume3 +
      aTri1.next() * volume2 +
      aTri2.next() * volume4;
     break;
     
   case SQUARE:  // Noise texture: sine + square with harmonic reduction
    asig = (long)
      aSin1.next() * volume1 +
      aSin2.next() * volume3 +
      aSqu1.next() * volume2 +
      aSqu2.next() * (volume4 >> 1);  // Reduce octave volume to prevent clipping
     break;
     
   case SAW:  // Sawtooth: sine + saw blend
    asig = (long)
      aSin1.next() * volume1 +
      aSin2.next() * volume3 +
      aSaw1.next() * volume2 +
      aSaw2.next() * volume4;
     break;

   case SINE:  // Pure triangle with octave harmonics
   default:
      asig = (long)
      aSin1.next() * volume1 +
      aSin2.next() * volume3 +
      aSin3.next() * (volume2 >> 1) +  // Reduce octave volume
      aSin4.next() * (volume4 >> 1);   // Reduce octave volume
     break;
 }
  return StereoOutput::fromAlmostNBit(AUDIO_OUTPUT_BITS, asig, asig).clip();
}
#pragma endregion Audio Output
#pragma endregion FUNCTIONS

//////////////////////////////////////////////////
//  E N D   C O D E  //
//////////////////////////////////////////////////


