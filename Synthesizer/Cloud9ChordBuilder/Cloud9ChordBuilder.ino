#pragma region README
/*
  ************************************************************************************
  *
  * ~ ~ ~ ~ ~ ~ ~ ~ ~ ~Welcome to the CLOUD 9 chord builder~ ~ ~ ~ ~ ~ ~ ~ ~ ~
  * This 5 voice polyphonic synthesizer program is complete with 8 different waveforms, 11 chord tables, and 88 total chord types!
  * 
  * Features:
  * • 11 Musical Tables: Classic, Innocent, Sorrow, Triumph, Mysterious, etc. (8 chords each)
  * • 88 Total chord types including Major, minor, 7ths, sus, dim, aug, add9, modal, and more
  * • Semitone-based system (easy to understand and modify by adding semitones to root MIDI note)
  * • Table switching for different musical moods and emotional journeys
  * • Smooth voice leading and inversions
  * • Multiple waveforms (sin, saw, square, triangle, etc.) 
      - try making your own submixes by crafting unique combinations of waves, 
      - just make sure the case you change matches in both setFrequencies and updateAudio
  *
  * CONTROLS:
  * • X-axis (touch): Select chord (0-7 within current table)
  * • Y-axis (touch): Control inversions and voicings  
 * • botPot (A5): Wave selection (sin, saw, square, triangle, etc.)
 * • midPot (A6): Root frequency control (MIDI 60-96, C4-C7)
 * • hiPot (A7): Volume control 
 * • Proximity sensor: Reduces volume when triggered (trilling effect) OR LFO modulation
 * • LFO Mode: Fidget spinner RPM converted to musical BPM for smooth volume modulation
    - for all of the above: try swapping controls or putting multiple parameters on one controller
    - experiment with different mappings, what if a knob could change our chord table by mapping to current table?
  *
 * CONFIGURATION:
 * • All user-configurable settings are in config.h
 * • Touchpad calibration, musical ranges, smoothing factors, and LFO settings
 * • Safe value ranges provided in comments for easy customization
 * 
 * CUSTOMIZATION GUIDE:
 * • Touchpad feel: Adjust smoothing rates on frequencies up or down for more slidey or discrete feel
 * • note from Dan: try out some different mappings! Instead of just attaching y-Axis to inversion, you could instead use it to control LFO rate, for example
 * • I use the touchpad X and Y here as distinct values, but it could be interesting to try out a more zone based approach
 * • also try working with an LLM to generate your own chord tables! if you paste an example in and tell it what sort of feeling or style of music you want to evoke, it's pretty good
 * • Response speed: Modify SMOOTH_* constants in config.h (lower = faster, higher = smoother)
 * • LFO behavior: Tune LFO_* settings in config.h for fidget spinner response
  *
  ************************************************************************************
  */
#pragma endregion README

#pragma region LICENSE
/*
  ************************************************************************************
  * See LICENSE files - this program was built on the incredible Mozzi library, by Tim Barrass and Contributors, and thus inherits its license
  * Copyright (c) 2025 Crunchlabs LLC (Laser Synthesizer Code) by Dan Tompkins
  ************************************************************************************
*/
#pragma endregion LICENSE

//////////////////////////////////////////////////
//  INCLUDES AND CONFIGURATION //
//////////////////////////////////////////////////
#pragma region INCLUDES

// Configuration files
#include "config.h"

// Mozzi audio library setup
#include <MozziConfigValues.h>
#define MOZZI_AUDIO_CHANNELS MOZZI_STEREO
#include <MozziGuts.h>
#include <Oscil.h>
#include <mozzi_fixmath.h>
#include <Smooth.h>
#include <mozzi_midi.h>

// Project-specific includes
#include "wavetables.h"
#include "chord_tables.h"

#pragma endregion INCLUDES

//////////////////////////////////////////////////
//  SYSTEM CONFIGURATION - MODIFY WITH CAUTION //
//////////////////////////////////////////////////
#pragma region SYSTEM CONFIG

/*
  ************************************************************************************
  *
  * SYSTEM CONFIGURATION - Internal System Constants
  * 
  * This section contains core system constants that define the synthesizer's
  * fundamental behavior. These values are carefully tuned for optimal performance
  * and should generally not be modified unless you understand the implications.
  * 
  * MODIFY WITH CAUTION: Changing these values may cause system instability
  * 
  ************************************************************************************
*/

// === CORE AUDIO ENGINE ===
#define CONTROL_RATE 64           // Control update rate in Hz (MUST be 64 for this sketch)
#define DEFAULT_FREQ 110          // Default oscillator frequency in Hz
#define ADC_MAX_VALUE 1023        // Maximum ADC reading value
#define WAVE_COUNT 8              // Number of available waveforms (0-7)
#define CHORD_COUNT 8             // Number of chords per table (0-7)
#define INVERSION_COUNT 16        // Number of inversion patterns (0-15)

// === MIDI AND FREQUENCY ===
#define OCTAVE_SEMITONES 12       // Semitones per octave
#define MAX_OCTAVE_SHIFT 2        // Maximum octave shift for inversions

// === TOUCHPAD MAPPING ===
#define INVERSION_MAP_MIN 6       // Minimum inversion mapping value
#define INVERSION_MAP_MAX 15      // Maximum inversion mapping value

#pragma endregion SYSTEM CONFIG

//////////////////////////////////////////////////
//  INVERSION PATTERNS DATA //
//////////////////////////////////////////////////
#pragma region INVERSION PATTERNS

// Inversion patterns array - much more efficient than switch statement!
// Each row represents an inversion pattern: [inv_aply1, inv_aply2, inv_aply3, inv_aply4, inv_aply5]
// Values: 0 = no octave shift, 1 = +1 octave, 2 = +2 octaves
const uint8_t inversion_patterns[16][5] PROGMEM = {
  {0, 0, 0, 0, 0},  // 0: Root position - no inversions
  {1, 0, 0, 0, 0},  // 1: First voice up an octave
  {1, 1, 0, 0, 0},  // 2: First two voices up an octave
  {1, 1, 1, 0, 0},  // 3: First three voices up an octave
  {1, 1, 1, 1, 0},  // 4: First four voices up an octave
  {2, 1, 1, 1, 0},  // 5: Root up 2 octaves, others up 1
  {0, 2, 2, 2, 1},  // 6: First two up 2 octaves, next two up 1
  {1, 2, 2, 2, 1},  // 7: First three up 2 octaves, fourth up 1
  {2, 2, 2, 1, 1},  // 8: Wide spread with 5th voice active
  {2, 2, 1, 1, 1},  // 9: Compressed spread with 5th voice active
  {2, 1, 1, 1, 1},  // 10: Root emphasis with 5th voice active
  {1, 1, 1, 1, 1},  // 11: All voices up an octave with 5th active
  {1, 1, 1, 0, 1},  // 12: Skip fourth voice, 5th active
  {1, 1, 0, 0, 1},  // 13: Skip third and fourth, 5th active
  {1, 0, 0, 0, 1},  // 14: Just first and fifth voices up
  {0, 0, 0, 0, 1}   // 15: Only 5th voice active (octave drone)
};

#pragma endregion INVERSION PATTERNS

//////////////////////////////////////////////////
//  SMOOTH OBJECTS //
//////////////////////////////////////////////////
#pragma region Smooth Objects
// Smooth objects for frequency control - using defined constants for consistency
Smooth<float> kSmoothFreq1(SMOOTH_FREQ_NORMAL);   // Main frequency smoothing
Smooth<float> kSmoothFreq2(SMOOTH_FREQ_NORMAL);   // Chord voice 2 smoothing
Smooth<float> kSmoothFreq3(SMOOTH_FREQ_NORMAL);   // Chord voice 3 smoothing
Smooth<float> kSmoothFreq4(SMOOTH_FREQ_NORMAL);   // Chord voice 4 smoothing
Smooth<float> kSmoothFreq5(SMOOTH_FREQ_SLOW);     // Chord voice 5 smoothing (slower for stability)
Smooth<float> kSmoothGain1(SMOOTH_GAIN_NORMAL);   // gain smoothing (normal attack)
Smooth<float> kSmoothGainFadeIn(SMOOTH_GAIN_FAST);    // Fast fade-in when touching pad
Smooth<float> kSmoothGainFadeOut(SMOOTH_GAIN_SLOW);   // Slower fade-out when releasing pad
#pragma endregion Smooth Objects

//////////////////////////////////////////////////
//  GLOBAL VARIABLES //
//////////////////////////////////////////////////
#pragma region Global Variables

// === AUDIO ENGINE STATE ===
int freq1 = DEFAULT_FREQ;  // base freq of OSC1
long asig;                 // our audio signal (for later audio output)

// Oscillator frequency values - all initialized to default frequency
int freqv1 = DEFAULT_FREQ;  // Voice 1 frequency
int freqv2 = DEFAULT_FREQ;  // Voice 2 frequency  
int freqv3 = DEFAULT_FREQ;  // Voice 3 frequency
int freqv4 = DEFAULT_FREQ;  // Voice 4 frequency
int freqv5 = DEFAULT_FREQ;  // Voice 5 frequency

// === INPUT AND CONTROL STATE ===
// Raw ADC input variables (0-1023 range)
uint16_t botPot = 0;  // Wave selection potentiometer
uint16_t midPot = 0;  // Root frequency control potentiometer
uint16_t hiPot = 0;   // Volume control potentiometer

// Semantic control variables (processed from raw inputs with hysteresis)
uint8_t waveSet = 0;   // Wave selection (0 to WAVE_COUNT-1)
uint16_t gainSet = 0;  // Gain/volume level (0 to ADC_MAX_VALUE)
uint8_t rootSet = 62;  // Root MIDI note (MIDI_ROOT_MIN to MIDI_ROOT_MAX)
uint8_t chordSet = 0;  // Chord selection (0 to CHORD_COUNT-1)
uint8_t invSet = 0;    // Inversion selection (0 to INVERSION_COUNT-1)

// Touchpad state variables
bool axis = false;     // Alternates between X and Y coordinate reading
int yVal, xVal;        // Current touchpad coordinates
int xBuff, yBuff;      // Buffered touchpad coordinates (legacy)

// Serial output control
int cmd_count = 0;     // Loop counter for serial output rate limiting

// === MUSICAL STATE ===
// Musical note variables (semitones above root)
byte note1 = 0; // Root note
byte note2 = 0; // 2nd voice note
byte note3 = 0; // 3rd voice note
byte note4 = 0; // 4th voice note
byte note5 = 0; // 5th voice note (root octave)

// Inversion application variables (octave shifts: 0, 1, or 2)
uint8_t inv_aply1 = 0; // Root voice octave shift
uint8_t inv_aply2 = 0; // 2nd voice octave shift
uint8_t inv_aply3 = 0; // 3rd voice octave shift
uint8_t inv_aply4 = 0; // 4th voice octave shift
bool inv_aply5 = 0;    // 5th voice enable

// Current musical state
uint8_t inv = 0;        // Current inversion index (0 to INVERSION_COUNT-1)
uint8_t chord = 0;      // Current chord index (0 to CHORD_COUNT-1)
uint8_t wave1 = 0;      // Current wave selection (0 to WAVE_COUNT-1)
uint8_t current_table = DEFAULT_CHORD_TABLE;  // Current chord table (0-10)

// Audio gain variables
int gain = 0;           // Master output gain
int bassGain = 0;       // Bass frequency gain
int trebleGain = 0;     // Treble frequency gain
int targetGain = 0;     // Target gain for smooth transitions
uint8_t prox = 0;       // Proximity sensor state (0 or 1)

// Target frequency tracking
float targetNote1 = 0;  // Target frequency for smooth transitions

// Finger-off state tracking
bool fingerOff = false;           // Track if finger is off touchpad
uint8_t frozenChord = 0;         // Frozen chord when finger off
uint8_t frozenInv = 0;           // Frozen inversion when finger off

// Hysteresis state variables (prevent control jitter)
int8_t lastChord = -1;     // Last chord for hysteresis tracking
int8_t lastInv = -1;       // Last inversion for hysteresis tracking
int8_t lastWave = -1;      // Last wave for hysteresis tracking

// Hysteresis thresholds (using defined constants)
uint8_t chordHysteresis = CHORD_HYSTERESIS;      // Chord selection dead zone
uint8_t invHysteresis = INVERSION_HYSTERESIS;    // Inversion selection dead zone
uint8_t potHysteresis = POT_HYSTERESIS;          // Potentiometer change threshold

// === LFO SYSTEM ===
uint16_t proxTriggerCount = 0;         // Control rate counter for trigger timing
uint16_t proxInterval = 0;             // Time between proximity triggers (control cycles)
float spinnerRPM = 0;                  // Calculated RPM of fidget spinner
float lfoBPM = LFO_MIN_BPM;            // LFO rate in beats per minute
float lfoFreq = 1.0;                   // LFO frequency in Hz
uint16_t decayCounter = 0;             // Counter for LFO decay timing

// Node hole filtering variables (using defined constants)
uint8_t proxLowCount = 0;              // Count consecutive LOW readings
uint8_t proxHighCount = 0;             // Count consecutive HIGH readings  
bool validNodeTrigger = false;         // True when we have a valid node (not hole)

#pragma endregion Global Variables

//////////////////////////////////////////////////
//  SETUP //
//////////////////////////////////////////////////
#pragma region SETUP
void setup()
{
 startMozzi(CONTROL_RATE);

 // Initialize LFO system
 kLFO.setFreq(1.0f);  // Start with 1 Hz (60 BPM)

 #if USE_SERIAL
   Serial.begin(9600);
   Serial.println(F("CHORD BUILDER SYNTH - STARTING UP"));
   Serial.println(F("Features: 11 Tables, 88 Chord Types, Multi-Waveform"));
   Serial.println(F("Controls: Touch=Chord/Inversions, Pots=Wave/Freq/Vol"));
   Serial.println(F("LFO: Fidget spinner modulation available"));
   Serial.println(F("=========================================="));
 #endif
}
#pragma endregion SETUP

//////////////////////////////////////////////////
//  CONTROL FUNCTION IMPLEMENTATIONS //
//////////////////////////////////////////////////
#pragma region CONTROL FUNCTIONS

/**
 * @brief Reads all input sensors and converts them to semantic control variables
 * 
 * Step-by-step process:
 * 1. Alternates between reading X and Y touchpad coordinates (prevents interference)
 * 2. Constrains touchpad values to calibrated ranges
 * 3. Reads all three potentiometers using fast Mozzi ADC
 * 4. Reads proximity sensor for fidget spinner detection
 * 5. Updates LFO system if enabled (tracks spinner speed)
 * 6. Converts raw inputs to semantic variables (wave, gain, root note, chord, inversion)
 * 7. Provides easy remapping points for different control schemes
 * 
 * Why: This centralizes all input processing and makes control remapping simple
 */
void readInputs() {
  // Alternates between X and Y coordinate reading - resistive touchpad requires separate readings to prevent interference
  if (axis){
    yVal = ycoor();
    yVal = constrain(yVal, TOUCHPAD_Y_MIN, TOUCHPAD_Y_MAX);
    axis = !axis;
  }else{
    xVal = xcoor();
    xVal = constrain(xVal, TOUCHPAD_X_MIN, TOUCHPAD_X_MAX);
    axis = !axis;
  }
  // Use Mozzi's optimized ADC for faster potentiometer readings
  botPot = mozziAnalogRead<10>(POT_PIN1);  // bottom knob, controls Wave selection by default (analog pin 5)
  midPot = mozziAnalogRead<10>(POT_PIN2);  // middle knob, sets root note by default (analog pin 5)
  hiPot = mozziAnalogRead<10>(POT_PIN3);  // top knob, controls gain by default (analog pin 5)

  prox = digitalRead(PROX_PIN); // Read proximity sensor - used for direct volume control or LFO timing (digital pin 6)

  // Update LFO system if in LFO mode
  if (LFO_MODE_ENABLED) {
    updateSpinnerLFO();
  }

  // Convert raw inputs to semantic control variables - easy remapping point
  // Example remapping: waveSet = 0; current_table = botPot>>7; (fixed wave, pot controls table)
  waveSet = botPot >> 7; // bitshift ADC_MAX_VALUE to 0-(WAVE_COUNT-1) range
  gainSet = hiPot;       // full ADC range for gain control
  rootSet = map(midPot, 0, ADC_MAX_VALUE, MIDI_ROOT_MIN, MIDI_ROOT_MAX);  // MIDI note range
  chordSet = map(xVal, TOUCHPAD_X_MIN, TOUCHPAD_X_MAX, 0, CHORD_COUNT-1); // chord selection
  invSet = map(yVal, TOUCHPAD_Y_MIN, TOUCHPAD_Y_MAX, INVERSION_MAP_MAX, INVERSION_MAP_MIN); // inversion patterns
  //invSet = map(yVal, 0, 925, 0, 9);//alternative inversion mapping for different feel
  
  // LFO MODE TOGGLE - uncomment one of these lines to enable LFO mode:
  // In LFO mode, fidget spinner speed sets LFO rate for smooth gain modulation (breathing effect)
  // In direct mode, proximity sensor immediately reduces volume (trilling effect)
  // LFO_MODE_ENABLED = true;  // Always use LFO mode
  //LFO_MODE_ENABLED = (midPot > 512);  // Toggle LFO with middle pot (above halfway)
  // LFO_MODE_ENABLED = (current_table >= 5);  // Use LFO for certain chord tables
}

/**
 * @brief Controls volume/gain with finger-off detection and LFO modulation
 * 
 * Step-by-step process:
 * 1. Detects if finger is off touchpad using edge coordinates
 * 2. If finger just went off: freezes current musical state, sets target gain to zero
 * 3. If finger is on pad: determines target gain based on mode
 *    - LFO Mode: Applies smooth modulation from fidget spinner speed
 *    - Direct Mode: Uses proximity sensor for immediate volume reduction
 * 4. Applies smooth fade-in when touching, smooth fade-out when releasing
 * 5. Prevents audio pops and maintains musical continuity
 * 
 * Why: Creates smooth volume transitions and prevents jarring changes when finger moves
 */
void gainControl() {
  // Better finger-off detection: check if coordinates are outside normal touch range
  bool fingerOffDetected = (yVal > TOUCHPAD_EDGE_DETECT) || (xVal < TOUCHPAD_X_EDGE_MIN) || (xVal > TOUCHPAD_X_EDGE_MAX);
  
  if (fingerOffDetected){
    // Finger off pad - freeze musical state and set target to zero
    if (!fingerOff) {
      // Just went finger-off, freeze current musical state
      fingerOff = true;
      frozenChord = chord;
      frozenInv = inv;
    }
    targetGain = 0; // Set target to zero for fade-out
  }else{
    // Finger on pad - unfreeze and determine target gain
    fingerOff = false;
    
    if (LFO_MODE_ENABLED) {
      // LFO mode: use spinner-derived LFO for smooth modulation
      float lfoMod = getLFOModulation();  // 0.5 to 1.5 multiplier
      float lfoGain = gainSet * lfoMod;   // Apply LFO modulation to base gain
      targetGain = constrain(lfoGain, 0, 1023);  // Constrain to valid range
    } else {
      // Direct proximity mode (original behavior)
      if (prox == LOW){//fidget spinner modulating gain for trilling effect 
        //**NOTE: sometimes sunlight can trigger prox sensor and trigger this to lower volume
        targetGain = gainSet>>2;//bitshift divide 0-1023/(2^2) = 0 - 255
      }else{
        targetGain = gainSet; // full 0-1023 range
      }
    }
  }
  
  // Always use the same smoother, but with different targets
  // This allows proper fade-in and fade-out behavior
  if (fingerOff) {
    gain = kSmoothGainFadeOut.next(targetGain); // Slow fade to zero
  } else {
    gain = kSmoothGainFadeIn.next(targetGain);  // Fast fade to target
  }
}

/**
 * @brief Selects chords and inversions with hysteresis to prevent jitter
 * 
 * Step-by-step process:
 * 1. If finger is off: uses frozen chord/inversion values to prevent noise
 * 2. If finger is on: processes new chord and inversion with hysteresis
 * 3. Hysteresis prevents rapid switching between adjacent zones
 * 4. Only changes chord/inversion when movement exceeds threshold
 * 5. Applies selected inversion pattern from lookup table
 * 6. Sets octave shifts for each voice (0, 1, or 2 octaves up)
 * 
 * Why: Prevents musical chaos from small finger movements while maintaining responsiveness
 */
void setChordAndInversions() {
  if (fingerOff) {
    // Finger off - use frozen values to prevent touchpad noise from changing the chord
    chord = frozenChord;
    inv = frozenInv;
  } else {
    // Finger on - normal chord/inversion processing with hysteresis
    // Chord selection with hysteresis to prevent flickering between adjacent zones
    uint8_t newChord = chordSet;
    
    // Apply hysteresis to chord selection
    if (lastChord == -1) {
      // First time, just set the chord
      chord = newChord;
      lastChord = chord;
    } else if (abs(chordSet - (150 + (lastChord * (740-150)/7))) > chordHysteresis) {
      // Only change chord if we're far enough from the last chord boundary
      chord = newChord;
      lastChord = chord;
    }
    // Otherwise keep the current chord (hysteresis effect)

    // Inversion selection with hysteresis to prevent rapid switching
    uint8_t newInv = invSet;
    
    // Apply hysteresis to inversion selection
    if (lastInv == -1) {
      // First time, just set the inversion
      inv = newInv;
      lastInv = inv;
    } else if (abs(invSet - (lastInv * 910/135)) > invHysteresis) {
      // Only change inversion if we're far enough from the last inversion boundary
      inv = newInv;
      lastInv = inv;
    }
    // Otherwise keep the current inversion (hysteresis effect)
  }

  // Apply inversion pattern (always do this, whether frozen or not)
  inv = constrain(inv, 0, 15);  // Ensure inv is within array bounds
  inv_aply1 = pgm_read_byte(&inversion_patterns[inv][0]);
  inv_aply2 = pgm_read_byte(&inversion_patterns[inv][1]);
  inv_aply3 = pgm_read_byte(&inversion_patterns[inv][2]);
  inv_aply4 = pgm_read_byte(&inversion_patterns[inv][3]);
  inv_aply5 = pgm_read_byte(&inversion_patterns[inv][4]);
}

/**
 * @brief Processes root frequency and wave selection with smooth transitions
 * 
 * Step-by-step process:
 * 1. Reads chord notes from current table using selected chord index
 * 2. Sets root frequency from MIDI note with smooth interpolation
 * 3. Applies wave selection with hysteresis to prevent switching jitter
 * 4. Updates target frequency for smooth frequency changes
 * 5. Prepares note data for oscillator frequency calculation
 * 
 * Why: Provides smooth musical transitions and prevents audio artifacts from rapid changes
 */
void processFrequencyAndWave() {
  // Read chord notes (semitones above root) from current chord table
  note1 = (pgm_read_byte(&(chord_tables[current_table][chord][0])));
  note2 = (pgm_read_byte(&(chord_tables[current_table][chord][1])));
  note3 = (pgm_read_byte(&(chord_tables[current_table][chord][2])));
  note4 = (pgm_read_byte(&(chord_tables[current_table][chord][3])));
  note5 = (pgm_read_byte(&(chord_tables[current_table][chord][0]))); // Root octave

  // Root frequency control - chromatic MIDI notes within configured range
  // Uses MIDI_ROOT_MIN to MIDI_ROOT_MAX for optimal range without going too low/high
  int rootNote = rootSet;
  targetNote1 = mtof(rootNote);
  int xfreq1 = xfreq1*0.9 + targetNote1*0.1;
  freq1 = xfreq1;
  
  // Wave selection with hysteresis to prevent jitter
  uint8_t newWave = waveSet;
  
  // Apply hysteresis to wave selection
  if (lastWave == -1) {
    // First time, just set the wave
    wave1 = newWave;
    lastWave = wave1;
  } else if (abs(botPot - (lastWave * 128)) > potHysteresis) {
    // Only change wave if we're far enough from the last wave boundary
    wave1 = newWave;
    lastWave = wave1;
  }
  // Otherwise keep the current wave (hysteresis effect)
}

//////////////////////////////////////////////////
//  TOUCHPAD HELPER FUNCTIONS //
//////////////////////////////////////////////////

/**
 * @brief Reads Y coordinate from resistive touchpad
 * 
 * Step-by-step process:
 * 1. Sets A1 and pin 8 as outputs (voltage divider)
 * 2. Sets A0 and pin 7 as inputs (voltage sensing)
 * 3. Applies voltage across Y-axis of touchpad
 * 4. Reads voltage at touch point using fast ADC
 * 5. Returns Y coordinate (0-1023 range)
 * 
 * Why: Resistive touchpad requires specific pin configuration for each axis
 */
int ycoor(){
  pinMode(A1,OUTPUT);
  pinMode(8,OUTPUT);
  pinMode(A0,INPUT);
  pinMode(7,INPUT);
  digitalWrite(A1,HIGH);
  digitalWrite(8,LOW);
  return mozziAnalogRead<10>(A0);
}

/**
 * @brief Reads X coordinate from resistive touchpad
 * 
 * Step-by-step process:
 * 1. Sets A0 and pin 7 as outputs (voltage divider)
 * 2. Sets A1 and pin 8 as inputs (voltage sensing)
 * 3. Applies voltage across X-axis of touchpad
 * 4. Reads voltage at touch point using fast ADC
 * 5. Returns X coordinate (0-1023 range)
 * 
 * Why: X-axis requires different pin configuration than Y-axis
 */
int xcoor(){
  pinMode(A0,OUTPUT);
  pinMode(7,OUTPUT);
  pinMode(A1,INPUT);
  pinMode(8,INPUT);
  digitalWrite(A0,LOW);
  digitalWrite(7,HIGH);
  return mozziAnalogRead<10>(A1);
}

//////////////////////////////////////////////////
//  LFO SYSTEM FUNCTIONS //
//////////////////////////////////////////////////

/**
 * @brief Tracks fidget spinner RPM and converts to musical BPM for LFO modulation
 * 
 * Step-by-step process:
 * 1. Filters proximity sensor to ignore holes between spinner nodes
 * 2. Detects valid node triggers using minimum pulse width
 * 3. Measures time between consecutive node detections
 * 4. Calculates spinner RPM from node timing and node count
 * 5. Converts RPM to musical BPM with 2x multiplier for feel
 * 6. Updates LFO oscillator frequency for smooth modulation
 * 7. Applies decay when spinner slows down or stops
 * 
 * Why: Creates smooth, musical modulation from physical spinner motion
 */
void updateSpinnerLFO() {
  static bool lastValidNodeState = false;
  static uint16_t lastTriggerTime = 0;
  bool currentProxState = (prox == LOW);
  
  // Increment counters every control cycle
  proxTriggerCount++;
  decayCounter++;
  
  // Filter out node holes by requiring minimum pulse width
  if (currentProxState) {
    proxLowCount++;
    proxHighCount = 0;
    // Valid node trigger if we've been LOW for minimum width
    if (proxLowCount >= LFO_MIN_NODE_WIDTH) {
      validNodeTrigger = true;
    }
  } else {
    proxHighCount++;
    proxLowCount = 0;
    // Reset valid trigger if we've been HIGH for a while (end of node)
    if (proxHighCount >= LFO_MIN_NODE_WIDTH) {
      validNodeTrigger = false;
    }
  }
  
  // Detect valid node trigger (rising edge of filtered signal)
  if (validNodeTrigger && !lastValidNodeState) {
    // Calculate interval since last valid trigger (in control cycles)
    if (lastTriggerTime > 0) {
      proxInterval = proxTriggerCount - lastTriggerTime;
      
      // Convert control cycles to milliseconds (CONTROL_RATE = 64 Hz)
      float intervalMs = (proxInterval * 1000.0f) / CONTROL_RATE;
      
      // Calculate RPM: 60000ms/min ÷ (interval × nodes) = RPM
      if (intervalMs > 50 && intervalMs < 5000) {  // Sanity check (50ms to 5s per node)
        spinnerRPM = 60000.0f / (intervalMs * LFO_SPINNER_NODES);
        
        // Map RPM to musical BPM (with constraints)
        lfoBPM = constrain(spinnerRPM * 2.0f, LFO_MIN_BPM, LFO_MAX_BPM);  // 2x multiplier for musical feel
        
        // Convert BPM to Hz for oscillator
        lfoFreq = lfoBPM / 60.0f;
        kLFO.setFreq(lfoFreq);
        
        // Reset decay counter since we got a fresh trigger
        decayCounter = 0;
      }
    }
    
    // Record this trigger time
    lastTriggerTime = proxTriggerCount;
  }
  
  lastValidNodeState = validNodeTrigger;
  
  // Decay LFO frequency if no triggers for a while (spinner stopping)
  if (decayCounter > LFO_DECAY_TIMEOUT) {
    lfoBPM = lfoBPM * 0.998f;  // Slower decay for smoother transition
    if (lfoBPM < LFO_MIN_BPM) lfoBPM = LFO_MIN_BPM;
    lfoFreq = lfoBPM / 60.0f;
    kLFO.setFreq(lfoFreq);
    
    // Reset decay counter for next decay cycle
    decayCounter = LFO_DECAY_TIMEOUT - 16;  // Check every 16 cycles for smoother decay
  }
}

/**
 * @brief Gets LFO modulation value for smooth gain control
 * 
 * Step-by-step process:
 * 1. Reads current LFO oscillator output (-127 to +127)
 * 2. Normalizes to -1.0 to +1.0 range
 * 3. Scales to 0.5 to 1.5 multiplier range
 * 4. Returns smooth modulation value for gain multiplication
 * 
 * Why: Provides smooth volume breathing effect instead of abrupt on/off
 * @return LFO modulation multiplier (0.5 to 1.5 range)
 */
float getLFOModulation() {
  // Get LFO output (-127 to +127) and scale to 0.5-1.5 range for smooth modulation
  int lfoValue = kLFO.next();
  float lfoNormalized = lfoValue / 127.0f;  // Convert to -1.0 to +1.0
  return 1.0f + (lfoNormalized * 0.5f);     // Convert to 0.5 to 1.5 multiplier
}

//////////////////////////////////////////////////
//  SERIAL OUTPUT FUNCTIONS //
//////////////////////////////////////////////////

/**
 * @brief Prints comprehensive synthesizer status for monitoring and debugging
 * 
 * Step-by-step process:
 * 1. Outputs current chord table name and index
 * 2. Shows selected chord and inversion numbers
 * 3. Displays current wave type and root note
 * 4. Reports gain level and proximity sensor status
 * 5. Shows LFO mode and BPM if active
 * 6. Outputs current touchpad coordinates
 * 
 * Why: Essential for debugging control issues and monitoring synth state
 */
void printSynthData() {
  Serial.print(F("Table: "));
  Serial.print(table_names[current_table]);
  Serial.print(F(", Chord: "));
  Serial.print(chord);
  Serial.print(F(", Inv: "));
  Serial.print(inv);
  Serial.print(F(", Wave: "));
  Serial.print(wave1);
  Serial.print(F(", Root: "));
  Serial.print(rootSet);
  Serial.print(F(", Gain: "));
  Serial.print(gain);
  Serial.print(F(", Prox: "));
  Serial.print(prox ? F("OFF") : F("ON"));
  Serial.print(F(", LFO: "));
  Serial.print(LFO_MODE_ENABLED ? F("ON") : F("OFF"));
  if (LFO_MODE_ENABLED) {
    Serial.print(F(" ("));
    Serial.print(lfoBPM, 1);
    Serial.print(F("BPM)"));
  }
  Serial.print(F(",Touch X: "));
  Serial.print(xVal);
  Serial.print(F(", Y: "));
  Serial.println(yVal);
}

/**
 * @brief Prints raw control input values for calibration and analysis
 * 
 * Step-by-step process:
 * 1. Outputs raw potentiometer readings (0-1023)
 * 2. Shows processed semantic control values
 * 3. Displays mapped ranges for each control
 * 4. Useful for touchpad calibration and control tuning
 * 
 * Why: Helps diagnose input issues and calibrate control ranges
 */
void printControlData() {
  Serial.print(F("botPot:"));
  Serial.print(botPot);
  Serial.print(F(",midPot:"));
  Serial.print(midPot);
  Serial.print(F(",hiPot:"));
  Serial.print(hiPot);
  Serial.print(F(",waveSet:"));
  Serial.print(waveSet);
  Serial.print(F(",rootSet:"));
  Serial.print(rootSet);
  Serial.print(F(",gainSet:"));
  Serial.print(gainSet);
  Serial.print(F(",chordSet:"));
  Serial.print(chordSet);
  Serial.print(F(",invSet:"));
  Serial.println(invSet);
}

/**
 * @brief Prints current musical note data and inversion settings for analysis
 * 
 * Step-by-step process:
 * 1. Outputs all five chord voice notes (semitones above root)
 * 2. Shows octave shift settings for each voice (0, 1, or 2 octaves)
 * 3. Displays current inversion pattern being applied
 * 4. Useful for understanding chord voicings and debugging inversions
 * 
 * Why: Essential for analyzing harmonic content and inversion behavior
 */
void printNoteData() {
  Serial.print(F("Notes:"));
  Serial.print(note1);
  Serial.print(F(","));
  Serial.print(note2);
  Serial.print(F(","));
  Serial.print(note3);
  Serial.print(F(","));
  Serial.print(note4);
  Serial.print(F(","));
  Serial.print(note5);
  Serial.print(F(",Inversions:"));
  Serial.print(inv_aply1);
  Serial.print(F(","));
  Serial.print(inv_aply2);
  Serial.print(F(","));
  Serial.print(inv_aply3);
  Serial.print(F(","));
  Serial.print(inv_aply4);
  Serial.print(F(","));
  Serial.println(inv_aply5);
}

#pragma endregion CONTROL FUNCTIONS

//////////////////////////////////////////////////
//  MAIN CONTROL FUNCTION //
//////////////////////////////////////////////////
#pragma region MAIN CONTROL

/**
 * @brief Main control update function called 64 times per second
 * 
 * Step-by-step process:
 * 1. Reads all inputs (touchpad, pots, proximity sensor)
 * 2. Processes gain control with finger detection and LFO
 * 3. Handles chord and inversion selection with hysteresis
 * 4. Updates frequency and wave settings
 * 5. Calculates and sets all oscillator frequencies
 * 6. Outputs serial data at reduced rate to avoid audio interference
 * 
 * Why: Coordinates all control processing in the correct order for smooth audio
 * Note: No delay() calls allowed - they would disrupt audio generation
 */
void updateControl() {
  // Clean, organized control flow using subfunctions
  readInputs();
  gainControl();
  setChordAndInversions();
  processFrequencyAndWave();
  updateOscillatorFrequencies();
  
  // Serial output at slower rate to avoid audio interference
  #if USE_SERIAL
  if (cmd_count > SERIAL_OUTPUT_DIVIDER) {  // Rate-limited serial output for better audio performance
    printSynthData();
    cmd_count = 0;
  }
  cmd_count++;
  #endif
}

#pragma endregion MAIN CONTROL

//////////////////////////////////////////////////
//  AUDIO GENERATION //
//////////////////////////////////////////////////
#pragma region AUDIO GENERATION
//////////////////////////////////////////////////
//  AUDIO FUNCTION IMPLEMENTATIONS //
//////////////////////////////////////////////////

/**
 * @brief Calculates and sets frequencies for all oscillator voices
 * 
 * Step-by-step process:
 * 1. Calculates octave offsets from inversion settings (0, 12, or 24 semitones)
 * 2. Computes frequency for each voice using semitone intervals
 * 3. Creates array of frequencies with voice indices for sorting
 * 4. Sorts frequencies to find two lowest (bass voices)
 * 5. Maps sorted frequencies to appropriate oscillators
 * 6. Applies smooth frequency transitions to prevent audio pops
 * 7. Sets oscillator frequencies based on selected waveform
 * 
 * Why: Ensures proper voice leading and prevents frequency conflicts between voices
 */
void updateOscillatorFrequencies() {
  // Calculate all voice frequencies using semitone intervals and inversion offsets
  float octave_offset1 = inv_aply1 * OCTAVE_SEMITONES; // 0, 12, or 24 semitones (0, 1, or 2 octaves)
  float octave_offset2 = inv_aply2 * OCTAVE_SEMITONES;
  float octave_offset3 = inv_aply3 * OCTAVE_SEMITONES; 
  float octave_offset4 = inv_aply4 * OCTAVE_SEMITONES;
  float octave_offset5 = inv_aply5 * OCTAVE_SEMITONES;
  
  // Calculate target frequency for each voice
  float tempFreq1 = freq1 * pow(2, (note1 + octave_offset1) / OCTAVE_SEMITONES); // ROOT
  float tempFreq2 = freq1 * pow(2, (note2 + octave_offset2) / OCTAVE_SEMITONES); // 2nd
  float tempFreq3 = freq1 * pow(2, (note3 + octave_offset3) / OCTAVE_SEMITONES); // 3rd
  float tempFreq4 = freq1 * pow(2, (note4 + octave_offset4) / OCTAVE_SEMITONES); // 4th
  float tempFreq5 = freq1 * pow(2, (note5 + octave_offset5) / OCTAVE_SEMITONES); // ROOT octave (or very high if disabled)
  
  // Create array of frequencies with their original voice indices
  struct FreqVoice {
    float freq;
    uint8_t voice;
  };
  
  FreqVoice freqArray[5] = {
    {tempFreq1, 1},
    {tempFreq2, 2}, 
    {tempFreq3, 3},
    {tempFreq4, 4},
    {tempFreq5, 5}
  };
  
  // Simple bubble sort to find two lowest frequencies (efficient for 5 elements)
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 4-i; j++) {
      if (freqArray[j].freq > freqArray[j+1].freq) {
        FreqVoice temp = freqArray[j];
        freqArray[j] = freqArray[j+1];
        freqArray[j+1] = temp;
      }
    }
  }
  
  // Now freqArray[0] and freqArray[1] contain the two lowest frequencies
  // Create mapping from original voice to new voice position
  uint8_t voiceMapping[5]; // voiceMapping[i] = which original voice goes to position i
  
  // Assign two lowest to voice1 and voice2
  voiceMapping[0] = freqArray[0].voice - 1; // Convert to 0-based index
  voiceMapping[1] = freqArray[1].voice - 1;
  
  // Assign remaining voices to voice3, voice4, voice5
  uint8_t remainingIndex = 2;
  for (int i = 2; i < 5; i++) {
    //if (freqArray[i].freq < 99999.0f) { // Skip disabled voice5
      voiceMapping[remainingIndex] = freqArray[i].voice - 1;
      remainingIndex++;
    //}
  }
  
  // Create array of original frequencies for smooth object mapping
  float originalFreqs[5] = {tempFreq1, tempFreq2, tempFreq3, tempFreq4, tempFreq5};
  
  // Create array of smooth objects for swapping
  Smooth<float>* smoothers[5] = {&kSmoothFreq1, &kSmoothFreq2, &kSmoothFreq3, &kSmoothFreq4, &kSmoothFreq5};
  
  // Apply frequencies using the correct smooth object for each frequency
  freqv1 = smoothers[voiceMapping[0]]->next(originalFreqs[voiceMapping[0]]);
  freqv2 = smoothers[voiceMapping[1]]->next(originalFreqs[voiceMapping[1]]);
  freqv3 = smoothers[voiceMapping[2]]->next(originalFreqs[voiceMapping[2]]);
  freqv4 = smoothers[voiceMapping[3]]->next(originalFreqs[voiceMapping[3]]);
  freqv5 = smoothers[voiceMapping[4]]->next(originalFreqs[voiceMapping[4]]);

  // Set oscillator frequencies based on selected waveform
  // Note: Wave assignments here must match those in updateAudio() for proper sound generation
  // Tip: Mix different waves (e.g., Sin1+Sin2 for bass, Saw3+Saw4 for treble) for custom voicings
  switch (wave1) {
    case 0://saw
      aSaw1.setFreq(freqv1); 
      aSaw2.setFreq(freqv2);
      aSaw3.setFreq(freqv3);
      aSaw4.setFreq(freqv4);
      aSaw5.setFreq(freqv5);
      break;

    case 1://squ
      aSqu1.setFreq(freqv1); 
      aSqu2.setFreq(freqv2);
      aSqu3.setFreq(freqv3);
      aSqu4.setFreq(freqv4);
      aSqu5.setFreq(freqv5);
      break;

    case 2://tri
      aTri1.setFreq(freqv1); 
      aTri2.setFreq(freqv2);
      aTri3.setFreq(freqv3);
      aTri4.setFreq(freqv4);
      aTri5.setFreq(freqv5);
      break;

    case 3://sin
      aSin1.setFreq(freqv1); 
      aSin2.setFreq(freqv2);
      aSin3.setFreq(freqv3);
      aSin4.setFreq(freqv4);
      aSin5.setFreq(freqv5);
      break;

    case 4://
      aChb1.setFreq(freqv1); 
      aChb2.setFreq(freqv2);
      aChb3.setFreq(freqv3);
      aChb4.setFreq(freqv4);
      aChb5.setFreq(freqv5);
      break;

    case 5://
      ahSin1.setFreq(freqv1); 
      ahSin2.setFreq(freqv2);
      ahSin3.setFreq(freqv3);
      ahSin4.setFreq(freqv4);
      ahSin5.setFreq(freqv5);
      break;

    case 6://
      aSig1.setFreq(freqv1); 
      aSig2.setFreq(freqv2);
      aSig3.setFreq(freqv3);
      aSig4.setFreq(freqv4);
      aSig5.setFreq(freqv5);
      break;

    case 7://
      aPha1.setFreq(freqv1);
      aPha2.setFreq(freqv2);
      aPha3.setFreq(freqv3);
      aPha4.setFreq(freqv4);
      aPha5.setFreq(freqv5);
      break;
  }
}


/**
 * @brief Generates audio output by mixing all oscillator voices
 * 
 * Step-by-step process:
 * 1. Checks gain level and sets bass/treble gains accordingly
 * 2. Ensures complete silence when gain is very low (prevents noise)
 * 3. Selects waveform and mixes appropriate oscillators
 * 4. Applies different gain levels to bass (lower) and treble (higher) frequencies
 * 5. Combines all voices into single audio signal
 * 6. Returns stereo output with clipping protection
 * 
 * Why: Creates rich harmonic content while maintaining proper frequency balance
 * @return Stereo audio output with automatic clipping protection
 */
AudioOutput_t updateAudio() {
  // Ensure complete silence when gain is very low
  if (gain < 5) {
    bassGain = 0;
    trebleGain = 0;
  } else {
    bassGain = (gain) >> BASS_GAIN_SHIFT;
    trebleGain = (gain) >> TREBLE_GAIN_SHIFT;
  }
 switch (wave1) { // Mix selected waveform voices into final audio signal
// Note: Exact gain balance between bass/treble voices is crucial for sound quality
// Tip: Experiment with different wave combinations for unique timbres
// Warning: Wave assignments must match those in updateOscillatorFrequencies()
   case 0://SAWS
    asig = (long)(((aSaw1.next() + aSaw2.next())*bassGain) + ((aSaw3.next() + aSaw4.next() + aSaw5.next())*trebleGain));
     break;

   case 1://SQUARE
       asig = (long)(((aSqu1.next() + aSqu2.next())*bassGain) + ((aSqu3.next() + aSqu4.next() + aSqu5.next())*trebleGain));
     break;

   case 2://TRIANGLE
       asig = (long)(((aTri1.next() + aTri2.next())*bassGain) + ((aTri3.next() + aTri4.next() + aTri5.next())*trebleGain));
     break;

   case 3://SINE
     asig = (long)(((aSin1.next() + aSin2.next())*bassGain) + ((aSin3.next() + aSin4.next() + aSin5.next())*trebleGain));
     break;

   case 4://CHEBYSHEV
       asig = (long)(((aChb1.next() + aChb2.next())*bassGain) + ((aChb3.next() + aChb4.next() + aChb5.next())*trebleGain));
     break;

   case 5://HALF SINE
      asig = (long)(((ahSin1.next() + ahSin2.next())*bassGain) + ((ahSin3.next() + ahSin4.next() + ahSin5.next())*trebleGain));
     break;

   case 6://SIG
      asig = (long)(((aSig1.next() + aSig2.next())*bassGain) + ((aSig3.next() + aSig4.next() + aSig5.next())*trebleGain));
     break;

   case 7://PHASOR
      asig = (long)(((aPha1.next() + aPha2.next())*bassGain) + ((aPha3.next() + aPha4.next() + aPha5.next())*trebleGain));
     break; 
 }
  return StereoOutput::fromAlmostNBit(AUDIO_OUTPUT_BITS, asig, asig).clip(); // Final stereo output with automatic clipping protection
  // AUDIO_OUTPUT_BITS can be adjusted (13-15) to balance dynamic range vs. processing load
  // Values below 13 may cause audio overflow and distortion
}

#pragma endregion AUDIO GENERATION

//////////////////////////////////////////////////
//  MAIN LOOP //
//////////////////////////////////////////////////
#pragma region MAIN LOOP

/**
 * @brief Main program loop - handles audio generation timing
 * 
 * Step-by-step process:
 * 1. Calls Mozzi's audioHook() function
 * 2. audioHook() automatically calls updateControl() at CONTROL_RATE (64Hz)
 * 3. audioHook() automatically calls updateAudio() at AUDIO_RATE (16384Hz)
 * 4. Maintains precise timing for smooth audio generation
 * 
 * Why: Mozzi handles all audio timing - adding other code here would disrupt audio
 * Warning: Do NOT add delays or other code here - it will cause audio glitches
 */
void loop() {
 audioHook(); // Mozzi's main audio engine - handles all timing automatically
}

#pragma endregion MAIN LOOP

//////////////////////////////////////////////////
//  END CODE //
//////////////////////////////////////////////////
