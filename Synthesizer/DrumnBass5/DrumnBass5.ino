/*  4-on-the-Floor Drum Machine
 *   Simple drum pattern with kick and hi-hat
 *   Uses Mozzi library for audio synthesis
 *   
 *   Kick drum: Low sine wave + envelope
 *   Hi-hat: Brown noise + envelope
 *   Bass: Saw wave + envelope with sequenced patterns
 *   Pattern: Kick on beats 1,2,3,4 - Hi-hat on off-beats - Bass sequences
 */

//////////////////////////////////////////////////
//  LIBRARIES AND CONFIGURATION //
//////////////////////////////////////////////////
#pragma region LIBRARIES AND CONFIG

// Configuration file - modify config.h to customize behavior
#include "config.h"

#include <MozziConfigValues.h>
#define MOZZI_AUDIO_CHANNELS MOZZI_STEREO
#include <MozziGuts.h>
#include <Oscil.h>
#include <Ead.h>                           // Exponential attack-decay envelope
#include <EventDelay.h>                    // For timing
#include <mozzi_midi.h>                    // For MIDI note conversion
#include <Smooth.h>
#include "wavetables.h"  // All wavetable includes and oscillator declarations
#include "drum_sequences.h"   // All drum and bass sequence patterns

#define CONTROL_RATE 256 // Needs 256 / high rate to improve the quality of the drum sounds

// Memory optimization - configuration moved to config.h
const bool useSerial = USE_SERIAL; // Set in config.h
const bool debugSequencer = DEBUG_SEQUENCER; // Set in config.h

//////////////////////////////////////////////////////////////////////////////////////////////////////////
// AUDIO ENGINE CONFIG
#pragma region Audio Engine

// Envelopes
Ead kickEnv(CONTROL_RATE);    // Kick drum envelope
Ead hihatEnv(CONTROL_RATE);   // Hi-hat envelope
Ead snareEnv(CONTROL_RATE);   // Snare drum envelope
Ead bassEnv(CONTROL_RATE);    // Bass envelope

// Timing - removed unused triggers to save memory
EventDelay stepDelay;         // Main step sequencer timing (16th notes)

// Pattern variables - optimized for memory
uint8_t currentStep = 0;          // Current step in the pattern (0-15 for 16-step patterns)
const uint8_t patternLength = PATTERN_LENGTH;  // Steps per pattern (from config.h)

// MARKOV CHAIN BASS GENERATION SYSTEM
// Replaces fixed sequences with probabilistic note generation
// Uses tempo-aware transition matrices for musical coherence

// Markov chain state space: scale indices 0-8 (9 states)
#define MARKOV_STATES 9
#define MARKOV_PROB_SCALE 255  // Use 8-bit probabilities (0-255)

// Transition matrices for different tempo ranges (chord-tone focused)
// Each row sums to ~255 for proper probability distribution
// Designed for harmonic flow: root-third-fifth relationships, chord tone emphasis

// SLOW TEMPO - Chord-tone focused with thirds and fifths emphasis
const uint8_t markovMatrixSlow[MARKOV_STATES][MARKOV_STATES] PROGMEM = {
  // From state 0 (ROOT): Strong self-loop, favor thirds (1,2) and fifths (3,4)
  {120, 90, 80, 70, 60, 20, 10, 5, 3},   // 0->0,1,2,3,4 (root to chord tones)
  // From state 1 (THIRD): Return to root, move to fifth, stay on third
  {90, 100, 60, 80, 70, 25, 15, 8, 5},   // 1->0,1,3,4 (third relationships)
  // From state 2 (THIRD): Similar to state 1, chord tone emphasis
  {80, 70, 95, 75, 65, 30, 20, 10, 8},   // 2->0,1,2,3,4 (third relationships)
  // From state 3 (FIFTH): Strong chord tone, return to root/third
  {70, 80, 75, 110, 50, 35, 25, 15, 10}, // 3->0,1,2,3 (fifth relationships)
  // From state 4 (FIFTH): Similar to state 3, chord tone emphasis  
  {60, 70, 65, 60, 105, 40, 30, 20, 15}, // 4->0,1,2,3,4 (fifth relationships)
  // From state 5: Moderate return to chord tones
  {30, 40, 50, 55, 60, 80, 45, 25, 20},  // 5->chord tones
  // From state 6: Return tendency to chord tones
  {20, 30, 40, 45, 50, 60, 85, 40, 30},  // 6->chord tones
  // From state 7: Strong return to chord tones
  {15, 25, 35, 40, 45, 50, 65, 90, 50},  // 7->chord tones
  // From state 8: Very strong return to chord tones
  {10, 20, 30, 35, 40, 45, 55, 70, 85}   // 8->chord tones
};

// MEDIUM TEMPO - Chord-tone focused with more activity
const uint8_t markovMatrixMed[MARKOV_STATES][MARKOV_STATES] PROGMEM = {
  // From state 0 (ROOT): Strong chord tone emphasis, more active than slow
  {110, 95, 85, 75, 65, 25, 15, 8, 5},   // 0->0,1,2,3,4 (root to chord tones)
  // From state 1 (THIRD): Active chord tone movement
  {95, 90, 70, 85, 75, 30, 20, 12, 8},   // 1->0,1,3,4 (third relationships)
  // From state 2 (THIRD): Similar to state 1, more active
  {85, 75, 85, 80, 70, 35, 25, 15, 10},  // 2->0,1,2,3,4 (third relationships)
  // From state 3 (FIFTH): Strong chord tone, active movement
  {75, 85, 80, 100, 60, 40, 30, 20, 15}, // 3->0,1,2,3 (fifth relationships)
  // From state 4 (FIFTH): Active fifth relationships
  {65, 75, 70, 70, 95, 45, 35, 25, 20},  // 4->0,1,2,3,4 (fifth relationships)
  // From state 5: Active return to chord tones
  {35, 45, 55, 60, 65, 75, 50, 30, 25},  // 5->chord tones
  // From state 6: Return tendency to chord tones
  {25, 35, 45, 50, 55, 65, 80, 45, 35},  // 6->chord tones
  // From state 7: Strong return to chord tones
  {20, 30, 40, 45, 50, 55, 70, 85, 60},  // 7->chord tones
  // From state 8: Very strong return to chord tones
  {15, 25, 35, 40, 45, 50, 60, 75, 80}   // 8->chord tones
};

// FAST TEMPO - Very active chord-tone movement
const uint8_t markovMatrixFast[MARKOV_STATES][MARKOV_STATES] PROGMEM = {
  // From state 0 (ROOT): Very active chord tone emphasis
  {100, 100, 90, 80, 70, 30, 20, 12, 8}, // 0->0,1,2,3,4 (root to chord tones)
  // From state 1 (THIRD): Very active third relationships
  {100, 85, 75, 90, 80, 35, 25, 15, 10}, // 1->0,1,3,4 (third relationships)
  // From state 2 (THIRD): Very active, similar to state 1
  {90, 80, 80, 85, 75, 40, 30, 20, 15},  // 2->0,1,2,3,4 (third relationships)
  // From state 3 (FIFTH): Very active fifth relationships
  {80, 90, 85, 90, 70, 45, 35, 25, 20},  // 3->0,1,2,3 (fifth relationships)
  // From state 4 (FIFTH): Very active fifth movement
  {70, 80, 75, 80, 85, 50, 40, 30, 25},  // 4->0,1,2,3,4 (fifth relationships)
  // From state 5: Very active return to chord tones
  {40, 50, 60, 65, 70, 70, 55, 35, 30},  // 5->chord tones
  // From state 6: Active return to chord tones
  {30, 40, 50, 55, 60, 70, 75, 50, 40},  // 6->chord tones
  // From state 7: Strong return to chord tones
  {25, 35, 45, 50, 55, 60, 75, 80, 65},  // 7->chord tones
  // From state 8: Very strong return to chord tones
  {20, 30, 40, 45, 50, 55, 65, 80, 75}   // 8->chord tones
};

// Markov chain state tracking
uint8_t markovCurrentState = 0;  // Current state (scale index)
uint32_t markovRandomSeed = 0xACE1FACE; // Xorshift32 seed (must be non-zero)

// Pattern repetition system for enhanced musicality
uint8_t markovPatternCount = 0;   // Current pattern repetition count (0-1)
uint8_t markovBarCount = 0;       // Current bar within pattern (0-15 for 16-step bars)
bool markovShouldEvolve = false;  // Flag to trigger pattern evolution

// Bass timing Markov system - probability of playing on each step type
// Values represent probability (0-255) of bass triggering on different step positions
struct TimingProbabilities {
  uint8_t onBeat;      // Probability on strong beats (0, 4, 8, 12)
  uint8_t offBeat;     // Probability on weak beats (2, 6, 10, 14)  
  uint8_t upBeat;      // Probability on upbeats (1, 3, 5, 7, 9, 11, 13, 15)
  uint8_t syncopated;  // Probability on syncopated positions (varies by tempo)
};

// Timing probabilities for different tempo ranges and complexity levels
// [tempo_range][complexity_level]
const TimingProbabilities timingProbs[3][5] PROGMEM = {
  // SLOW TEMPO - Hip hop, trap, ambient
  {
    {200, 80, 40, 20},   // Complexity 0: Very predictable (mostly on-beats)
    {180, 100, 60, 40},  // Complexity 1: Slightly more varied
    {160, 120, 80, 60},  // Complexity 2: Balanced
    {140, 140, 100, 80}, // Complexity 3: More syncopated
    {120, 160, 120, 100} // Complexity 4: Very syncopated
  },
  // MEDIUM TEMPO - House, techno, funk
  {
    {180, 120, 80, 40},  // Complexity 0: House-like (strong on-beats)
    {160, 140, 100, 60}, // Complexity 1: More driving
    {140, 160, 120, 80}, // Complexity 2: Funky
    {120, 180, 140, 100},// Complexity 3: Very funky
    {100, 200, 160, 120} // Complexity 4: Chaotic funk
  },
  // FAST TEMPO - DnB, breakbeat, jungle
  {
    {160, 100, 120, 60}, // Complexity 0: Simple DnB
    {140, 120, 140, 80}, // Complexity 1: More active
    {120, 140, 160, 100},// Complexity 2: Breakbeat style
    {100, 160, 180, 120},// Complexity 3: Complex breaks
    {80, 180, 200, 140}  // Complexity 4: Jungle madness
  }
};

// Simplified scale selection for memory efficiency
enum BassScale { 
  MINOR_BLUES,      // Classic bluesy bass
  DORIAN_MODE,      // Dark, jazzy bass  
  PHRYGIAN_MODE,    // Dark, ethnic bass
  MAJOR_PENTATONIC  // Bright, uplifting
};

// Simplified scale assignments - one scale per tempo range
BassScale bassScaleSlow = PHRYGIAN_MODE;    // Dark, ethereal for slow tempos
BassScale bassScaleMed = DORIAN_MODE;       // Dark house for medium tempos  
BassScale bassScaleFast = PHRYGIAN_MODE;    // Dark DnB for fast tempos

// Markov chain pattern complexity control (0-4, affects transition randomness)
// Higher values = more chaotic/experimental, lower = more predictable
uint8_t markovComplexity = 2;  // Default medium complexity

// Current pattern selections - optimized for memory (0-4 indexing)
uint8_t currentKickPattern = 0;   // Start with first pattern
uint8_t currentHihatPattern = 0;  // Start with first pattern  
uint8_t currentSnarePattern = 0;  // Start with first pattern
const uint8_t currentBassPattern = 2; // Fixed bass pattern (user can change this constant, 0-4)
uint8_t bassNoteIndex = 0;        // Current position in bass note sequence
uint16_t bassNoteLength = 80;     // Bass note length in ms (controlled by botPot)
uint8_t currentTempoRange = 1;    // 0=slow, 1=medium, 2=fast

// Simplified bass - removed phasing/beating to save memory

// BASS-CENTRIC MIDI tuning system - Bass is the foundation, drums follow
uint8_t bassRootMidiNote = BASS_ROOT_MIDI_NOTE;           // A#1 - bass root MIDI note (foundation)
uint8_t kickMidiNote = BASS_ROOT_MIDI_NOTE + KICK_INTERVAL_OFFSET;     // F1 - kick follows bass root
uint8_t snareToneMidiNote = BASS_ROOT_MIDI_NOTE + SNARE_TONE_INTERVAL_OFFSET; // F2 - snare tone follows bass root

// Optimized scales using semitone offsets (more musical than frequency ratios)
const int8_t minorBluesIntervals[] PROGMEM = {0, 3, 5, 6, 7, 10};        // Minor blues (6 notes)
const int8_t majorPentatonicIntervals[] PROGMEM = {0, 2, 4, 7, 9};       // Major pentatonic (5 notes)
const int8_t dorianIntervals[] PROGMEM = {0, 2, 3, 5, 7, 9, 10};         // Dorian mode (7 notes)
const int8_t phrygianIntervals[] PROGMEM = {0, 1, 3, 5, 7, 8, 10};       // Phrygian mode (7 notes)

// Scale sizes
const uint8_t scaleSizes[] PROGMEM = {6, 5, 7, 7};

// Current bass note (MIDI)
uint8_t currentBassMidiNote = 0;
#pragma endregion Audio Engine

//////////////////////////////////////////////////////////////////////////////////////////////////////////
// INPUT VARIABLES
#pragma region Input Variables

// Control variables for touchpad and pots - optimized for memory
uint16_t xVal, yVal;
uint16_t botPot, midPot, hiPot;
uint8_t prox; // Proximity sensor reading
bool axis = false; // For touchpad coordinate alternating

// Mapped control variables (processed from raw inputs) - optimized for memory
uint16_t bassNoteLengthSet = 80;   // Bass note length (20-600ms)
uint8_t tempoSet = 180;            // Tempo (30-200 BPM)
int8_t swingSet = 0;               // Swing (-100 to +100)
uint8_t kick_pattern_set = 0;      // Bass patterns (0-7)
uint8_t drumSet = 5;               // Drum patterns (0-10)
#pragma endregion Input Variables

//////////////////////////////////////////////////////////////////////////////////////////////////////////
// CONTROL SYSTEM CONFIG
#pragma region Control System

// Simplified control system - direct touchpad mapping

// Control parameters - optimized for memory
uint8_t tempo = DEFAULT_TEMPO;              // BPM - now controlled by midPot
uint8_t baseTempo = DEFAULT_TEMPO;          // Base tempo
uint16_t effectiveTempo = DEFAULT_TEMPO;    // Actual tempo used for timing (may be doubled/quadrupled)
uint16_t targetTempo = DEFAULT_TEMPO;       // Target tempo for gradual transitions
uint16_t startTransitionTempo = DEFAULT_TEMPO; // Starting tempo for the current transition
uint8_t tempoMultiplier = 1;      // Multiplier for beat repeat effect (1, 2, or 4)
bool tempoTransitioning = false;  // Flag to indicate tempo is transitioning
uint8_t tempoTransitionSteps = 0; // Counter for transition steps
const uint8_t tempoTransitionLength = TEMPO_TRANSITION_LENGTH; // Number of steps to transition over

// Swing control - optimized for memory
int8_t swingAmount = 0;           // Swing amount (-100 to +100, 0 = no swing)
uint16_t baseStepTime = 0;        // Base step timing without swing
bool isOffBeat = false;           // Track if current step is an off-beat for swing
uint8_t kickNote = 24;            // C2 - typical kick drum note
float kickPitch = KICK_FREQUENCY;           // Base frequency for kick (from config.h)
uint8_t kickAttack = KICK_ATTACK_TIME;          // Attack time in ms (from config.h)
uint8_t kickDecay = KICK_DECAY_TIME;          // Decay time in ms (from config.h)
uint8_t hihatAttack = HIHAT_ATTACK_TIME;          // Attack time in ms (from config.h)
uint8_t hihatDecay = HIHAT_DECAY_TIME;          // Decay time in ms (from config.h)
uint8_t snareAttack = SNARE_ATTACK_TIME;          // Snare attack time in ms (from config.h)
uint8_t snareDecay = SNARE_DECAY_TIME;          // Snare decay time in ms (from config.h)
uint8_t bassAttack = BASS_ATTACK_TIME;          // Bass attack time in ms (from config.h)
uint8_t bassDecay = BASS_DECAY_TIME;          // Bass decay time in ms (from config.h)

// Pattern length control - optimized for memory
uint8_t currentPatternLength = PATTERN_LENGTH;  // Current active pattern length (from config.h)
const uint8_t maxPatternLength = MAX_PATTERN_LENGTH; // Maximum pattern length (from config.h)

// Audio mixing - optimized for memory
int16_t kickGain = 0;
int16_t hihatGain = 0;
int16_t snareGain = 0;
int16_t bassGain = 0;
float bassFreq = 55.0;       // Current bass frequency
#pragma endregion Control System
#pragma endregion LIBRARIES AND CONFIG

// Get MIDI note from scale index using semitone offsets
uint8_t getScaleMidiNote(BassScale scaleType, uint8_t noteIndex) {
  const int8_t* intervals;
  uint8_t scaleSize;
  
  // Select intervals and size based on scale type
  switch (scaleType) {
    case MINOR_BLUES:
      intervals = minorBluesIntervals;
      scaleSize = pgm_read_byte(&scaleSizes[0]);
      break;
    case MAJOR_PENTATONIC:
      intervals = majorPentatonicIntervals;
      scaleSize = pgm_read_byte(&scaleSizes[1]);
      break;
    case DORIAN_MODE:
      intervals = dorianIntervals;
      scaleSize = pgm_read_byte(&scaleSizes[2]);
      break;
    case PHRYGIAN_MODE:
      intervals = phrygianIntervals;
      scaleSize = pgm_read_byte(&scaleSizes[3]);
      break;
    default:
      intervals = minorBluesIntervals;
      scaleSize = pgm_read_byte(&scaleSizes[0]);
      break;
  }
  
  // Wrap note index within scale, allowing octave extensions
  uint8_t octave = noteIndex / scaleSize;
  uint8_t scaleNote = noteIndex % scaleSize;
  
  // Return MIDI note: bass root + scale interval + octave offset
  uint8_t midiNote = bassRootMidiNote + pgm_read_byte(&intervals[scaleNote]) + (octave * 12);
  
  // Keep within reasonable MIDI range (24-84 for bass)
  return constrain(midiNote, 24, 84);
}

// Convert MIDI note to frequency using Mozzi's mtof() function
float midiNoteToFrequency(uint8_t midiNote) {
  return mtof(midiNote);
}

// Xorshift32 PRNG - High quality, fast, memory efficient
// Much better statistical properties than simple LCG
uint32_t markovRandom() {
  // Xorshift32 algorithm - excellent randomness with minimal memory
  markovRandomSeed ^= markovRandomSeed << 13;
  markovRandomSeed ^= markovRandomSeed >> 17;
  markovRandomSeed ^= markovRandomSeed << 5;
  
  // Ensure seed never becomes zero (would break Xorshift)
  if (markovRandomSeed == 0) {
    markovRandomSeed = 0xDEADBEEF; // Non-zero fallback
  }
  
  return markovRandomSeed & 0x7FFF; // Return 15-bit value for compatibility
}

// Determine step type for timing probability calculation
uint8_t getStepType(uint8_t step) {
  // Classify step position for timing probability
  if (step % 4 == 0) return 0;      // On-beat (0, 4, 8, 12)
  if (step % 4 == 2) return 1;      // Off-beat (2, 6, 10, 14)
  if (step % 2 == 1) return 2;      // Up-beat (1, 3, 5, 7, 9, 11, 13, 15)
  return 3;                         // Syncopated (other positions)
}

// Check if bass should trigger using Markov timing probabilities
bool getMarkovBassTiming(uint8_t step) {
  // Get timing probabilities for current tempo and complexity
  TimingProbabilities probs;
  memcpy_P(&probs, &timingProbs[currentTempoRange][markovComplexity], sizeof(TimingProbabilities));
  
  // Determine step type and get corresponding probability
  uint8_t stepType = getStepType(step);
  uint8_t probability;
  
  switch(stepType) {
    case 0: probability = probs.onBeat; break;
    case 1: probability = probs.offBeat; break;
    case 2: probability = probs.upBeat; break;
    case 3: probability = probs.syncopated; break;
    default: probability = probs.onBeat; break;
  }
  
  // Generate random decision
  uint16_t randomVal = markovRandom() & 0xFF; // 8-bit random value
  return randomVal < probability;
}

// Get next Markov state based on current state and tempo range
uint8_t getNextMarkovState() {
  // Select appropriate transition matrix based on tempo range
  const uint8_t* matrix;
  switch(currentTempoRange) {
    case 0: matrix = (const uint8_t*)markovMatrixSlow[markovCurrentState]; break;
    case 1: matrix = (const uint8_t*)markovMatrixMed[markovCurrentState]; break;
    case 2: matrix = (const uint8_t*)markovMatrixFast[markovCurrentState]; break;
    default: matrix = (const uint8_t*)markovMatrixMed[markovCurrentState]; break;
  }
  
  // Apply complexity scaling to probabilities
  // Higher complexity = more uniform distribution (more chaos)
  // Lower complexity = more peaked distribution (more predictable)
  uint16_t scaledProbs[MARKOV_STATES];
  uint16_t totalProb = 0;
  
  for (uint8_t i = 0; i < MARKOV_STATES; i++) {
    uint8_t baseProb = pgm_read_byte(&matrix[i]);
    
    // Complexity scaling: 0=very peaked, 4=very uniform
    if (markovComplexity == 0) {
      // Very predictable - amplify highest probabilities
      scaledProbs[i] = (baseProb > 30) ? baseProb * 2 : baseProb / 2;
    } else if (markovComplexity == 4) {
      // Very chaotic - flatten distribution
      scaledProbs[i] = (baseProb + 128) / 2;
    } else {
      // Linear interpolation between base and flattened
      uint16_t flatProb = (baseProb + 128) / 2;
      scaledProbs[i] = baseProb + ((flatProb - baseProb) * markovComplexity) / 4;
    }
    
    totalProb += scaledProbs[i];
  }
  
  // Generate random number and find corresponding state
  uint16_t randomVal = markovRandom() % totalProb;
  uint16_t cumulative = 0;
  
  for (uint8_t i = 0; i < MARKOV_STATES; i++) {
    cumulative += scaledProbs[i];
    if (randomVal < cumulative) {
      return i;
    }
  }
  
  // Fallback (should never reach here)
  return 0;
}

// Function to read kick pattern from PROGMEM (tempo-aware)
bool getKickStep(uint8_t step) {
  switch(currentTempoRange) {
    case 0: // Slow tempo
      switch(currentKickPattern) {
        case 0: return pgm_read_byte(&kickPatternSlow1[step]);
        case 1: return pgm_read_byte(&kickPatternSlow2[step]);
        case 2: return pgm_read_byte(&kickPatternSlow3[step]);
        case 3: return pgm_read_byte(&kickPatternSlow4[step]);
        case 4: return pgm_read_byte(&kickPatternSlow5[step]);
        default: return pgm_read_byte(&kickPatternSlow1[step]);
      }
    case 1: // Medium tempo
      switch(currentKickPattern) {
        case 0: return pgm_read_byte(&kickPatternMed1[step]);
        case 1: return pgm_read_byte(&kickPatternMed2[step]);
        case 2: return pgm_read_byte(&kickPatternMed3[step]);
        case 3: return pgm_read_byte(&kickPatternMed4[step]);
        case 4: return pgm_read_byte(&kickPatternMed5[step]);
        default: return pgm_read_byte(&kickPatternMed1[step]);
      }
    case 2: // Fast tempo
      switch(currentKickPattern) {
        case 0: return pgm_read_byte(&kickPatternFast1[step]);
        case 1: return pgm_read_byte(&kickPatternFast2[step]);
        case 2: return pgm_read_byte(&kickPatternFast3[step]);
        case 3: return pgm_read_byte(&kickPatternFast4[step]);
        case 4: return pgm_read_byte(&kickPatternFast5[step]);
        default: return pgm_read_byte(&kickPatternFast1[step]);
      }
    default: return pgm_read_byte(&kickPatternMed1[step]);
  }
}

// Function to read hi-hat pattern from PROGMEM (tempo-aware)
bool getHihatStep(uint8_t step) {
  switch(currentTempoRange) {
    case 0: // Slow tempo
      switch(currentHihatPattern) {
        case 0: return pgm_read_byte(&hihatPatternSlow1[step]);
        case 1: return pgm_read_byte(&hihatPatternSlow2[step]);
        case 2: return pgm_read_byte(&hihatPatternSlow3[step]);
        case 3: return pgm_read_byte(&hihatPatternSlow4[step]);
        case 4: return pgm_read_byte(&hihatPatternSlow5[step]);
        default: return pgm_read_byte(&hihatPatternSlow1[step]);
      }
    case 1: // Medium tempo
      switch(currentHihatPattern) {
        case 0: return pgm_read_byte(&hihatPatternMed1[step]);
        case 1: return pgm_read_byte(&hihatPatternMed2[step]);
        case 2: return pgm_read_byte(&hihatPatternMed3[step]);
        case 3: return pgm_read_byte(&hihatPatternMed4[step]);
        case 4: return pgm_read_byte(&hihatPatternMed5[step]);
        default: return pgm_read_byte(&hihatPatternMed1[step]);
      }
    case 2: // Fast tempo
      switch(currentHihatPattern) {
        case 0: return pgm_read_byte(&hihatPatternFast1[step]);
        case 1: return pgm_read_byte(&hihatPatternFast2[step]);
        case 2: return pgm_read_byte(&hihatPatternFast3[step]);
        case 3: return pgm_read_byte(&hihatPatternFast4[step]);
        case 4: return pgm_read_byte(&hihatPatternFast5[step]);
        default: return pgm_read_byte(&hihatPatternFast1[step]);
      }
    default: return pgm_read_byte(&hihatPatternMed1[step]);
  }
}

// Function to read snare pattern from PROGMEM (tempo-aware)
bool getSnareStep(uint8_t step) {
  switch(currentTempoRange) {
    case 0: // Slow tempo
      switch(currentSnarePattern) {
        case 0: return pgm_read_byte(&snarePatternSlow1[step]);
        case 1: return pgm_read_byte(&snarePatternSlow2[step]);
        case 2: return pgm_read_byte(&snarePatternSlow3[step]);
        case 3: return pgm_read_byte(&snarePatternSlow4[step]);
        case 4: return pgm_read_byte(&snarePatternSlow5[step]);
        default: return pgm_read_byte(&snarePatternSlow1[step]);
      }
    case 1: // Medium tempo
      switch(currentSnarePattern) {
        case 0: return pgm_read_byte(&snarePatternMed1[step]);
        case 1: return pgm_read_byte(&snarePatternMed2[step]);
        case 2: return pgm_read_byte(&snarePatternMed3[step]);
        case 3: return pgm_read_byte(&snarePatternMed4[step]);
        case 4: return pgm_read_byte(&snarePatternMed5[step]);
        default: return pgm_read_byte(&snarePatternMed1[step]);
      }
    case 2: // Fast tempo
      switch(currentSnarePattern) {
        case 0: return pgm_read_byte(&snarePatternFast1[step]);
        case 1: return pgm_read_byte(&snarePatternFast2[step]);
        case 2: return pgm_read_byte(&snarePatternFast3[step]);
        case 3: return pgm_read_byte(&snarePatternFast4[step]);
        case 4: return pgm_read_byte(&snarePatternFast5[step]);
        default: return pgm_read_byte(&snarePatternFast1[step]);
      }
    default: return pgm_read_byte(&snarePatternMed1[step]);
  }
}

// Function to check if bass should trigger on this step (tempo-aware)
bool getBassTiming(uint8_t step) {
  switch(currentTempoRange) {
    case 0: // Slow tempo
      switch(currentBassPattern) {
        case 1: return pgm_read_byte(&bassTimingSlow1[step]);
        case 2: return pgm_read_byte(&bassTimingSlow2[step]);
        case 3: return pgm_read_byte(&bassTimingSlow3[step]);
        case 4: return pgm_read_byte(&bassTimingSlow4[step]);
        case 5: return pgm_read_byte(&bassTimingSlow5[step]);
        default: return pgm_read_byte(&bassTimingSlow1[step]);
      }
    case 1: // Medium tempo
      switch(currentBassPattern) {
        case 1: return pgm_read_byte(&bassTimingMed1[step]);
        case 2: return pgm_read_byte(&bassTimingMed2[step]);
        case 3: return pgm_read_byte(&bassTimingMed3[step]);
        case 4: return pgm_read_byte(&bassTimingMed4[step]);
        case 5: return pgm_read_byte(&bassTimingMed5[step]);
        default: return pgm_read_byte(&bassTimingMed1[step]);
      }
    case 2: // Fast tempo
      switch(currentBassPattern) {
        case 1: return pgm_read_byte(&bassTimingFast1[step]);
        case 2: return pgm_read_byte(&bassTimingFast2[step]);
        case 3: return pgm_read_byte(&bassTimingFast3[step]);
        case 4: return pgm_read_byte(&bassTimingFast4[step]);
        case 5: return pgm_read_byte(&bassTimingFast5[step]);
        default: return pgm_read_byte(&bassTimingFast1[step]);
      }
    default: return pgm_read_byte(&bassTimingMed1[step]);
  }
}

// Handle pattern repetition and evolution for enhanced musicality
void updateMarkovPatternLogic() {
  // Increment bar count
  markovBarCount++;
  
  // Check if we've completed a 16-step bar
  if (markovBarCount >= 16) {
    markovBarCount = 0;
    markovPatternCount++;
    
    // Check if we've played the pattern twice
    if (markovPatternCount >= 2) {
      markovPatternCount = 0;
      markovShouldEvolve = true;
      
      if (useSerial) {
        Serial.println(F("Markov: Pattern evolution triggered"));
      }
    }
  }
}

// Function to get the next bass MIDI note using Markov chain generation
uint8_t getBassNote() {
  // Check if we should evolve the pattern (only evolve on note generation, not every step)
  if (markovShouldEvolve) {
    // Generate next state using Markov chain (evolution)
    markovCurrentState = getNextMarkovState();
    markovShouldEvolve = false;
    
    if (useSerial) {
      Serial.print(F("Markov: Evolved to state "));
      Serial.println(markovCurrentState);
    }
  }
  // If not evolving, keep current state for pattern repetition
  
  // Get the appropriate scale for this tempo range
  BassScale currentScale;
  switch(currentTempoRange) {
    case 0: currentScale = bassScaleSlow; break;  // Slow tempo
    case 1: currentScale = bassScaleMed; break;   // Medium tempo
    case 2: currentScale = bassScaleFast; break;  // Fast tempo
    default: currentScale = PHRYGIAN_MODE; break;
  }
  
  // Convert Markov state (scale index) to MIDI note and return
  return getScaleMidiNote(currentScale, markovCurrentState);
}

//////////////////////////////////////////////////
//  S E T U P //
//////////////////////////////////////////////////
#pragma region SETUP

/**
 * @brief Arduino setup function
 * 
 * Initializes Mozzi audio library, sets up oscillators and envelopes,
 * configures initial timing parameters, and prints startup information.
 */
void setup() {  
  startMozzi(CONTROL_RATE);
  
  // Set up BASS-CENTRIC tuning system - Bass is foundation, drums follow
  kickPitch = midiNoteToFrequency(kickMidiNote);  // Convert kick MIDI note to frequency
  float snareToneFreq = midiNoteToFrequency(snareToneMidiNote); // Convert snare tone MIDI to frequency
  
  // Set up kick drum
  kickOsc.setFreq(kickPitch);
  kickEnv.set(kickAttack, kickDecay);
  
  // Set up hi-hat
  hihatOsc.setFreq((float)AUDIO_RATE/WHITENOISE8192_SAMPLERATE);
  hihatEnv.set(hihatAttack, hihatDecay);
  
  // Set up snare drum (noise + tonal component)
  snareOsc.setFreq((float)AUDIO_RATE/WHITENOISE8192_SAMPLERATE);
  snareTone.setFreq(snareToneFreq); // Tonal component derived from bass root
  snareEnv.set(snareAttack, snareDecay);
  
  // Set up bass (single oscillator for clean sound)
  bassOsc.setFreq(bassFreq);
  bassEnv.set(bassAttack, bassDecay);
  
  // Calculate initial step timing - 16th notes
  effectiveTempo = baseTempo * tempoMultiplier;  // Initialize effective tempo
  int stepTime = 60000 / (effectiveTempo * 4);
  baseStepTime = stepTime;  // Initialize base step time for swing calculations
  stepDelay.set(stepTime);
  stepDelay.start();
  

  
  if (useSerial) {
    Serial.begin(SERIAL_BAUD_RATE);
    Serial.println(F("MARKOV CHAIN Drum Machine + SNARE Ready!"));
    Serial.println(F("X=Kick+Bass Complexity, Y=Hi-hat, Diagonal=Snare"));
    Serial.println(F("botPot=Bass Length, midPot=Tempo, hiPot=Swing"));
    Serial.println(F("Tempo: Slow(30-79), Med(80-119), Fast(120-160)"));
    Serial.println(F("BASS-CENTRIC TUNING: Bass is foundation, drums follow"));
    Serial.print(F("Bass Root MIDI: "));
    Serial.print(bassRootMidiNote);
    Serial.print(F(" ("));
    Serial.print(midiNoteToFrequency(bassRootMidiNote));
    Serial.print(F("Hz)"));
    Serial.println();
    Serial.print(F("Kick MIDI: "));
    Serial.print(kickMidiNote);
    Serial.print(F(", Snare Tone MIDI: "));
    Serial.println(snareToneMidiNote);
    Serial.println(F("ENHANCED MARKOV BASS: Notes + Timing, Pattern Repetition!"));
    Serial.println(F("- Probabilistic note selection AND timing"));
    Serial.println(F("- Patterns repeat twice before evolving"));
    Serial.println(F("- Tempo-aware timing probabilities"));
    Serial.println(F("- Xorshift32 PRNG for superior randomness"));
    Serial.print(F("Initial Complexity: "));
    Serial.println(markovComplexity);
  }
}
#pragma endregion SETUP

//////////////////////////////////////////////////
//  FUNCTIONS  //
//////////////////////////////////////////////////
#pragma region FUNCTIONS

// Subfunctions to keep updateControl() organized and readable
#pragma region Control Helpers

/**
 * @brief Reads all input sources and maps them to control variables
 * 
 * Handles touchpad coordinate reading with proper alternating X/Y sampling,
 * reads all potentiometers and proximity sensor, then maps raw values to
 * semantic control variables for easy remapping.
 */
void readInputs() {
  // Toggle between checking x or y coordinate for touchpad - can only do one at a time to prevent interference
  if (axis){
    yVal = ycoor();
    yVal = constrain(yVal, TOUCHPAD_Y_MIN, TOUCHPAD_Y_MAX);//constrains to the range you can actually touch
    axis = !axis;
  }else{
    xVal = xcoor();
    xVal = constrain(xVal, TOUCHPAD_X_MIN, TOUCHPAD_X_MAX);//constrains to the range you can actually touch
    axis = !axis;
  }
  
    //use the Mozzi version of analogRead (it's faster) to get our potentiometer values
    botPot = mozziAnalogRead<10>(POT_PIN1);  // bass note length
    midPot = mozziAnalogRead<10>(POT_PIN2);  // tempo
    hiPot = mozziAnalogRead<10>(POT_PIN3);  // swing

  prox = digitalRead(PROX_PIN);// check prox sensor for control mode switching

  // EASY REMAPPING SECTION - Change these mappings to reassign controls
  // For example, you could set tempoSet = botPot>>7 to use bottom knob for tempo
  // and then set bassNoteLengthSet = midPot to use middle knob for bass note length
  bassNoteLengthSet = map(botPot, 0, 1023, BASS_NOTE_LENGTH_MIN, BASS_NOTE_LENGTH_MAX);    // Bass note length (from config.h)
  tempoSet = map(midPot, 0, 1023, TEMPO_MIN, TEMPO_MAX);             // Tempo (from config.h)
  swingSet = map(hiPot, 0, 1023, SWING_MIN, SWING_MAX);            // Swing (from config.h)
  // Only update patterns if finger is actually on the touchpad
  // When finger is off: xVal ~390, yVal ~990-1000
  bool fingerOnPad = (yVal < TOUCHPAD_EDGE_DETECT); // yVal goes high when finger is off
  
  if (fingerOnPad) {
    kick_pattern_set = map(xVal, TOUCHPAD_X_MIN, TOUCHPAD_X_MAX, 0, 4);               // Kick patterns (0-4) 
    drumSet = map(yVal, TOUCHPAD_Y_MIN, TOUCHPAD_Y_MAX, 0, 4);                        // Drum patterns (0-4)
  }
  // If finger is off pad, keep current pattern values unchanged
    
  // Apply hysteresis to prevent jittery pattern switching
  applyInputHysteresis();
}

/**
 * @brief Applies hysteresis to mapped input values to prevent jitter
 * 
 * Prevents rapid switching between adjacent values when inputs hover
 * around boundaries. Only changes values when they move significantly.
 */
void applyInputHysteresis() {
  // Pattern hysteresis - only change if moved by more than threshold
  static uint8_t lastKickPattern = 255; // Initialize to invalid value
  static uint8_t lastDrumSet = 255;
  
  // Kick pattern hysteresis (0-4 range)
  if (lastKickPattern == 255) {
    lastKickPattern = kick_pattern_set; // First time initialization
  } else {
    // Only update if change is significant or at boundaries
    int diff = abs((int)kick_pattern_set - (int)lastKickPattern);
    if (diff >= 1 || kick_pattern_set == 0 || kick_pattern_set == 4) {
      lastKickPattern = kick_pattern_set;
    } else {
      kick_pattern_set = lastKickPattern; // Keep previous value
    }
  }
  
  // Drum set hysteresis (0-4 range)  
  if (lastDrumSet == 255) {
    lastDrumSet = drumSet; // First time initialization
  } else {
    // Only update if change is significant or at boundaries
    int diff = abs((int)drumSet - (int)lastDrumSet);
    if (diff >= 1 || drumSet == 0 || drumSet == 4) {
      lastDrumSet = drumSet;
    } else {
      drumSet = lastDrumSet; // Keep previous value
    }
  }
}

/**
 * @brief Processes tempo control and timing parameters
 * 
 * Handles tempo changes with smooth transitions, determines tempo range
 * for pattern selection, and manages bass note length and swing timing.
 */
void processTempoAndTiming() {
  // Use mapped tempo value
  int newTempo = tempoSet;
  bool tempoChanged = abs(newTempo - baseTempo) > TEMPO_CHANGE_THRESHOLD;
  
  // Determine tempo range for pattern selection (smaller, overlapping ranges)
  int newTempoRange;
  if (newTempo < 80) {
    newTempoRange = 0; // Slow (30-79 BPM)
  } else if (newTempo < 120) {
    newTempoRange = 1; // Medium (80-119 BPM)
  } else {
    newTempoRange = 2; // Fast (120-160 BPM)
  }
  
  // Reset bass note index when tempo range changes
  if (newTempoRange != currentTempoRange) {
    currentTempoRange = newTempoRange;
    bassNoteIndex = 0;
  }
  
  if (tempoChanged) {  // Only update if significantly different
    tempo = newTempo;
    baseTempo = newTempo;
    targetTempo = baseTempo * tempoMultiplier;
    
    // Start gradual tempo transition instead of instant change
    if (!tempoTransitioning) {
      tempoTransitioning = true;
      tempoTransitionSteps = 0;
      startTransitionTempo = effectiveTempo; // Use current effective tempo as starting point
    }
  }
  
  // Use mapped bass note length value
  int newBassNoteLength = bassNoteLengthSet;
  if (abs(newBassNoteLength - bassNoteLength) > BASS_LENGTH_CHANGE_THRESHOLD) { // Small threshold to prevent jitter
    bassNoteLength = newBassNoteLength;
  }
  
  // Use mapped swing value - no threshold for smooth control
  swingAmount = swingSet;
}

/**
 * @brief Manages tempo transitions and pattern length changes
 * 
 * Handles gradual tempo transitions to avoid audio artifacts,
 * manages pattern length updates, and calculates step timing.
 */
void handleTempoTransitions() {
  // Hardcode pattern length to 16 steps (full pattern)
  int newPatternLength = 16;
  int newTempoMultiplier = 1;
  
  bool patternChanged = (newPatternLength != currentPatternLength || newTempoMultiplier != tempoMultiplier);
  
  // Handle pattern changes - use gentler transitions to avoid artifacts
  if (patternChanged) {
    int oldTempoMultiplier = tempoMultiplier;
    
    currentPatternLength = newPatternLength;
    tempoMultiplier = newTempoMultiplier;
    targetTempo = baseTempo * tempoMultiplier;
    
    // For tempo multiplier changes (beat repeat), update immediately
    // For pattern length changes, let it transition naturally at next beat boundary
    if (newTempoMultiplier != oldTempoMultiplier) {
      effectiveTempo = targetTempo;
      int stepTime = 60000 / (effectiveTempo * 4);
      stepDelay.set(stepTime);
      baseStepTime = stepTime; // Update base step time for swing calculations
    }
    
    // Don't reset currentStep immediately - let it wrap naturally
    // This prevents audio glitches from abrupt sequencer resets
    if (currentStep >= currentPatternLength) {
      currentStep = 0; // Only reset if we're outside the new pattern bounds
    }
    
    // Stop any ongoing tempo transition since we're changing patterns
    tempoTransitioning = false;
  }
  
  // Handle gradual tempo transitions
  if (tempoTransitioning && !patternChanged) {
    // Calculate transition progress (0.0 to 1.0)
    float transitionProgress = (float)tempoTransitionSteps / tempoTransitionLength;
    
    if (transitionProgress >= 1.0) {
      // Transition complete
      effectiveTempo = targetTempo;
      tempoTransitioning = false;
    } else {
      // Interpolate between current and target tempo
      int startTempo = startTransitionTempo; // Use the starting tempo for interpolation
      effectiveTempo = startTempo + (int)((targetTempo - startTempo) * transitionProgress);
    }
    
    // Update step timing with new effective tempo
    int stepTime = 60000 / (effectiveTempo * 4);
    stepDelay.set(stepTime);
    baseStepTime = stepTime; // Update base step time when tempo changes
  }
  
  // Only recalculate base step time if it wasn't already updated above
  if (!tempoTransitioning && !patternChanged) {
    baseStepTime = 60000 / (effectiveTempo * 4);
  }
  
  // Safety check: ensure baseStepTime is always valid
  if (baseStepTime <= 0) {
    baseStepTime = 60000 / (180 * 4); // Fallback to 180 BPM
  }
}

/**
 * @brief Handles pattern selection using direct touchpad mapping
 * 
 * Uses kick_pattern_set and drumSet directly without complex remapping logic.
 * X-axis controls kick, Y-axis controls hi-hat, diagonal controls snare.
 */
void handleControlModeAndPatterns() {
  // Direct mapping - X-axis controls kick patterns (0-4)
  uint8_t newKickPattern = constrain(kick_pattern_set, 0, 4);
  if (newKickPattern != currentKickPattern) {
    currentKickPattern = newKickPattern;
    if (useSerial) {
      Serial.print(F("Kick: "));
      Serial.println(currentKickPattern);
    }
  }
  
  // Direct mapping - Y-axis controls hi-hat patterns (0-4)
  uint8_t newHihatPattern = constrain(drumSet, 0, 4);
  if (newHihatPattern != currentHihatPattern) {
    currentHihatPattern = newHihatPattern;
    if (useSerial) {
      Serial.print(F("Hi-hat: "));
      Serial.println(currentHihatPattern);
    }
  }
  
  // SNARE: When hi-hat is 0, force snare to 0 for "kick only" mode
  // Otherwise use diagonal mapping from bottom-left to top-right
  uint8_t newSnarePattern;
  if (drumSet == 0) {
    // Bottom of touchpad = kick only mode
    newSnarePattern = 0;
  } else {
    // Diagonal mapping for snare complexity (starts at drumSet = 1)
    float diagonalPosition = (float)(kick_pattern_set + (drumSet - 1)) / 7.0f; // 0-1 range
    newSnarePattern = (uint8_t)(diagonalPosition * 4.0f);
    newSnarePattern = constrain(newSnarePattern, 0, 4);
  }
  
  if (newSnarePattern != currentSnarePattern) {
    currentSnarePattern = newSnarePattern;
    if (useSerial) {
      Serial.print(F("Snare: "));
      Serial.println(currentSnarePattern);
    }
  }
  
  // Keep bass gain active for sequencer
  bassGain = 2;
  
  // MARKOV CHAIN BASS CONTROL - Map touchpad patterns to Markov complexity
  // X-axis (kick patterns 0-4) controls Markov complexity (0-4)
  // This provides smooth integration with existing pattern control
  uint8_t newMarkovComplexity = constrain(currentKickPattern, 0, 4);
  if (newMarkovComplexity != markovComplexity) {
    markovComplexity = newMarkovComplexity;
    if (useSerial) {
      Serial.print(F("Markov Complexity: "));
      Serial.println(markovComplexity);
    }
  }
}

/**
 * @brief Processes main sequencer step logic
 * 
 * Handles step-based triggering of bass, kick, and hi-hat patterns,
 * manages swing timing calculations, and advances the step counter.
 */
void processSequencerStep() {
  // Check if it's time for the next main step (16th notes for kick/bass)
  if (stepDelay.ready()) {
    // Update Markov pattern logic for repetition tracking
    updateMarkovPatternLogic();
    
    // Trigger bass using Markov timing AND bass note length > minimum
    if (getMarkovBassTiming(currentStep) && bassNoteLength > 22) {
      // Get the next MIDI note using Markov chain generation
      uint8_t midiNote = getBassNote();
      
      // Convert MIDI note to frequency and set bass frequencies
      bassFreq = midiNoteToFrequency(midiNote);
      currentBassMidiNote = midiNote; // Store for debugging
      
      // Set bass frequency (single oscillator for clean sound)
      bassOsc.setFreq(bassFreq);
      bassEnv.set(bassAttack, bassNoteLength); // Use dynamic note length
      bassEnv.start();
    }
    
    // Trigger kick drum if current step has kick
    if (getKickStep(currentStep)) {
      kickEnv.start();
    }
    
    // Trigger snare drum if current step has snare
    if (getSnareStep(currentStep)) {
      snareEnv.start();
    }
    
     // Trigger hi-hat - simple check once per step (16th notes)
     if (getHihatStep(currentStep)) {
       hihatEnv.start();
     }
    
    // Move to next step (always advance, pattern wraps automatically)
    currentStep = (currentStep + 1) % currentPatternLength;
    
    // Calculate swing timing for next step
    // Proper swing maintains overall tempo by compensating on-beats and off-beats
    isOffBeat = (currentStep % 2 == 1);
    
    int swingStepTime = baseStepTime;
    if (swingAmount != 0) {
      // Calculate swing adjustment
      int swingAdjustment = (baseStepTime * abs(swingAmount)) / 200; // Always positive adjustment
      
      if (swingAmount > 0) {
        // Positive swing (backward swing - off-beats delayed)
        if (isOffBeat) {
          swingStepTime = baseStepTime + swingAdjustment;
        } else {
          swingStepTime = baseStepTime - (swingAdjustment / 2);
        }
      } else {
        // Negative swing (forward swing - off-beats rushed)
        if (isOffBeat) {
          swingStepTime = baseStepTime - swingAdjustment;
        } else {
          swingStepTime = baseStepTime + (swingAdjustment / 2);
        }
      }
    }
    
    // Ensure we don't go below minimum timing (prevent negative or zero delays)
    if (swingStepTime < (baseStepTime / 4)) {
      swingStepTime = baseStepTime / 4;
    }
    
    // Ensure we have a valid step time (safety check)
    if (swingStepTime <= 0) {
      swingStepTime = baseStepTime > 0 ? baseStepTime : 200; // Fallback to 200ms
      if (debugSequencer && useSerial) {
        Serial.println(F("WARNING: Invalid step time, using fallback"));
      }
    }
    
    stepDelay.set(swingStepTime);
    stepDelay.start();
    
    // Debug output for sequencer timing (only if both flags enabled)
    if (debugSequencer && useSerial && (currentStep % 8 == 0)) {
      Serial.print(F("Step: "));
      Serial.print(currentStep);
      Serial.print(F(" Time: "));
      Serial.print(swingStepTime);
      Serial.print(F(" Base: "));
      Serial.println(baseStepTime);
    }
    
    // Advance tempo transition counter if transitioning
    if (tempoTransitioning) {
      tempoTransitionSteps++;
    }
  }
}

/**
 * @brief Updates audio envelopes and bass effects
 * 
 * Updates kick, hi-hat, and bass envelopes, handles bass phasing and beating effects,
 * and modulates oscillator frequencies for enhanced bass sound.
 */
void updateEnvelopesAndEffects() {
  // Update envelopes
  kickGain = kickEnv.next();
  hihatGain = hihatEnv.next();
  snareGain = snareEnv.next();
  bassGain = bassEnv.next();
  
  // Bass effects removed to save memory
}

/**
 * @brief Main control update function
 * 
 * Orchestrates all control processing using organized subfunctions.
 * Called at CONTROL_RATE frequency (256 Hz) for responsive control.
 */
void updateControl() {
  // Clean, organized control flow using subfunctions
  readInputs();
  processTempoAndTiming();
  handleTempoTransitions();
  handleControlModeAndPatterns();
  processSequencerStep();
  updateEnvelopesAndEffects();
}
#pragma endregion Control Helpers

//////////////////////////////////////////////////
//  A U D I O   O U T P U T //
//////////////////////////////////////////////////
#pragma region Audio Output

/**
 * @brief Main audio generation function
 * 
 * Generates and mixes kick drum, hi-hat, and bass signals for stereo output.
 * Called at AUDIO_RATE frequency for real-time audio synthesis.
 */
AudioOutput_t updateAudio() {
  // Generate kick drum signal
  int kickSignal = (kickOsc.next() * kickGain) >> KICK_GAIN_SHIFT;
  
  // Generate hi-hat signal (quieter)
  int hihatSignal = (hihatOsc.next() * hihatGain >> HIHAT_GAIN_SHIFT_1) >> HIHAT_GAIN_SHIFT_2;  // Made quieter
  
  // Generate 808-style snare signal (tonal-focused with minimal noise)
  int snareSignal = 0;
  if (snareGain > 0) {
    // Minimal noise component (808 snares are mostly tonal)
    int snareNoise = snareOsc.next() * snareGain >> SNARE_NOISE_SHIFT;
    
    // Dominant tonal component (the main 808 snare sound)
    int snareTonal = snareTone.next() * snareGain >> SNARE_TONAL_SHIFT;
    
    // 808-style enhancement: add harmonic distortion and punch
    int enhanced808 = snareTonal + (snareTonal >> 1); // Add 50% for 808 punch
    
    // Optional: Add slight pitch bend effect (808 characteristic)
    // Scale tonal component based on envelope for pitch bend simulation
    int pitchBendEffect = (enhanced808 * snareGain) >> 8; // Envelope-based scaling
    
    // Combine: mostly tonal with tiny bit of noise (classic 808 recipe)
    snareSignal = pitchBendEffect + (snareNoise >> 2); // 4:1 ratio tonal:noise
  }
  
  // Generate clean bass signal (single oscillator)
  int bassSignal = 0;
  
  if (bassGain > 0) {
    // Clean bass - single oscillator for artifact-free sound
    bassSignal = (bassOsc.next() * bassGain) >> BASS_GAIN_SHIFT;
  }

  
  // Mix for stereo output
  // Left channel: Kick + Snare + Hi-hat + Bass
  //int leftSignal = kickSignal + snareSignal + bassSignal + (hihatSignal >> 1);
  
  // Right channel: Kick + Snare + Bass (+ some hi-hat for fullness)
  int rightSignal = kickSignal + snareSignal + bassSignal + hihatSignal;
  
  // Prevent clipping
  // if (leftSignal > 127) leftSignal = 127;
  // if (leftSignal < -128) leftSignal = -128;
  // if (rightSignal > 127) rightSignal = 127;
  // if (rightSignal < -128) rightSignal = -128;
  
  return StereoOutput::fromAlmostNBit(AUDIO_OUTPUT_BITS, rightSignal, rightSignal).clip();
}
#pragma endregion Audio Output

//////////////////////////////////////////////////
//  L O O P //
//////////////////////////////////////////////////
#pragma region LOOP
/**
 * @brief Main program loop
 * 
 * Continuously calls audioHook() to maintain real-time audio processing.
 * No other code should be added here to avoid audio interruption.
 */
void loop() {
  audioHook();
}
#pragma endregion LOOP

/***** TOUCHPAD HELPERS ****/
#pragma region Touchpad Helpers

// Legacy function removed to save memory

/**
 * @brief Reads Y coordinate from touchpad
 * 
 * Configures pins for Y-axis reading and returns analog value.
 * Uses Mozzi's fast analog read for optimal performance.
 */
int ycoor(){
  pinMode(TOUCHPAD_Y1, OUTPUT);
  pinMode(TOUCHPAD_Y2, OUTPUT);
  pinMode(TOUCHPAD_X1, INPUT);
  pinMode(TOUCHPAD_X2, INPUT);
  digitalWrite(TOUCHPAD_Y1, HIGH);
  digitalWrite(TOUCHPAD_Y2, LOW);
  return mozziAnalogRead<10>(TOUCHPAD_X1);
}

/**
 * @brief Reads X coordinate from touchpad
 * 
 * Configures pins for X-axis reading and returns analog value.
 * Uses Mozzi's fast analog read for optimal performance.
 */
int xcoor(){
  pinMode(TOUCHPAD_X1, OUTPUT);
  pinMode(TOUCHPAD_X2, OUTPUT);
  pinMode(TOUCHPAD_Y1, INPUT);
  pinMode(TOUCHPAD_Y2, INPUT);
  digitalWrite(TOUCHPAD_X1, LOW);
  digitalWrite(TOUCHPAD_X2, HIGH);
  return mozziAnalogRead<10>(TOUCHPAD_Y1);
}
#pragma endregion Touchpad Helpers
#pragma endregion FUNCTIONS

//////////////////////////////////////////////////
//  E N D   C O D E  //
//////////////////////////////////////////////////