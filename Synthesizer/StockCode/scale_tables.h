#pragma once
/*
  ************************************************************************************
  * SCALE TABLES - Comprehensive scale system for synthesizer
  * 
  * Contains semitone interval definitions for various musical scales
  * Supports automatic scale generation and easy transposition
  *
  * Usage:
  * - Set currentScale (0-11) to select scale type
  * - Call generateScale(scaleIndex, rootNote) to build scale array
  * - Generated scale contains 3 octaves of notes
  *
  ************************************************************************************
*/

//////////////////////////////////////////////////////////////////////////////////////////////////////////
// SCALE SYSTEM VARIABLES
//////////////////////////////////////////////////////////////////////////////////////////////////////////

// Scale system state variables
uint8_t rootNote = 36;              // Starting root note (C1 = MIDI 36)
uint8_t currentScale = 0;           // Current scale index (0-11)
uint8_t generatedScale[21];         // Generated scale array (3 octaves)
uint8_t scaleLength = 7;            // Number of notes per octave in current scale

//////////////////////////////////////////////////////////////////////////////////////////////////////////
// SCALE DEFINITIONS (Semitone intervals from root)
//////////////////////////////////////////////////////////////////////////////////////////////////////////

// Traditional Western Scales
const uint8_t majorScale[] = {0, 2, 4, 5, 7, 9, 11};           // Ionian mode
const uint8_t minorScale[] = {0, 2, 3, 5, 7, 8, 10};           // Natural minor (Aeolian)
const uint8_t dorianScale[] = {0, 2, 3, 5, 7, 9, 10};          // Dorian mode
const uint8_t phrygianScale[] = {0, 1, 3, 5, 7, 8, 10};        // Phrygian mode
const uint8_t lydianScale[] = {0, 2, 4, 6, 7, 9, 11};          // Lydian mode
const uint8_t mixolydianScale[] = {0, 2, 4, 5, 7, 9, 10};      // Mixolydian mode

// Pentatonic Scales
const uint8_t pentatonicMajor[] = {0, 2, 4, 7, 9};             // Major pentatonic
const uint8_t pentatonicMinor[] = {0, 3, 5, 7, 10};            // Minor pentatonic

// Blues and Jazz Scales
const uint8_t bluesScale[] = {0, 3, 5, 6, 7, 10};              // Blues scale
const uint8_t jazzMinor[] = {0, 2, 3, 5, 7, 9, 11};           // Jazz melodic minor

// Exotic Scales
const uint8_t hungarianMinor[] = {0, 2, 3, 6, 7, 8, 11};       // Hungarian minor
const uint8_t arabicScale[] = {0, 1, 4, 5, 7, 8, 11};         // Arabic/Hijaz scale

//////////////////////////////////////////////////////////////////////////////////////////////////////////
// SCALE METADATA
//////////////////////////////////////////////////////////////////////////////////////////////////////////

// Scale lengths for each scale type
const uint8_t scaleLengths[] = {
  7,  // Major
  7,  // Minor  
  7,  // Dorian
  7,  // Phrygian
  7,  // Lydian
  7,  // Mixolydian
  5,  // Pentatonic Major
  5,  // Pentatonic Minor
  6,  // Blues
  7,  // Jazz Minor
  7,  // Hungarian Minor
  7   // Arabic
};

// Pointers to scale arrays for easy switching
const uint8_t* scaleTypes[] = {
  majorScale,
  minorScale,
  dorianScale,
  phrygianScale,
  lydianScale,
  mixolydianScale,
  pentatonicMajor,
  pentatonicMinor,
  bluesScale,
  jazzMinor,
  hungarianMinor,
  arabicScale
};

// Scale names for debugging and display
const char* scaleNames[] = {
  "Major",
  "Minor",
  "Dorian",
  "Phrygian", 
  "Lydian",
  "Mixolydian",
  "Pentatonic Major",
  "Pentatonic Minor",
  "Blues",
  "Jazz Minor",
  "Hungarian Minor",
  "Arabic"
};

// Total number of available scales
const uint8_t NUM_SCALES = 12;
