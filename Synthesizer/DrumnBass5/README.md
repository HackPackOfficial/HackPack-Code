# DRUM & BASS MACHINE - Refactored Version

This is a refactored version of the original drum and bass synthesizer, organized to match the structure and style of the chord builder project.

## File Structure

### Main Files
- **DrumnBass5.ino** - Main sketch with pragma regions and organized structure

### Header Files
- **drum_patterns.h** - All kick and hi-hat pattern definitions
- **bass_sequences.h** - Bass timing patterns, note sequences, and scale definitions  
- **drum_oscillators.h** - Audio engine components and oscillator declarations

### Implementation Files
- **drum_patterns.cpp** - Pattern access functions
- **bass_sequences.cpp** - Bass sequence and scale generation functions

## Key Improvements

### Organization
- **Pragma regions** for clear code organization (README, LICENSE, LIBRARIES, SETUP, FUNCTIONS, etc.)
- **Modular header files** separate concerns (patterns, sequences, oscillators)
- **Consistent commenting style** matching the chord builder approach
- **Clean function organization** with helper functions grouped logically

### Control System
- **readInputs()** function matching chord builder style
- **Organized control flow** with dedicated subfunctions:
  - `readInputs()` - Read all sensors and pots
  - `processControls()` - Process tempo, swing, note length
  - `handleModeAndPatterns()` - Handle mode switching and pattern selection

### Features Preserved
- **Tempo-aware patterns** (Slow 60-100, Medium 100-140, Fast 140-200 BPM)
- **15 kick + 15 hi-hat patterns** across 3 tempo ranges
- **Sequenced bass** with 4 different scales
- **Dual control modes** (bass control vs pattern control)
- **Center-out pattern mapping** for intuitive performance
- **Advanced bass synthesis** with phasing and beating effects
- **Swing control** and tempo transitions

## Usage

The controls and functionality remain identical to the original version:

- **X-axis (touch)**: Bass patterns (bass mode) OR Kick patterns (pattern mode)
- **Y-axis (touch)**: Drum patterns (bass mode) OR Hi-hat patterns (pattern mode)  
- **botPot (A5)**: Bass note length (20-600ms)
- **midPot (A6)**: Tempo control (60-200 BPM)
- **hiPot (A7)**: Swing amount (-100 to +100)
- **Proximity sensor**: Mode switch (HIGH=bass mode, LOW=pattern mode)

## Benefits of Refactoring

1. **Better maintainability** - Code is organized into logical sections
2. **Easier to extend** - Adding new patterns or features is more straightforward
3. **Consistent style** - Matches other projects in the codebase
4. **Cleaner compilation** - Better separation of concerns reduces complexity
5. **Improved documentation** - Clear commenting and structure throughout