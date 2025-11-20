# SYNTH_CHORDBUILDER9 - File Structure Documentation

## Overview
The synthesizer code has been reorganized into a modular structure for better maintainability, readability, and user customization. The code is now split into logical components with clear separation between user-configurable settings and system internals.

## File Structure

### 📁 **Main Files**
- **`Cloud9ChordBuilder.ino`** - Main Arduino sketch file
  - Contains setup(), loop(), updateControl()
  - Global variable declarations
  - Smooth object initialization
  - Minimal, focused on core structure

### 📁 **Configuration Files**
- **`config.h`** - User-configurable settings ⚙️ **SAFE TO MODIFY**
  - Touchpad calibration values
  - Musical ranges (MIDI notes, chord tables)
  - Control smoothing factors
  - Hysteresis settings
  - LFO system parameters
  - Audio mixing settings
  - Serial output rate
  - All values include suggested ranges in comments

- **`system_config.h`** - Internal system constants ⚠️ **MODIFY WITH CAUTION**
  - Core audio engine constants (CONTROL_RATE, etc.)
  - System limits (WAVE_COUNT, CHORD_COUNT, etc.)
  - MIDI and frequency constants
  - Touchpad mapping constants

### 📁 **Function Libraries**
- **`control_functions.h/.cpp`** - Input processing and control logic
  - `readInputs()` - Touchpad and potentiometer reading
  - `gainControl()` - Volume control with finger detection
  - `setChordAndInversions()` - Chord selection with hysteresis
  - `processFrequencyAndWave()` - Frequency and wave processing
  - `ycoor()`, `xcoor()` - Touchpad coordinate functions
  - `updateSpinnerLFO()`, `getLFOModulation()` - LFO system
  - `printSynthData()`, `printControlData()`, `printNoteData()` - Serial output

- **`audio_functions.h/.cpp`** - Audio generation and processing
  - `updateOscillatorFrequencies()` - Frequency calculation and voice mapping
  - `updateAudio()` - Audio mixing and output generation

### 📁 **Data Files** (Existing)
- **`wavetables.h`** - Oscillator definitions and wavetable includes
- **`chord_tables.h`** - All chord progression definitions
- **`inversion_patterns.cpp`** - Inversion pattern lookup table

### 📁 **Backup**
- **`SYNTH_CHORDBUILDER9_BACKUP.ino`** - Original monolithic file (backup)

## Benefits of New Structure

### 🎯 **For Users**
- **Easy Customization**: All user settings clearly separated in `config.h`
- **Safe Modification**: Clear distinction between safe-to-modify and system files
- **Better Documentation**: Each setting includes suggested value ranges
- **Focused Editing**: No need to scroll through 1100+ lines to find settings

### 🔧 **For Developers**
- **Modular Design**: Related functions grouped logically
- **Maintainable**: Easier to debug and modify specific subsystems
- **Readable**: Main file focuses on structure, not implementation details
- **Reusable**: Control and audio functions can be reused in other projects

### 📊 **Code Organization**
- **Separation of Concerns**: Configuration, control, audio, and data clearly separated
- **Reduced Complexity**: Main file reduced from 1100+ lines to ~250 lines
- **Better Testing**: Individual modules can be tested independently
- **Professional Structure**: Industry-standard file organization

## Usage Guide

### 🎛️ **Customizing the Synthesizer**
1. **Basic Settings**: Edit `config.h` for touchpad calibration, musical ranges
2. **Performance Tuning**: Adjust smoothing factors and hysteresis in `config.h`
3. **LFO Behavior**: Modify LFO constants in `config.h`
4. **Advanced Changes**: Only modify `system_config.h` if you understand the implications

### 🔍 **Debugging**
1. **Control Issues**: Check `control_functions.cpp` and enable serial output in `config.h`
2. **Audio Problems**: Examine `audio_functions.cpp` and audio mixing settings
3. **System Issues**: Verify `system_config.h` constants match your hardware

### 🚀 **Compilation**
- All files must be in the same directory as the main `.ino` file
- Arduino IDE will automatically compile all `.cpp` files
- Include paths are handled automatically by the IDE

## File Sizes (Approximate)
- **Main .ino**: ~250 lines (was 1100+)
- **config.h**: ~70 lines
- **system_config.h**: ~40 lines  
- **control_functions.cpp**: ~400 lines
- **audio_functions.cpp**: ~200 lines
- **Total**: Similar line count, much better organized

## Migration Notes
- Original file backed up as `SYNTH_CHORDBUILDER9_BACKUP.ino`
- All functionality preserved
- Configuration values moved to `config.h` with same defaults
- Function behavior unchanged
- Serial output now controlled by `USE_SERIAL` define in `config.h`

---
*This modular structure makes the synthesizer more professional, maintainable, and user-friendly while preserving all original functionality.*
