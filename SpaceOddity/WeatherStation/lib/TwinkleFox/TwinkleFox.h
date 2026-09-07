/// @file    TwinkleFox.h
/// @brief   TwinkleFox LED effect packaged as a reusable C++ class.
///
/// Original TwinkleFOX effect by Mark Kriegsman, December 2015.
/// Source: https://github.com/FastLED/FastLED/blob/master/examples/TwinkleFox/TwinkleFox.ino
///
/// This is a modified version of the TwinkleFox example from the FastLED library.
/// The effect logic is unchanged; it has been wrapped in a class so it can be
/// instantiated and driven from an existing project that already initializes
/// FastLED independently. Call run() from your main loop to update and display
/// the effect. All palette data, timing state, and tuning parameters are
/// encapsulated per-instance.

#pragma once

#include <FastLED.h>

/// @class TwinkleFox
/// @brief Drives a twinkling holiday-light effect on a CRGB LED array.
///
/// Instantiate with a pointer to your existing CRGB array and its length.
/// The class does NOT call FastLED.addLeds() or FastLED.setMaxPower(); those
/// are assumed to be handled by the host project. run() does call
/// FastLED.show(), so do not call it separately on the same frame if you
/// intend this effect to be the sole output for that frame. If you are
/// multiplexing multiple effects, gate your FastLED.show() calls yourself
/// and use update() instead, which performs all state updates without calling
/// FastLED.show().
class TwinkleFox {
public:
    /// @param leds     Pointer to the CRGB array managed by the host project.
    /// @param numLeds  Number of LEDs in the array.
    TwinkleFox(CRGB* leds, uint16_t numLeds);

    /// Update effect state and call FastLED.show().
    void run();

    /// Update effect state only; does NOT call FastLED.show().
    /// Use this if you are compositing multiple effects and want to control
    /// when show() is called yourself.
    void update();

    // -------------------------------------------------------------------------
    // Tuning setters
    // -------------------------------------------------------------------------

    /// Overall twinkle speed. Range 0 (very slow) to 8 (very fast). Default 4.
    void setTwinkleSpeed(uint8_t speed);

    /// Fraction of pixels lit at any moment. Range 0 (none) to 8 (all). Default 5.
    void setTwinkleDensity(uint8_t density);

    /// How many seconds each color palette is held before advancing. Default 30.
    void setSecondsPerPalette(uint16_t seconds);

    /// When true, colors shift toward red as they fade, mimicking incandescent
    /// bulb behavior. Default true.
    void setCoolLikeIncandescent(bool enabled);

    /// Explicit background color for unlit pixels. Default CRGB::Black.
    /// Has no effect when auto-select background is enabled.
    void setBackgroundColor(CRGB color);

    /// When enabled, if the first two palette entries are identical, a heavily
    /// dimmed version of that color is used as the background automatically.
    /// Default false.
    void setAutoSelectBackground(bool enabled);

private:
    CRGB*    _leds;
    uint16_t _numLeds;

    uint8_t  _twinkleSpeed;
    uint8_t  _twinkleDensity;
    uint16_t _secondsPerPalette;
    bool     _coolLikeIncandescent;
    bool     _autoSelectBackground;
    CRGB     _backgroundColor;

    CRGBPalette16 _currentPalette;
    CRGBPalette16 _targetPalette;

    uint32_t _lastPaletteChangeMs;
    uint32_t _lastBlendMs;

    void     chooseNextColorPalette(CRGBPalette16& pal);
    void     drawTwinkles();
    CRGB     computeOneTwinkle(uint32_t ms, uint8_t salt);
    uint8_t  attackDecayWave8(uint8_t i);
    void     coolLikeIncandescentFn(CRGB& c, uint8_t phase);
};
