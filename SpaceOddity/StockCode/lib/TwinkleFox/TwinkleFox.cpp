/// @file    TwinkleFox.cpp
/// @brief   TwinkleFox LED effect packaged as a reusable C++ class.
///
/// Original TwinkleFOX effect by Mark Kriegsman, December 2015.
/// Source: https://github.com/FastLED/FastLED/blob/master/examples/TwinkleFox/TwinkleFox.ino
///
/// This is a modified version of the TwinkleFox example from the FastLED library.
/// The effect logic is unchanged; it has been wrapped in a class so it can be
/// instantiated and driven from an existing project that already initializes
/// FastLED independently. See TwinkleFox.h for usage notes.

#include "TwinkleFox.h"

// =============================================================================
//  Color palettes
//  All palette definitions are file-scoped to this translation unit to avoid
//  ODR violations if this file is compiled alongside other FastLED projects
//  that define palettes with the same names.
// =============================================================================

// A mostly red palette with green accents and white trim.
// "CRGB::Gray" is used as white to keep the brightness more uniform.
static const TProgmemRGBPalette16 RedGreenWhite_p FL_PROGMEM = {
    CRGB::Red, CRGB::Red, CRGB::Red,  CRGB::Red,
    CRGB::Red, CRGB::Red, CRGB::Red,  CRGB::Red,
    CRGB::Red, CRGB::Red, CRGB::Gray, CRGB::Gray,
    CRGB::Green, CRGB::Green, CRGB::Green, CRGB::Green
};

// A mostly (dark) green palette with red berries.
#define Holly_Green 0x00580c
#define Holly_Red   0xB00402
static const TProgmemRGBPalette16 Holly_p FL_PROGMEM = {
    Holly_Green, Holly_Green, Holly_Green, Holly_Green,
    Holly_Green, Holly_Green, Holly_Green, Holly_Green,
    Holly_Green, Holly_Green, Holly_Green, Holly_Green,
    Holly_Green, Holly_Green, Holly_Green, Holly_Red
};

// A red and white striped palette.
// "CRGB::Gray" is used as white to keep the brightness more uniform.
static const TProgmemRGBPalette16 RedWhite_p FL_PROGMEM = {
    CRGB::Red,  CRGB::Red,  CRGB::Red,  CRGB::Red,
    CRGB::Gray, CRGB::Gray, CRGB::Gray, CRGB::Gray,
    CRGB::Red,  CRGB::Red,  CRGB::Red,  CRGB::Red,
    CRGB::Gray, CRGB::Gray, CRGB::Gray, CRGB::Gray
};

// A mostly blue palette with white accents.
// "CRGB::Gray" is used as white to keep the brightness more uniform.
static const TProgmemRGBPalette16 BlueWhite_p FL_PROGMEM = {
    CRGB::Blue, CRGB::Blue, CRGB::Blue, CRGB::Blue,
    CRGB::Blue, CRGB::Blue, CRGB::Blue, CRGB::Blue,
    CRGB::Blue, CRGB::Blue, CRGB::Blue, CRGB::Blue,
    CRGB::Blue, CRGB::Gray, CRGB::Gray, CRGB::Gray
};

// A pure "fairy light" palette with some brightness variations.
#define HALFFAIRY    ((CRGB::FairyLight & 0xFEFEFE) / 2)
#define QUARTERFAIRY ((CRGB::FairyLight & 0xFCFCFC) / 4)
static const TProgmemRGBPalette16 FairyLight_p FL_PROGMEM = {
    CRGB::FairyLight, CRGB::FairyLight, CRGB::FairyLight, CRGB::FairyLight,
    HALFFAIRY,        HALFFAIRY,        CRGB::FairyLight, CRGB::FairyLight,
    QUARTERFAIRY,     QUARTERFAIRY,     CRGB::FairyLight, CRGB::FairyLight,
    CRGB::FairyLight, CRGB::FairyLight, CRGB::FairyLight, CRGB::FairyLight
};

// A palette of soft snowflakes with the occasional bright one.
static const TProgmemRGBPalette16 Snow_p FL_PROGMEM = {
    0x304048, 0x304048, 0x304048, 0x304048,
    0x304048, 0x304048, 0x304048, 0x304048,
    0x304048, 0x304048, 0x304048, 0x304048,
    0x304048, 0x304048, 0x304048, 0xE0F0FF
};

// A palette reminiscent of large 'old-school' C9-size tree lights
// in the five classic colors: red, orange, green, blue, and white.
#define C9_Red    0xB80400
#define C9_Orange 0x902C02
#define C9_Green  0x046002
#define C9_Blue   0x070758
#define C9_White  0x606820
static const TProgmemRGBPalette16 RetroC9_p FL_PROGMEM = {
    C9_Red,    C9_Orange, C9_Red,    C9_Orange,
    C9_Orange, C9_Red,    C9_Orange, C9_Red,
    C9_Green,  C9_Green,  C9_Green,  C9_Green,
    C9_Blue,   C9_Blue,   C9_Blue,   C9_White
};

// A cold, icy pale blue palette.
#define Ice_Blue1 0x0C1040
#define Ice_Blue2 0x182080
#define Ice_Blue3 0x5080C0
static const TProgmemRGBPalette16 Ice_p FL_PROGMEM = {
    Ice_Blue1, Ice_Blue1, Ice_Blue1, Ice_Blue1,
    Ice_Blue1, Ice_Blue1, Ice_Blue1, Ice_Blue1,
    Ice_Blue1, Ice_Blue1, Ice_Blue1, Ice_Blue1,
    Ice_Blue2, Ice_Blue2, Ice_Blue2, Ice_Blue3
};

// Add or remove palette names from this list to control which color
// palettes are used, and in what order.
static const TProgmemRGBPalette16* ActivePaletteList[] = {
    &RetroC9_p,
    &BlueWhite_p,
    &RainbowColors_p,
    &FairyLight_p,
    &RedGreenWhite_p,
    &PartyColors_p,
    &RedWhite_p,
    &Snow_p,
    &Holly_p,
    &Ice_p
};


// =============================================================================
//  TwinkleFox implementation
// =============================================================================

TwinkleFox::TwinkleFox(CRGB* leds, uint16_t numLeds)
    : _leds(leds)
    , _numLeds(numLeds)
    , _twinkleSpeed(4)
    , _twinkleDensity(5)
    , _secondsPerPalette(30)
    , _coolLikeIncandescent(true)
    , _autoSelectBackground(false)
    , _backgroundColor(CRGB::Black)
    , _lastPaletteChangeMs(0)
    , _lastBlendMs(0)
{
    chooseNextColorPalette(_targetPalette);
    _currentPalette = _targetPalette;
}

// -----------------------------------------------------------------------------
//  Public interface
// -----------------------------------------------------------------------------

void TwinkleFox::run() {
    update();
    FastLED.show();
}

void TwinkleFox::update() {
    uint32_t now = millis();

    if (now - _lastPaletteChangeMs >= ((uint32_t)_secondsPerPalette * 1000UL)) {
        chooseNextColorPalette(_targetPalette);
        _lastPaletteChangeMs = now;
    }

    if (now - _lastBlendMs >= 10) {
        nblendPaletteTowardPalette(_currentPalette, _targetPalette, 12);
        _lastBlendMs = now;
    }

    drawTwinkles();
}

void TwinkleFox::setTwinkleSpeed(uint8_t speed) {
    _twinkleSpeed = speed < 8 ? speed : 8;
}

void TwinkleFox::setTwinkleDensity(uint8_t density) {
    _twinkleDensity = density < 8 ? density : 8;
}

void TwinkleFox::setSecondsPerPalette(uint16_t seconds) {
    _secondsPerPalette = seconds;
}

void TwinkleFox::setCoolLikeIncandescent(bool enabled) {
    _coolLikeIncandescent = enabled;
}

void TwinkleFox::setBackgroundColor(CRGB color) {
    _backgroundColor = color;
}

void TwinkleFox::setAutoSelectBackground(bool enabled) {
    _autoSelectBackground = enabled;
}

// -----------------------------------------------------------------------------
//  Private helpers
// -----------------------------------------------------------------------------

// Advance to the next color palette in the list (above).
void TwinkleFox::chooseNextColorPalette(CRGBPalette16& pal) {
    const uint8_t numberOfPalettes =
        sizeof(ActivePaletteList) / sizeof(ActivePaletteList[0]);
    static uint8_t whichPalette = -1;
    whichPalette = addmod8(whichPalette, 1, numberOfPalettes);
    pal = *(ActivePaletteList[whichPalette]);
}

// This function is like 'triwave8', which produces a
// symmetrical up-and-down triangle sawtooth waveform, except that this
// function produces a triangle wave with a faster attack and a slower decay:
//
//     / \ 
//    /     \ 
//   /         \ 
//  /             \ 
//
uint8_t TwinkleFox::attackDecayWave8(uint8_t i) {
    if (i < 86) {
        return i * 3;
    } else {
        i -= 86;
        return 255 - (i + (i / 2));
    }
}

// This function takes a pixel, and if its in the 'fading down'
// part of the cycle, it adjusts the color a little bit like the
// way that incandescent bulbs fade toward 'red' as they dim.
void TwinkleFox::coolLikeIncandescentFn(CRGB& c, uint8_t phase) {
    if (phase < 128) return;

    uint8_t cooling = (phase - 128) >> 4;
    c.g = qsub8(c.g, cooling);
    c.b = qsub8(c.b, cooling * 2);
}

//  This function takes a time in pseudo-milliseconds,
//  figures out brightness = f( time ), and also hue = f( time )
//  The 'low digits' of the millisecond time are used as
//  input to the brightness wave function.
//  The 'high digits' are used to select a color, so that the color
//  does not change over the course of the fade-in, fade-out
//  of one cycle of the brightness wave function.
//  The 'high digits' are also used to determine whether this pixel
//  should light at all during this cycle, based on the TWINKLE_DENSITY.
CRGB TwinkleFox::computeOneTwinkle(uint32_t ms, uint8_t salt) {
    uint16_t ticks      = ms >> (8 - _twinkleSpeed);
    uint8_t  fastcycle8 = ticks;
    uint16_t slowcycle16 = (ticks >> 8) + salt;
    slowcycle16 += sin8(slowcycle16);
    slowcycle16  = (slowcycle16 * 2053) + 1384;
    uint8_t slowcycle8 = (slowcycle16 & 0xFF) + (slowcycle16 >> 8);

    uint8_t bright = 0;
    if (((slowcycle8 & 0x0E) / 2) < _twinkleDensity) {
        bright = attackDecayWave8(fastcycle8);
    }

    uint8_t hue = slowcycle8 - salt;
    CRGB c;
    if (bright > 0) {
        c = ColorFromPalette(_currentPalette, hue, bright, NOBLEND);
        if (_coolLikeIncandescent) {
            coolLikeIncandescentFn(c, fastcycle8);
        }
    } else {
        c = CRGB::Black;
    }
    return c;
}

//  This function loops over each pixel, calculates the
//  adjusted 'clock' that this pixel should use, and calls
//  "computeOneTwinkle" on each pixel.  It then displays
//  either the twinkle color or the background color,
//  whichever is brighter.
void TwinkleFox::drawTwinkles() {
    // "PRNG16" is the pseudorandom number generator.
    // It MUST be reset to the same starting value each time
    // this function is called, so that the sequence of 'random'
    // numbers that it generates is (paradoxically) stable.
    uint16_t PRNG16   = 11337;
    uint32_t clock32  = millis();

    // Set up the background color, "bg".
    // if _autoSelectBackground is true, and the first two colors of
    // the current palette are identical, then a deeply faded version of
    // that color is used for the background color.
    CRGB bg;
    if (_autoSelectBackground && (_currentPalette[0] == _currentPalette[1])) {
        bg = _currentPalette[0];
        uint8_t bglight = bg.getAverageLight();
        if (bglight > 64) {
            bg.nscale8_video(16); // very bright, so scale to 1/16th
        } else if (bglight > 16) {
            bg.nscale8_video(64); // not that bright, so scale to 1/4th
        } else {
            bg.nscale8_video(86); // dim, scale to 1/3rd
        }
    } else {
        bg = _backgroundColor; // just use the explicitly defined background color
    }

    uint8_t backgroundBrightness = bg.getAverageLight();

    for (uint16_t i = 0; i < _numLeds; i++) {
        PRNG16 = (uint16_t)(PRNG16 * 2053) + 1384; // next 'random' number
        uint16_t myclockoffset16 = PRNG16;          // use that number as clock offset
        PRNG16 = (uint16_t)(PRNG16 * 2053) + 1384; // next 'random' number
        // use that number as clock speed adjustment factor (in 8ths, from 8/8ths to 23/8ths)
        uint8_t  myspeedmultiplierQ5_3 =
            ((((PRNG16 & 0xFF) >> 4) + (PRNG16 & 0x0F)) & 0x0F) + 0x08;
        uint32_t myclock30 =
            (uint32_t)((clock32 * myspeedmultiplierQ5_3) >> 3) + myclockoffset16;
        uint8_t myunique8 = PRNG16 >> 8; // get 'salt' value for this pixel

        // We now have the adjusted 'clock' for this pixel; call the function
        // that computes the color based on the "brightness = f( time )" idea.
        CRGB c = computeOneTwinkle(myclock30, myunique8);

        uint8_t  cbright     = c.getAverageLight();
        int16_t  deltabright = cbright - backgroundBrightness;

        if (deltabright >= 32 || (!bg)) {
            // If the new pixel is significantly brighter than the background color,
            // use the new color.
            _leds[i] = c;
        } else if (deltabright > 0) {
            // If the new pixel is just slightly brighter than the background color,
            // mix a blend of the new color and the background color.
            _leds[i] = blend(bg, c, deltabright * 8);
        } else {
            // If the new pixel is not at all brighter than the background color,
            // just use the background color.
            _leds[i] = bg;
        }
    }
}
