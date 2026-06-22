#pragma once
#include <Arduino.h>

// ===================== Panel / Clock Defaults =====================
// These are what PanelColorController expects already:
extern const bool Hr24Time;           // if you care (marquee: doesn't matter)
extern const bool displayLeadingZero; // same

extern const int  bootColor;          // starting color wheel position [0..255]
extern const int  bootBrightness;     // initial LED brightness 0..255
extern const uint8_t msg_col[3];      // base UI / message color

// 0..NUM_MODES-1  (from enum ColorState { RAINBOW, SOLID, GRADIENT, FLOW, WIPE, PULSE, BOUNCE, NUM_MODES })
extern const uint8_t bootMode;        // starting color mode index

// ===================== Marquee-Specific Settings =====================
extern const uint16_t MARQUEE_SCROLL_MS;         // ms per scroll step
extern const char MARQUEE_MESSAGE[];             // scrolling text
