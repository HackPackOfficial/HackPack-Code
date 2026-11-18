#pragma once

#include <Arduino.h>
#include <avr/pgmspace.h>

#define CRUNCH_LOGO_WIDTH 14
#define CRUNCH_LOGO_HEIGHT 14
#define CRUNCH_LOGO_FRAME_COUNT 8

extern const uint8_t CRUNCH_LOGO_FRAMES[CRUNCH_LOGO_FRAME_COUNT][28] PROGMEM;
