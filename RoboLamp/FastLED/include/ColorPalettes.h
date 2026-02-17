#ifndef COLOR_PALETTES_H
#define COLOR_PALETTES_H

#include <FastLED.h>

// Gradient palette definitions (PROGMEM)
extern const TProgmemRGBGradientPalette_byte FairyLightColors_gp[];
extern const TProgmemRGBGradientPalette_byte FullRainbowColors_gp[];

// Runtime palettes decoded from gradients
extern CRGBPalette16 FairyLightColors_p;
extern CRGBPalette16 FullRainbowColors_p;

// Palette selector function
CRGBPalette16 getPaletteForIndex(uint8_t index);

// Must be called in setup() to initialize gradient-based palettes
void initPalettes();

#endif // COLOR_PALETTES_H