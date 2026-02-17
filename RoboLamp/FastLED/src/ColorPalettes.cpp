#include "ColorPalettes.h"

// --- Fairy Light Palette (whites, pinks, purples) ---
DEFINE_GRADIENT_PALETTE(FairyLightColors_gp){
    0, 255, 220, 220,   // Light pink
    48, 255, 160, 225,  // Soft magenta
    96, 230, 120, 210,  // Purple-pink
    128, 255, 200, 245, // Light lavender
    160, 255, 140, 210, // More saturated magenta
    192, 255, 180, 235, // Softer pink again
    224, 255, 200, 210, // Back to lavender-pink
    255, 255, 255, 255  // White
};

// --- Full Rainbow Palette (continuous spectrum) ---
DEFINE_GRADIENT_PALETTE(FullRainbowColors_gp) {
  0, 255, 0, 0,  // Red
  36, 255, 0, 255,  // Pink
  72, 0, 8, 255,  // Blue
  108, 43, 255, 0,  // Green
  144, 255, 247, 0,  // Yellow
  180, 255, 123, 0,  // Orange
  216, 128, 0, 255,  // Purple
  255, 0, 255, 238,  // Teal
};

// --- Decoded runtime palettes ---
CRGBPalette16 FairyLightColors_p;
CRGBPalette16 FullRainbowColors_p;

// --- Init function to decode gradients into palettes ---
void initPalettes()
{
    FairyLightColors_p = CRGBPalette16(FairyLightColors_gp);
    FullRainbowColors_p = CRGBPalette16(FullRainbowColors_gp);
}

CRGBPalette16 getPaletteForIndex(uint8_t index)
{
    switch (index)
    {
    case 0:
        return LavaColors_p;
    case 1:
        return HeatColors_p;
    case 2:
        return RainbowColors_p;
    case 3:
        return FullRainbowColors_p;
    case 4:
        return ForestColors_p;
    case 5:
        return OceanColors_p;
    case 6:
        return CloudColors_p;
    case 7:
        return PartyColors_p;
    case 8:
        return FairyLightColors_p;
    default:
        return RainbowColors_p;
    }
}