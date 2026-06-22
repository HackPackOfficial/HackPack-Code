#include "UnderGlow.h"
#include "PanelColorController.h"
#include "TimeKeeper.h"
#include "config.h"

#include "UnderGlow.h"
#include "TimeKeeper.h"

// How many LEDs per 7-segment panel
static const int LEDS_PER_PANEL = 7;

// *Internal Function* Average all 7 LEDs in a panel into one color
static uint32_t averagePanelColor(const PanelColorController &panels, int panelIndex)
{
  if (panelIndex < 0) return 0;

  uint32_t sumR = 0, sumG = 0, sumB = 0;
  int count = 0;

  int base = panelIndex * LEDS_PER_PANEL;

  for (int i = 0; i < LEDS_PER_PANEL; ++i) {
    uint32_t c = panels.getPixelColor(base + i);   // <-- uses PanelColorController API
    uint8_t r = (c >> 16) & 0xFF;
    uint8_t g = (c >>  8) & 0xFF;
    uint8_t b =  c        & 0xFF;

    sumR += r;
    sumG += g;
    sumB += b;
    ++count;
  }

  if (count == 0) return 0;

  uint8_t r = sumR / count;
  uint8_t g = sumG / count;
  uint8_t b = sumB / count;

  // We only ever feed this back into setPixelColor(), so we can keep the same 0xRRGGBB format
  return ( (uint32_t)r << 16 ) | ( (uint32_t)g << 8 ) | b;
}

// Blend two panel averages together (simple 50/50 mix)
static uint32_t blendTwoPanels(const PanelColorController &panels,
                               int leftPanel,
                               int rightPanel)
{
  uint32_t c1 = averagePanelColor(panels, leftPanel);
  uint32_t c2 = averagePanelColor(panels, rightPanel);

  uint8_t r1 = (c1 >> 16) & 0xFF;
  uint8_t g1 = (c1 >>  8) & 0xFF;
  uint8_t b1 =  c1        & 0xFF;

  uint8_t r2 = (c2 >> 16) & 0xFF;
  uint8_t g2 = (c2 >>  8) & 0xFF;
  uint8_t b2 =  c2        & 0xFF;

  uint8_t r = (uint16_t(r1) + r2) / 2;
  uint8_t g = (uint16_t(g1) + g2) / 2;
  uint8_t b = (uint16_t(b1) + b2) / 2;

  return ( (uint32_t)r << 16 ) | ( (uint32_t)g << 8 ) | b;
}

// Constructor
UnderGlow::UnderGlow(uint8_t pin, uint16_t numPixels)
  : strip(numPixels, pin, NEO_GRB + NEO_KHZ800),
    _numPixels(numPixels),
    _lastSecond(255),
    _blinkOn(false),
    _colorPos(glow_col),
    _brightnessPct((glowBrightness / 255.0) * 100.0),
    _colonLeftColor(0),
    _colonRightColor(0)
{
}

void UnderGlow::begin(uint8_t brightness)
{
  strip.begin();
  // use _brightnessPct to decide actual brightness
  uint8_t level = (uint16_t)_brightnessPct * 255 / 100;
  strip.setBrightness(level);
  strip.show();
}

// Simple color wheel (similar to PanelColorController::Wheel)
uint32_t UnderGlow::wheel(uint8_t pos) const
{
  pos = 255 - pos;
  if (pos < 85) {
    return strip.Color(255 - pos * 3, 0, pos * 3);
  } else if (pos < 170) {
    pos -= 85;
    return strip.Color(0, pos * 3, 255 - pos * 3);
  } else {
    pos -= 170;
    return strip.Color(pos * 3, 255 - pos * 3, 0);
  }
}

void UnderGlow::getCurrentColor(uint8_t &r, uint8_t &g, uint8_t &b) const
{
  uint32_t c = wheel(_colorPos);
  r = (c >> 16) & 0xFF;
  g = (c >> 8)  & 0xFF;
  b =  c        & 0xFF;
}

void UnderGlow::setBrightnessPct(uint8_t pct)
{
  if (pct > 100) pct = 100;
  _brightnessPct = pct;

  uint8_t level = (uint16_t)pct * 255 / 100;
  strip.setBrightness(level);
}

uint8_t UnderGlow::getBrightnessPct() const
{
  return _brightnessPct;
}


void UnderGlow::update(const PanelColorController &panels,
                       TimeKeeper                 &nixie,
                       bool showTimeMode,
                       bool timerRunning,
                       bool alarmRunning)
{
  strip.clear();

  // Base under-glow color (badge + fallback for colons)
  uint32_t base = wheel(_colorPos);

  // --- blinking state based on seconds --- //
  uint8_t sec = nixie.getTime().c_sec;
  if (sec != _lastSecond) {
    _lastSecond = sec;
    _blinkOn    = !_blinkOn;
  }

  // --- COLONS (LEDs 0 and 1) --- //
  if (_blinkOn && _numPixels >= 2) {
    uint32_t col0, col1;

    if (showTimeMode) {
      // In TIME mode: blend adjacent panels for a nice aesthetic
      col0 = blendTwoPanels(panels, 1, 2);  // between panels 1 & 2
      col1 = blendTwoPanels(panels, 3, 4);  // between panels 3 & 4
    } else {
      // In non-TIME modes (menus/timer/etc.), fall back to explicit colon colors,
      // or to the base color if they are 0.
      uint32_t colonLeft  = (_colonLeftColor  != 0) ? _colonLeftColor  : base;
      uint32_t colonRight = (_colonRightColor != 0) ? _colonRightColor : base;
      col0 = colonLeft;
      col1 = colonRight;
    }

    strip.setPixelColor(0, col0);
    strip.setPixelColor(1, col1);
  }

  // --- badge glow (LEDs 2..9) --- //
  for (int i = 2; i < 10 && i < _numPixels; ++i) {
    strip.setPixelColor(i, base);
  }

  // --- indicators: AM/PM + alarm --- //
  if (showTimeMode && !timerRunning) {
    if (nixie.getTime().isPm) {
      if (11 < _numPixels) strip.setPixelColor(11, strip.Color(0, 0, 0));          // AM off
      if (12 < _numPixels) strip.setPixelColor(12, strip.Color(80, 20, 255));      // PM purple
    } else {
      if (11 < _numPixels) strip.setPixelColor(11, strip.Color(255, 140, 20));     // AM amber
      if (12 < _numPixels) strip.setPixelColor(12, strip.Color(0, 0, 0));          // PM off
    }

    if (alarmRunning && 10 < _numPixels) {
      strip.setPixelColor(10, strip.Color(255, 0, 0));                             // red alarm
    }
  }

  strip.show();
}
