#pragma once

#include <Arduino.h>
#include <Adafruit_NeoPixel.h>

class TimeKeeper;
class PanelColorController;

class UnderGlow {
public:
  UnderGlow(uint8_t pin, uint16_t numPixels);

  void begin(uint8_t brightness = 255);

  void update(const PanelColorController &panels,
              TimeKeeper                 &nixie,
              bool showTimeMode,
              bool timerRunning,
              bool alarmRunning);

  // Independent color position for badge / indicators
  void setColorPos(uint8_t pos) { _colorPos = pos; }
  uint8_t getColorPos() const   { return _colorPos; }

  // For web UI color dot
  void getCurrentColor(uint8_t &r, uint8_t &g, uint8_t &b) const;

  // Colon color overrides
  void setColonColors(uint32_t left, uint32_t right) {
    _colonLeftColor  = left;
    _colonRightColor = right;
  }

  // 🔥 NEW: brightness control in 0–100%
  void setBrightnessPct(uint8_t pct);
  uint8_t getBrightnessPct() const;

private:
  Adafruit_NeoPixel strip;
  uint16_t _numPixels;
  uint8_t  _lastSecond;
  bool     _blinkOn;
  uint8_t  _colorPos;

  uint32_t _colonLeftColor;
  uint32_t _colonRightColor;

  uint8_t  _brightnessPct;    // store brightness as 0–100%

  uint32_t wheel(uint8_t pos) const;
};
