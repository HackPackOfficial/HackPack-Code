#include <Arduino.h>
#include "PanelColorController.h"
#include "config.h"

// Hardware
#define LED_PIN    0
#define LED_COUNT  42  // 6 panels × 7 segments

PanelColorController panels(LED_PIN, LED_COUNT);

// Marquee state
String scrollBuf;
uint16_t scrollPos = 0;
unsigned long nextTick = 0;

void earlyPinsAndSerial() {
  // Your global boot hack for BNO085 etc.
  pinMode(2, OUTPUT);
  digitalWrite(2, HIGH);
  Serial.begin(115200);
  delay(1000);
}

void marqueeBegin(const String& msg) {
  scrollBuf = "";

  // pad left so text scrolls in nicely
  for (int i = 0; i < 6; i++) scrollBuf += ' ';

  scrollBuf += msg;

  // pad right so it scrolls fully off
  for (int i = 0; i < 6; i++) scrollBuf += ' ';

  scrollPos = 0;
}

void marqueeTick() {
  if (millis() < nextTick) return;
  nextTick = millis() + MARQUEE_SCROLL_MS;   // from config.h

  // Build 6-character window
  char window[7];
  for (int i = 0; i < 6; i++) {
    window[i] = scrollBuf[(scrollPos + i) % scrollBuf.length()];
  }
  window[6] = '\0';

  scrollPos = (scrollPos + 1) % scrollBuf.length();

  // ---- Update glyphs & colors ----
  panels.showString(window);  // maps chars to segments using your glyph atlas

  // Animated color mode:
  // col_state is already initialized from bootMode in PanelColorController.cpp.
  panels.update();            // advances RAINBOW / FLOW / etc.
  panels.displayPanels();     // pushes display[] + displayColors[] to LEDs
}

void setup() {
  earlyPinsAndSerial();
  Serial.println("Nixie Marquee with PanelColorController + animated colors");

  panels.begin();   // uses bootBrightness, etc. from config

  // If you ever change bootMode or bootColor at runtime,
  // you can force a re-setup by:
  // panels.getColors().modeChanged = true;

  marqueeBegin(String(MARQUEE_MESSAGE));   // from config
}

void loop() {
  marqueeTick();
}
