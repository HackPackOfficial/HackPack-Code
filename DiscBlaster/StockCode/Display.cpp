#include "BLSTR.h"
#include "Graphics.h"

// ---------- CrunchLabs armed / idle icon ----------

void drawCrunchLabsStateIcon() {
  static uint8_t crunchFrame = 0;
  static unsigned long lastCrunchFrameTime = 0;
  const unsigned long CRUNCH_FRAME_MS = 20;
  if (flywheelsEnabled) {
    if (millis() - lastCrunchFrameTime >= CRUNCH_FRAME_MS) {
      lastCrunchFrameTime = millis();
      crunchFrame++;
      if (crunchFrame >= CRUNCH_LOGO_FRAME_COUNT) {
        crunchFrame = 0;
      }
    }
  } else {
    crunchFrame = 0;
    lastCrunchFrameTime = millis();
  }
  display.drawBitmap(
    0,
    0,
    CRUNCH_LOGO_FRAMES[crunchFrame],
    CRUNCH_LOGO_WIDTH,
    CRUNCH_LOGO_HEIGHT,
    SSD1306_WHITE
  );
}

uint8_t activeReticleStyle = RETICLE_STYLE;

int drawBaseReticle() {
  const int cx = RETICLE_X;
  const int cy = RETICLE_Y;
  int y_offset = 0;
  switch (activeReticleStyle) {
    case 1: {
      // Trishot - three angular brackets with center dot

      // Top bracket
      display.drawLine(cx - 3, cy - 7, cx + 3, cy - 7, SSD1306_WHITE);
      display.drawLine(cx - 3, cy - 7, cx - 6, cy - 4, SSD1306_WHITE);
      display.drawLine(cx + 3, cy - 7, cx + 6, cy - 4, SSD1306_WHITE);

      // Bottom-left bracket
      display.drawLine(cx - 9, cy + 2, cx - 12, cy + 6, SSD1306_WHITE);
      display.drawLine(cx - 12, cy + 6, cx - 9, cy + 10, SSD1306_WHITE);
      display.drawLine(cx - 9, cy + 10, cx - 6, cy + 10, SSD1306_WHITE);

      // Bottom-right bracket
      display.drawLine(cx + 9, cy + 2, cx + 12, cy + 6, SSD1306_WHITE);
      display.drawLine(cx + 12, cy + 6, cx + 9, cy + 10, SSD1306_WHITE);
      display.drawLine(cx + 9, cy + 10, cx + 6, cy + 10, SSD1306_WHITE);

      // Center dot
      display.drawPixel(cx + 1, cy + 3, SSD1306_WHITE);
      display.drawPixel(cx - 1, cy + 3, SSD1306_WHITE);
      display.drawPixel(cx, cy + 4, SSD1306_WHITE);
      display.drawPixel(cx, cy + 2, SSD1306_WHITE);

      y_offset = 3;
      break;
    }
    case 2: {
      // Circular reticle with gaps at the four crosshair intersections.
      int radius = 8;
      int gap = 4;
      int arm = 12;

      if (batteryWakeMode) {
        display.drawCircle(cx, cy, radius, SSD1306_WHITE);

        // 5-pixel-wide cutouts at top, bottom, left, and right.
        display.fillRect(cx - 2, cy - radius - 2, 5, 5, SSD1306_BLACK);
        display.fillRect(cx - 2, cy + radius - 2, 5, 5, SSD1306_BLACK);
        display.fillRect(cx - radius - 2, cy - 2, 5, 5, SSD1306_BLACK);
        display.fillRect(cx + radius - 2, cy - 2, 5, 5, SSD1306_BLACK);

        // Original crosshair dashes unchanged.
        display.drawLine(cx - arm, cy, cx - gap, cy, SSD1306_WHITE);
        display.drawLine(cx + gap, cy, cx + arm, cy, SSD1306_WHITE);
        display.drawLine(cx, cy - arm, cx, cy - gap, SSD1306_WHITE);
        display.drawLine(cx, cy + gap, cx, cy + arm, SSD1306_WHITE);
      }

      y_offset = 0;
      break;
    }
    case 3: {
      // Double chevron: two stacked open upward-pointing V shapes.
      //top chevron
      display.drawLine(cx, cy + 4, cx - 3, cy + 7, SSD1306_WHITE);
      display.drawLine(cx, cy + 4, cx + 3, cy + 7, SSD1306_WHITE);
      //bottom chevron
      display.drawLine(cx, cy + 9, cx - 3, cy + 12, SSD1306_WHITE);
      display.drawLine(cx, cy + 9, cx + 3, cy + 12, SSD1306_WHITE);
      //dashes
      display.drawLine(cx - 11, cy, cx - 6, cy, SSD1306_WHITE);
      display.drawLine(cx + 6, cy, cx + 11, cy, SSD1306_WHITE);
      display.drawLine(cx, cy - 11, cx, cy - 6, SSD1306_WHITE);

      y_offset = 3;
      break;
    }
    case 4: {
      // Aircraft HUD reticle - curved wings
      const int r = 8;

      // Left wing
      display.drawLine(cx - 12, cy - 3, cx - 8, cy - 3, SSD1306_WHITE);
      display.drawPixel(cx - 7, cy - 2, SSD1306_WHITE);
      display.drawPixel(cx - 6, cy - 1, SSD1306_WHITE);
      display.drawPixel(cx - 5, cy, SSD1306_WHITE);

      // Right wing
      display.drawLine(cx + 8, cy - 3, cx + 12, cy - 3, SSD1306_WHITE);
      display.drawPixel(cx + 7, cy - 2, SSD1306_WHITE);
      display.drawPixel(cx + 6, cy - 1, SSD1306_WHITE);
      display.drawPixel(cx + 5, cy, SSD1306_WHITE);

      // Center ring
      display.drawCircle(cx, cy - 2, 1, SSD1306_WHITE);

      // Downward stem
      display.drawLine(cx, cy + 4, cx, cy + 7, SSD1306_WHITE);

      y_offset = 2;
      break;
    }
   case 5: {
      // Biohazard reticle - refined three-lobe design

      // Upper outer lobe
      display.drawLine(cx - 3, cy - 3, cx - 3, cy - 8, SSD1306_WHITE);
      display.drawLine(cx - 3, cy - 8, cx - 2, cy - 9, SSD1306_WHITE);
      display.drawLine(cx - 2, cy - 10, cx - 1, cy - 11, SSD1306_WHITE);

      display.drawLine(cx + 3, cy - 3, cx + 3, cy - 8, SSD1306_WHITE);
      display.drawLine(cx + 3, cy - 8, cx + 2, cy - 9, SSD1306_WHITE);
      display.drawLine(cx + 2, cy - 10, cx, cy - 11, SSD1306_WHITE);

      // Inner upper arc
      display.drawLine(cx - 4, cy - 3, cx - 3, cy - 5, SSD1306_WHITE);
      display.drawLine(cx - 3, cy - 5, cx, cy - 6, SSD1306_WHITE);
      display.drawLine(cx, cy - 6, cx + 3, cy - 5, SSD1306_WHITE);
      display.drawLine(cx + 3, cy - 5, cx + 4, cy - 3, SSD1306_WHITE);

      // Lower-left outer lobe
      display.drawLine(cx - 7, cy + 1, cx - 10, cy + 2, SSD1306_WHITE);
      display.drawLine(cx - 10, cy + 3, cx - 11, cy + 4, SSD1306_WHITE);
      display.drawLine(cx - 10, cy + 7, cx - 8, cy + 8, SSD1306_WHITE);
      display.drawLine(cx - 8, cy + 8, cx - 3, cy + 7, SSD1306_WHITE);

      // Lower-right outer lobe 
      display.drawLine(cx + 7, cy + 1, cx + 10, cy + 2, SSD1306_WHITE);
      display.drawLine(cx + 10, cy + 3, cx + 11, cy + 4, SSD1306_WHITE);
      display.drawLine(cx + 10, cy + 7, cx + 8, cy + 8, SSD1306_WHITE);
      display.drawLine(cx + 8, cy + 8, cx + 3, cy + 7, SSD1306_WHITE);

      // Inner lower arcs 
      display.drawLine(cx - 6, cy, cx - 6, cy + 3, SSD1306_WHITE);
      display.drawLine(cx - 6, cy + 3, cx - 4, cy + 6, SSD1306_WHITE);
      display.drawLine(cx - 4, cy + 6, cx - 2, cy + 7, SSD1306_WHITE);

      display.drawLine(cx + 6, cy, cx + 6, cy + 3, SSD1306_WHITE);
      display.drawLine(cx + 6, cy + 3, cx + 4, cy + 6, SSD1306_WHITE);
      display.drawLine(cx + 4, cy + 6, cx + 2, cy + 7, SSD1306_WHITE);

      // Center-dot
      display.drawPixel(cx, cy, SSD1306_WHITE);
      display.drawPixel(cx, cy + 2, SSD1306_WHITE);
      display.drawPixel(cx - 1, cy + 1, SSD1306_WHITE);
      display.drawPixel(cx + 1, cy + 1, SSD1306_WHITE);

      y_offset = 1;
      break;
    }
  }
  return y_offset;
}

void drawHitMarker(int y_offset) {
  const unsigned long HIT_MARKER_MS = 120;
  if (lastVerifiedShotTime == 0 ||
      millis() - lastVerifiedShotTime >= HIT_MARKER_MS) {
    return;
  }
  // Center position follows the selected reticle.
  const int cx = RETICLE_X;
  const int cy = RETICLE_Y + y_offset;
  // Sets length of dashes for hit marker.
  const int d_s = 7;          // dash start
  const int d_e = 11;         // dash end
  // Four diagonal strokes with a gap at the center.
  display.drawLine(cx - d_e, cy - d_e, cx - d_s, cy - d_s, SSD1306_WHITE);
  display.drawLine(cx + d_e, cy - d_e, cx + d_s, cy - d_s, SSD1306_WHITE);
  display.drawLine(cx - d_e, cy + d_e, cx - d_s, cy + d_s, SSD1306_WHITE);
  display.drawLine(cx + d_e, cy + d_e, cx + d_s, cy + d_s, SSD1306_WHITE);
}

void drawBatteryWidget(int x, int y, int pct, float voltage) {
  pct = constrain(pct, 0, 100);
  const int w = 24;
  const int h = 10;
  const int nubW = 2;
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  // Percent above the icon. Center-ish over the battery body.
  display.setCursor(x + 3, y - 10);
  display.print(pct);
  display.print(F("%"));
  // Battery outline and nub.
  display.drawRect(x, y, w, h, SSD1306_WHITE);
  display.fillRect(x + w, y + 3, nubW, 4, SSD1306_WHITE);
  // Fill area.
  int fillW = map(pct, 0, 100, 0, w - 4);
  if (fillW > 0) {
    display.fillRect(x + 2, y + 2, fillW, h - 4, SSD1306_WHITE);
  }
  // Voltage to the left of the battery icon.
  // This keeps the battery cluster in the bottom-right, opposite RPM.
  display.setCursor(x - 31, y + 2);
  display.print(voltage, 1);
  display.print(F("V"));
}

void drawOLED(float rpmValue, int battPct, float battVoltage) {
  display.clearDisplay();
  if (batteryWakeMode) {
    int y_offset = drawBaseReticle();
    drawHitMarker(y_offset);
  }
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  if (!batteryWakeMode) {
    // Clean programming/charging screen. The main battery board is OFF.
    display.setCursor(32, 12);
    display.print(F("USB SAFE MODE"));
    display.setCursor(17, 28);
    display.print(F("Charging / Program"));
    display.setCursor(29, 42);
    display.print(F("Main power OFF"));
    // Battery divider is pre-PMOS, so battery telemetry remains available.
    display.setCursor(36, 54);
    display.print(battVoltage, 2);
    display.print(F("V  "));
    display.print(battPct);
    display.print(F("%"));
  } else {
    // CrunchLabs icon replaces the old ARMED / IDLE text.
    // Still while idle, rotating while the flywheels are armed.
    drawCrunchLabsStateIcon();
    display.setCursor(SHOT_COUNTER_X, SHOT_COUNTER_Y);
    display.print(F("S:"));
    display.print(lifetimeShots);
  }
  // Bottom status bar is only meaningful in battery mode.
  if (batteryWakeMode) {
    display.fillRect(RPM_METER_X, RPM_METER_Y - 2, SCREEN_WIDTH, 15, SSD1306_BLACK);
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(RPM_METER_X, RPM_METER_Y);
    display.print(F("RPM:"));
    display.print((long)rpmValue);
    drawBatteryWidget(99, 51, battPct, battVoltage);
  }
}

void refreshOLEDIfDue() {
  static unsigned long lastOLEDRefresh = 0;
  unsigned long now = millis();
  if (now - lastOLEDRefresh < OLED_REFRESH_MS) return;
  lastOLEDRefresh = now;
  drawOLED(rpm, cachedBatteryPercent, cachedBatteryVoltage);
  display.display();
}

void i2cBusRecover() {
  // Nano/Uno: A4=SDA, A5=SCL
  pinMode(A4, INPUT_PULLUP);
  pinMode(A5, INPUT_PULLUP);
  delay(1);
  // If a device is holding SDA low, pulse SCL up to 9 times
  for (int i = 0; i < 9; i++) {
    pinMode(A5, OUTPUT);
    digitalWrite(A5, HIGH);
    delayMicroseconds(5);
    pinMode(A5, INPUT_PULLUP);
    delayMicroseconds(5);
  }
}

bool initOLED() {
  i2cBusRecover();
  Wire.begin();
  Wire.setClock(100000);
  Wire.setWireTimeout(3000, true);
  for (int attempt = 0; attempt < 3; attempt++) {
    if (display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
      display.ssd1306_command(SSD1306_SEGREMAP | 0x0);
      display.setRotation(2);   // 180 degrees for your new display
      updateBatteryCache();
      drawOLED(0, cachedBatteryPercent, cachedBatteryVoltage);
      display.display();
      return true;
    }
    delay(100);
  }
  return false;
}

void showHackPackBootAnimation() {
  //switch (BOOT_ANIMATION) {

  int animation = random(1, 4);
  switch (animation){
    case 1:
      showHackPackCurtainAnimation();
      break;
    case 2:
      showHackPackDissolveAnimation();
      break;
    case 3:
      showHackPackCircularAnimation();
      break;
    default:
      showHackPackDissolveAnimation();
      break;
  }
}

void showHackPackCurtainAnimation() {
  const int rowBytes = (HACK_PACK_LOGO_WIDTH + 7) / 8;
  const int centerX = HACK_PACK_LOGO_WIDTH / 2;
  for (int reveal = 0; reveal <= centerX; reveal += 4) {
    display.clearDisplay();
    for (int y = 0; y < HACK_PACK_LOGO_HEIGHT; y++) {
      for (int x = 0; x < HACK_PACK_LOGO_WIDTH; x++) {
        uint8_t logoByte = pgm_read_byte(&HACK_PACK_LOGO[y * rowBytes + x / 8]);
        if (logoByte & (0x80 >> (x & 7))) {
          if (abs(x - centerX) <= reveal) {
            display.drawPixel(HACK_PACK_LOGO_X + x,
                              HACK_PACK_LOGO_Y + y,
                              SSD1306_WHITE);
          }
        }
      }
    }
    display.display();
    delay(10);
  }
  delay(250);
}

void showHackPackDissolveAnimation() {
  const int rowBytes = (HACK_PACK_LOGO_WIDTH + 7) / 8;
  for (uint8_t step = 1; step <= ANIMATION_STEPS; step++) {
    display.clearDisplay();
    for (int y = 0; y < HACK_PACK_LOGO_HEIGHT; y++) {
      for (int x = 0; x < HACK_PACK_LOGO_WIDTH; x++) {
        uint8_t logoByte = pgm_read_byte(&HACK_PACK_LOGO[y * rowBytes + x / 8]);
        if (logoByte & (0x80 >> (x & 7))) {
          uint32_t n = (uint32_t)y * HACK_PACK_LOGO_WIDTH + x;
          n ^= n >> 7;
          n *= 40503UL;
          n ^= n >> 5;
          n *= 7919UL;
          n ^= n >> 8;
          uint8_t pixelStep = (n % ANIMATION_STEPS) + 1;
          if (pixelStep <= step) {
            display.drawPixel(HACK_PACK_LOGO_X + x,
                              HACK_PACK_LOGO_Y + y,
                              SSD1306_WHITE);
          }
        }
      }
    }
    display.display();
    delay(20);
  }
  delay(250);
}

void showHackPackCircularAnimation() {
  const int rowBytes = (HACK_PACK_LOGO_WIDTH + 7) / 8;
  const int centerX = SCREEN_WIDTH / 2;
  const int centerY = SCREEN_HEIGHT / 2;
  // 16 steps across a 180-degree sweep.
  // Because the sweep has two opposite arms,
  // 180 degrees reveals the entire logo.
  const uint8_t sweepSteps = 16;
  for (uint8_t step = 0; step <= sweepSteps; step++) {
    display.clearDisplay();
    for (int y = 0; y < HACK_PACK_LOGO_HEIGHT; y++) {
      for (int x = 0; x < HACK_PACK_LOGO_WIDTH; x++) {
        uint8_t logoByte =
          pgm_read_byte(&HACK_PACK_LOGO[y * rowBytes + x / 8]);
        if (logoByte & (0x80 >> (x & 7))) {
          int screenX = HACK_PACK_LOGO_X + x;
          int screenY = HACK_PACK_LOGO_Y + y;
          int dx = screenX - centerX;
          int dy = screenY - centerY;
          // Convert the pixel position into an approximate angle
          // from 0-16 across 180 degrees.
          //
          // This intentionally avoids atan2() and floating point.
          uint8_t pixelStep;
          int ax = abs(dx);
          int ay = abs(dy);
          if (dx >= 0 && dy >= 0) {
            // Bottom-right quadrant: 0 -> 4
            pixelStep = (ay * 4) / (ax + ay + 1);
          }
          else if (dx < 0 && dy >= 0) {
            // Bottom-left quadrant: 4 -> 8
            pixelStep = 4 + (ax * 4) / (ax + ay + 1);
          }
          else if (dx < 0 && dy < 0) {
            // Top-left quadrant maps to opposite sweep arm
            pixelStep = (ay * 4) / (ax + ay + 1);
          }
          else {
            // Top-right quadrant maps to opposite sweep arm
            pixelStep = 4 + (ax * 4) / (ax + ay + 1);
          }
          // Scale our 0-8 angular approximation to the
          // requested number of sweep frames.
          pixelStep = ((uint16_t)pixelStep * sweepSteps) / 8;
          if (pixelStep <= step) {
            display.drawPixel(
              screenX,
              screenY,
              SSD1306_WHITE
            );
          }
        }
      }
    }
    display.display();
    delay(10);
  }
  delay(250);
}

bool waitForWarningButton() {
  // Require a fresh press; this prevents the battery wake press from skipping a warning.
  while (digitalRead(WAKE_BTN) == HIGH) {
    delay(5);
  }
  unsigned long warningStart = millis();
  while (batteryWakeMode) {
    if (digitalRead(WAKE_BTN) == HIGH) {
      unsigned long pressStart = millis();
      while (digitalRead(WAKE_BTN) == HIGH) {
        if (millis() - pressStart >= POWER_BUTTON_LONG_PRESS_MS) {
          Serial.println(F("POWER: long press during warning - shutting down"));
          shutdownBlaster();
          return false;
        }
        delay(5);
      }
      if (millis() - pressStart >= POWER_BUTTON_DEBOUNCE_MS) {
        return true;
      }
    }
    // Warnings still advance automatically if the button is not pressed.
    if (millis() - warningStart >= WARNING_AUTO_ADVANCE_MS) {
      return true;
    }
    delay(5);
  }
  return false;
}

void showStartupSafetyWarnings() {
  if (!batteryWakeMode) return;
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(24, 5);
  display.print(F("SAFETY WARNING"));
  display.setCursor(10, 23);
  display.print(F("WEAR SAFETY GLASSES"));
  display.setCursor(15, 39);
  display.print(F("DO NOT AIM AT FACE"));
  display.setCursor(44, 51);
  display.print(F("OR EYES"));
  display.display();
  if (!waitForWarningButton()) return;
  if (MOTOR1_SPEED > DEFAULT_MOTOR1_SPEED || MOTOR2_SPEED > DEFAULT_MOTOR2_SPEED) {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 4);
    display.print(F("!MODIFIED SOFTWARE!"));
    display.setCursor(10, 20);
    display.print(F("FLYWHEEL SPEEDS ARE"));
    display.setCursor(10, 32);
    display.print(F("SET ABOVE"));
    display.setCursor(10, 44);
    display.print(F("RECOMMENDED VALUES"));
    display.setCursor(10, 56);
    display.print(F("USE EXTRA CAUTION"));
    display.display();
    if (!waitForWarningButton()) return;
  }
  // Warning presses are consumed here and never reach the normal IDLE/ARMED toggle.
  lastWakeBtnState = LOW;
  powerPressHandled = false;
  powerButtonArmed = true;
}
