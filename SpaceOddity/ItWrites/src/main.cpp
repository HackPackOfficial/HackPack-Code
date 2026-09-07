// =============================================================================
// Hack Pack: Space Oddity
// Hack: Who Taught it to Write? No, Seriously, Guys, WHO TAUGHT IT TO WRITE?!
// =============================================================================
//
// Connect to the Oddity over the USB serial monitor and type a message, followed
// by pressing enter (e.g. "hello" + Enter). 
// 
// Some strange changes to the way it communicates can be effected by typing in
// an octothorpe character, followed by the Enter key.
// =============================================================================

#include <Arduino.h>
#include <configuration.h>      // behavior configuration variables
#include <hardware_map.h>       // hardware variables

#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>       // graphics library
#include <Adafruit_SSD1306.h>   // display management library
#include <FastLED.h>            // pinned at version 3.7.8
#include <ServoWrapper.h>       // useful functions for controlling servos
#include <AgileStateMachine.h>  // a very nice state machine library
#include <PowerLink.h>          // interface with the power/buttons via coprocessor
#include <bootloader_random.h>  // for hardware RNG support
#include <esp_random.h>          // esp_random() for seeding Arduino random()
#include "glyph_points.h"       // normalized 100-point glyphs (GLYPH_TABLE, GLYPH_COUNT, GLYPH_POINT_COUNT)


// =============================================================================
// POWER AND FRONT BUTTONS CONFIGURATION
// =============================================================================
// The main PCB has a CH32V003 microcontroller that acts as a coprocessor.
// It manages the front buttons, the power button, and power management.
// It communicates with the ESP32-C3 over UART.
PowerLink carrierPCB;


// =============================================================================
// MODE DEFINITIONS
// =============================================================================
enum class Mode
{
  IDLE,    // breathing, waiting for a serial message
  WRITE    // tracing the current message one glyph at a time
};


// =============================================================================
// GLOBAL OBJECTS
// =============================================================================
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Offscreen 1-bit canvas used as the processing layer: the clean letter is
// rendered here, then a distortion pass copies it (warped/noised) to the OLED.
GFXcanvas1 glyphCanvas(SCREEN_WIDTH, SCREEN_HEIGHT);

// Currently selected on-screen distortion effect (changeable at runtime via "#").
GlyphFx currentGlyphFx = WRITE_DISPLAY_EFFECT;

// Last time the idle snow frame was regenerated.
uint32_t lastIdleNoise = 0;

CRGB leds[NUM_LEDS];

ServoWrapper leftServo(PIN_SERVO_LEFT, SERVO_LEFT_TRIM),
             rightServo(PIN_SERVO_RIGHT, SERVO_RIGHT_TRIM),
             magnetServo(PIN_SERVO_MAGNET, MAGNET_SERVO_TRIM);


// =============================================================================
// STATE VARIABLES
// =============================================================================
Mode currentMode = Mode::IDLE;

bool newStateEntry = false;   // set by the FSM on state entry

// Serial message handling
String  activeMessage   = "";
String  pendingMessage  = "";
bool    newMessageReady = false;
int32_t writerIdx       = 0;
char    serialBuf[SERIAL_MSG_MAX];
uint16_t serialBufLen   = 0;

// Timing
uint32_t lastPatternUpdate = 0;

// Park tracking (used by the power-button shutdown path)
bool magnetParked = false;


// =============================================================================
// FUNCTION PROTOTYPES
// =============================================================================
void setupStateMachine();
void onIdleEntry();
void onIdleRun();
void writeMessage();

static const float (*findGlyph(char c))[2];
void traceGlyph(const float (*pts)[2], uint16_t n);
void showLetterOnDisplay(char c);
void drawLetterToCanvas(char c);
void presentGlyphFx();
void fxGlyphNone();
void fxGlyphStatic();
void fxGlyphBlockGlitch();
void fxGlyphWave();
void fxGlyphScanline();
void fxGlyphZeta();
const char* glyphFxName(GlyphFx fx);
void cycleGlyphFx();
static void zetaPrecompute();
void showIdlePrompt();
void updateIdleDisplay();
void handleSerialInput();

void calculateRotatedServoTargets(int16_t px, int16_t py);
void updateMagnetPulse();

void attachArms(uint16_t staggerMs = 100);
void attachAll(uint16_t staggerMs = 100);
void servicePowerButton(bool allowShutdown = true);

void pickUpFerrofluid();
void putItInPark();

void drawStartupAnimation();
void updateRainbowCycle();
CRGB colorWheel(uint8_t position);

int32_t randomInRange(int32_t min = INT32_MIN, int32_t max = INT32_MAX);

inline float fmap(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


// =============================================================================
// STATE MACHINE STUFF
// =============================================================================
bool wantIdle()  { return currentMode == Mode::IDLE; }
bool wantWrite() { return currentMode == Mode::WRITE; }

StateMachine fsm;

void setupStateMachine() {
  State* st_Idle  = fsm.addState("Idle",  0, 0, onIdleEntry, nullptr, onIdleRun);
  State* st_Write = fsm.addState("Write", 0, 0, pickUpFerrofluid, nullptr, writeMessage);

  st_Idle->addTransition(st_Write, wantWrite);
  st_Write->addTransition(st_Idle, wantIdle);

  st_Idle->addAction(Action::Type::RE, newStateEntry);
  st_Write->addAction(Action::Type::RE, newStateEntry);

  fsm.setInitialState(st_Idle);
  fsm.start();
}


// =============================================================================
// SETUP
// =============================================================================
void setup() {
  // hardware random number generator (RNG) initialization.
  // NOTE: THIS HAS TO BE DISABLED IF YOU WANT TO USE WIFI OR BLUETOOTH
  bootloader_random_enable();
  randomSeed(esp_random());

  SERIAL_BEGIN(115200);
  delay(100);
  SERIAL_PRINTLN("begin");
  carrierPCB.begin();
  SERIAL_PRINTLN("UART established");

  Wire.setPins(21, 20);
  display.begin(SSD1306_SWITCHCAPVCC, SCREEN_I2C_ADDR);
  display.clearDisplay();
  drawStartupAnimation();
  delay(500);
  zetaPrecompute();   // build the zeta warp field once at boot (avoids a hitch on first use)

  FastLED.addLeds<WS2812B, PIN_LEDS, GRB>(leds, NUM_LEDS);
  FastLED.setBrightness(LED_BRIGHTNESS);

  // Assume the arms are physically at the park pose on power-up, so attaching
  // holds that pose and causes no movement (staggered to flatten inrush current)
  leftServo.syncCurrentPos(180.0);
  rightServo.syncCurrentPos(0.0);
  magnetServo.syncCurrentPos(MAGNET_PARK_POS);
  leftServo.attach();
  delay(250);
  rightServo.attach();
  delay(250);
  magnetServo.attach();
  delay(100);

  leftServo.targetPos = 180.0;
  rightServo.targetPos = 0.0;
  leftServo.write(leftServo.targetPos);
  rightServo.write(rightServo.targetPos);

  setupStateMachine();
  delay(500);
  SERIAL_PRINTLN("ready - type a message and press enter");
}


// =============================================================================
// MAIN LOOP
// =============================================================================
void loop() {
  static uint32_t powerButtonPressedTime = 0;

  carrierPCB.update();      // power + front button states from the coprocessor

  if (carrierPCB.stateChanged()) {
    if (carrierPCB.buttonPressed(PowerLink::BTN_POWER)) powerButtonPressedTime = millis();
    if (carrierPCB.buttonReleased(PowerLink::BTN_POWER)) powerButtonPressedTime = 0;
  }

  // long-press power button: park the magnet and shut down
  if (powerButtonPressedTime != 0 && millis() - powerButtonPressedTime >= SOFT_SHUTDOWN_TIME) {
    if (!magnetParked) putItInPark();
    delay(500);
    carrierPCB.shutdown();
  }

  // read typed messages from the serial monitor
  handleSerialInput();
  if (newMessageReady && currentMode == Mode::IDLE) {
    activeMessage = pendingMessage;
    pendingMessage = "";
    newMessageReady = false;
    writerIdx = 0;
    currentMode = Mode::WRITE;
  }

  // LED pattern (rainbow cycle)
  if (millis() - lastPatternUpdate > PATTERN_INTERVAL_MS) {
    updateRainbowCycle();
    lastPatternUpdate = millis();
  }

  fsm.execute();
}


// =============================================================================
// SERIAL INPUT
// =============================================================================
void handleSerialInput() {
  while (Serial.available()) {
    char c = (char)Serial.read();

    if (c == '\n' || c == '\r') {
      if (serialBufLen > 0) {
        serialBuf[serialBufLen] = 0;
        // special command: '#' cycles the on-screen distortion effect live
        if (serialBufLen == 1 && serialBuf[0] == '#') {
          cycleGlyphFx();
          serialBufLen = 0;
        } else {
          pendingMessage = String(serialBuf);
          serialBufLen = 0;
          SERIAL_PRINT("> ");
          SERIAL_PRINTLN(pendingMessage);
          newMessageReady = true;
        }
      }
      // ignore empty lines
    } else if (c == 8 || c == 127) {       // backspace / delete
      if (serialBufLen > 0) serialBufLen--;
    } else if (serialBufLen < SERIAL_MSG_MAX - 1) {
      if (c >= 'a' && c <= 'z') c = (char)(c - ('a' - 'A'));  // glyphs are uppercase
      serialBuf[serialBufLen++] = c;
    }
  }
}


// =============================================================================
// IDLE STATE
// =============================================================================
void onIdleEntry() {
  showIdlePrompt();
  // release the arm servos; the magnet stays attached for breathing
  leftServo.detach();
  rightServo.detach();
  magnetParked = false;
}

void onIdleRun() {
  updateMagnetPulse();
  if (millis() - lastIdleNoise >= IDLE_NOISE_INTERVAL_MS) {
    lastIdleNoise = millis();
    updateIdleDisplay();
  }
}


// =============================================================================
// WRITE STATE
// =============================================================================
void writeMessage() {
  // finished the current message?
  if (writerIdx >= (int32_t)activeMessage.length()) {
    if (newMessageReady) {
      // chain straight into the next buffered message
      activeMessage = pendingMessage;
      pendingMessage = "";
      newMessageReady = false;
      writerIdx = 0;
    } else {
      currentMode = Mode::IDLE;
      return;
    }
  }

  char c = activeMessage[writerIdx];
  const float (*pts)[2] = findGlyph(c);   // nullptr -> space/unknown: just pause

  showLetterOnDisplay(c);   // one letter at a time on the OLED
  traceGlyph(pts, GLYPH_POINT_COUNT);

  writerIdx++;
}


// Return the 100-point polyline for a character, or nullptr for space/unknown.
// Glyph keys are uppercase, so match case-insensitively.
static const float (*findGlyph(char c))[2] {
  if (c >= 'a' && c <= 'z') c = (char)(c - ('a' - 'A'));
  for (uint8_t i = 0; i < GLYPH_COUNT; i++) {
    if (GLYPH_TABLE[i].character == c) return GLYPH_TABLE[i].points;
  }
  return nullptr;
}


// Trace one glyph: scale so the farthest point sits at WRITE_RADIUS (keeps it
// inside the reachable circle), then walk the 100 normalized points at a fixed
// pace. The connecting move between letters happens naturally as the magnet
// eases from the previous glyph's last point toward this glyph's first point.
void traceGlyph(const float (*pts)[2], uint16_t n) {
  if (pts == nullptr || n < 2) {   // space / unknown: just pause
    delay(WRITE_LETTER_PAUSE);
    return;
  }

  // scale so the farthest point sits exactly at WRITE_RADIUS
  float maxR = 0.0f;
  for (uint16_t i = 0; i < n; i++) {
    float r = sqrtf(pts[i][0] * pts[i][0] + pts[i][1] * pts[i][1]);
    if (r > maxR) maxR = r;
  }
  float scale = (maxR > 0.0f) ? (float)WRITE_RADIUS / maxR : 1.0f;

  leftServo.setSmoothing(WRITE_SMOOTHING);
  rightServo.setSmoothing(WRITE_SMOOTHING);

  for (uint16_t i = 0; i < n; i++) {
    int16_t ix = (int16_t)lroundf(pts[i][0] * scale);
    int16_t iy = (int16_t)lroundf(pts[i][1] * scale);

    // jar Y points down (flip glyph Y-up); X is mirrored by the linkage, so
    // also negate X to keep letters readable (not backwards)
    calculateRotatedServoTargets(JOY_CENTER - ix, JOY_CENTER - iy);

    uint32_t stepStart = millis();
    while (millis() - stepStart < WRITE_POINT_INTERVAL_MS) {
      servicePowerButton();
      leftServo.moveTo(leftServo.targetPos);
      rightServo.moveTo(rightServo.targetPos);
    }
  }

  delay(WRITE_LETTER_PAUSE);
}


// =============================================================================
// DISPLAY FUNCTIONS
// =============================================================================
// Render a clean (crisp) letter into the offscreen canvas, centered.
void drawLetterToCanvas(char c) {
  glyphCanvas.fillScreen(0);
  glyphCanvas.setTextSize(WRITE_DISPLAY_SIZE);
  glyphCanvas.setTextColor(1);
  glyphCanvas.setTextWrap(false);

  char s[2] = { c, '\0' };
  int16_t x1, y1;
  uint16_t w, h;
  glyphCanvas.getTextBounds(s, 0, 0, &x1, &y1, &w, &h);
  int16_t cx = (SCREEN_WIDTH  - (int16_t)w) / 2 - x1;
  int16_t cy = (SCREEN_HEIGHT - (int16_t)h) / 2 - y1;
  glyphCanvas.setCursor(cx, cy);
  glyphCanvas.print(c);
}


// ---- Distortion passes (the processing layer) -------------------------------
// Each reads the clean letter from glyphCanvas and writes a mangled version to
// the OLED. Add new algorithms here, then list them in the GlyphFx enum.

void fxGlyphNone() {
  display.clearDisplay();
  display.drawBitmap(0, 0, glyphCanvas.getBuffer(), SCREEN_WIDTH, SCREEN_HEIGHT, SSD1306_WHITE);
  display.display();
}

// TV-snow: jitter each letter pixel, randomly drop some, and sprinkle noise.
void fxGlyphStatic() {
  display.clearDisplay();
  for (int16_t y = 0; y < SCREEN_HEIGHT; y++) {
    for (int16_t x = 0; x < SCREEN_WIDTH; x++) {
      if (glyphCanvas.getPixel(x, y)) {
        int16_t jx = x + (int16_t)random(-WRITE_DISPLAY_STATIC_JIT, WRITE_DISPLAY_STATIC_JIT + 1);
        int16_t jy = y + (int16_t)random(-WRITE_DISPLAY_STATIC_JIT, WRITE_DISPLAY_STATIC_JIT + 1);
        if (random(100) < WRITE_DISPLAY_STATIC_KEEP &&
            jx >= 0 && jx < SCREEN_WIDTH && jy >= 0 && jy < SCREEN_HEIGHT) {
          display.drawPixel(jx, jy, SSD1306_WHITE);
        }
      }
    }
  }
  for (uint16_t i = 0; i < WRITE_DISPLAY_STATIC_NOISE; i++) {
    display.drawPixel((int16_t)random(SCREEN_WIDTH), (int16_t)random(SCREEN_HEIGHT), SSD1306_WHITE);
  }
  display.display();
}

// Slice the letter into horizontal bands and shift each band sideways, with the
// occasional band replaced by static.
void fxGlyphBlockGlitch() {
  display.clearDisplay();
  const int16_t bandH = SCREEN_HEIGHT / WRITE_DISPLAY_BANDS;
  for (uint8_t b = 0; b < WRITE_DISPLAY_BANDS; b++) {
    int16_t shift = (int16_t)random(-6, 7);
    int16_t y0 = b * bandH;
    int16_t y1b = (b + 1) * bandH;
    for (int16_t y = y0; y < y1b; y++) {
      for (int16_t x = 0; x < SCREEN_WIDTH; x++) {
        if (glyphCanvas.getPixel(x, y)) {
          int16_t nx = x + shift;
          if (nx >= 0 && nx < SCREEN_WIDTH) display.drawPixel(nx, y, SSD1306_WHITE);
        }
      }
    }
    if (random(100) < 20) {   // noise band
      for (int16_t y = y0; y < y1b; y++)
        for (int16_t x = 0; x < SCREEN_WIDTH; x++)
          if (random(100) < 18) display.drawPixel(x, y, SSD1306_WHITE);
    }
  }
  display.display();
}

// Horizontal sine displacement (wavy letter).
void fxGlyphWave() {
  display.clearDisplay();
  const float amp = 4.0f, freq = 0.30f;
  for (int16_t y = 0; y < SCREEN_HEIGHT; y++) {
    int16_t dx = (int16_t)(amp * sinf((float)y * freq));
    for (int16_t x = 0; x < SCREEN_WIDTH; x++) {
      if (glyphCanvas.getPixel(x, y)) {
        int16_t nx = x + dx;
        if (nx >= 0 && nx < SCREEN_WIDTH) display.drawPixel(nx, y, SSD1306_WHITE);
      }
    }
  }
  display.display();
}

// Pixel sort: reverse alternating runs of consecutive lit pixels within each
// row. The result is a glitchy, "sorted" scramble of the letter.
// Scanline dropout: randomly drop whole rows to black or replace them with
// noise, leaving the rest of the letter intact.
void fxGlyphScanline() {
  display.clearDisplay();
  for (int16_t y = 0; y < SCREEN_HEIGHT; y++) {
    uint8_t mode = (uint8_t)random(100);
    if (mode < WRITE_DISPLAY_SCAN_DROP) continue;                       // dropped
    if (mode < WRITE_DISPLAY_SCAN_DROP + WRITE_DISPLAY_SCAN_NOISE) {    // noise row
      for (int16_t x = 0; x < SCREEN_WIDTH; x++)
        if (random(100) < 50) display.drawPixel(x, y, SSD1306_WHITE);
      continue;
    }
    for (int16_t x = 0; x < SCREEN_WIDTH; x++)                          // intact row
      if (glyphCanvas.getPixel(x, y)) display.drawPixel(x, y, SSD1306_WHITE);
  }
  display.display();
}

// -----------------------------------------------------------------------------
// Riemann-zeta warp: a deterministic, structured displacement field generated
// from fractional parts of the real zeta function evaluated at per-row / per-
// column parameters. Cheap (computed once) and gives an organic, "weird" warp.
// -----------------------------------------------------------------------------
static float zetaReal(float s) {
  if (s <= 1.0001f) s = 1.0001f;     // stay right of the pole
  if (s > 4.0f)     s = 4.0f;
  float sum = 0.0f;
  for (int n = 1; n <= 250; n++) sum += 1.0f / powf((float)n, s);
  return sum;
}
static float fractf(float v) { return v - floorf(v); }

static int16_t  zetaDX[SCREEN_HEIGHT];
static int16_t  zetaDY[SCREEN_WIDTH];
static bool     zetaInit = false;
static void zetaPrecompute() {
  if (zetaInit) return;
  zetaInit = true;
  // Oscillate the zeta argument so the fractional part swings across rows/cols
  // (a plain linear sweep collapses to a near-constant shift). The result is a
  // deterministic, structured-but-erratic warp field.
  for (int16_t y = 0; y < SCREEN_HEIGHT; y++) {
    float s = 2.5f + 0.7f * sinf(y * 0.35f);
    float v = fractf(zetaReal(s));
    zetaDX[y] = (int16_t)roundf((v - 0.5f) * 2.0f * WRITE_DISPLAY_ZETA_AMP);
  }
  for (int16_t x = 0; x < SCREEN_WIDTH; x++) {
    float s = 2.5f + 0.7f * cosf(x * 0.21f);
    float v = fractf(zetaReal(s));
    zetaDY[x] = (int16_t)roundf((v - 0.5f) * 2.0f * WRITE_DISPLAY_ZETA_AMP);
  }
}
// Riemann-zeta warp combined with the static (TV-snow) noise. The warp gives a
// structured, deterministic distortion; the jitter + dropout + snow make it hard
// to read. All static tunables (WRITE_DISPLAY_STATIC_*) apply here too.
void fxGlyphZeta() {
  zetaPrecompute();
  display.clearDisplay();
  for (int16_t y = 0; y < SCREEN_HEIGHT; y++) {
    for (int16_t x = 0; x < SCREEN_WIDTH; x++) {
      if (glyphCanvas.getPixel(x, y)) {
        int16_t jx = x + (int16_t)random(-WRITE_DISPLAY_STATIC_JIT, WRITE_DISPLAY_STATIC_JIT + 1);
        int16_t jy = y + (int16_t)random(-WRITE_DISPLAY_STATIC_JIT, WRITE_DISPLAY_STATIC_JIT + 1);
        int16_t nx = jx + zetaDX[y];
        int16_t ny = jy + zetaDY[x];
        if (random(100) < WRITE_DISPLAY_STATIC_KEEP &&
            nx >= 0 && nx < SCREEN_WIDTH && ny >= 0 && ny < SCREEN_HEIGHT)
          display.drawPixel(nx, ny, SSD1306_WHITE);
      }
    }
  }
  for (uint16_t i = 0; i < WRITE_DISPLAY_STATIC_NOISE; i++) {
    display.drawPixel((int16_t)random(SCREEN_WIDTH), (int16_t)random(SCREEN_HEIGHT), SSD1306_WHITE);
  }
  display.display();
}

const char* glyphFxName(GlyphFx fx) {
  switch (fx) {
    case GlyphFx::NONE:         return "NONE";
    case GlyphFx::STATIC:       return "STATIC";
    case GlyphFx::BLOCK_GLITCH: return "BLOCK_GLITCH";
    case GlyphFx::WAVE:         return "WAVE";
    case GlyphFx::SCANLINE:     return "SCANLINE";
    case GlyphFx::ZETA:         return "ZETA";
    default:                    return "?";
  }
}

// Run the currently selected distortion pass on whatever is in glyphCanvas.
void presentGlyphFx() {
  switch (currentGlyphFx) {
    case GlyphFx::NONE:         fxGlyphNone();        break;
    case GlyphFx::STATIC:       fxGlyphStatic();      break;
    case GlyphFx::BLOCK_GLITCH: fxGlyphBlockGlitch(); break;
    case GlyphFx::WAVE:         fxGlyphWave();        break;
    case GlyphFx::SCANLINE:     fxGlyphScanline();    break;
    case GlyphFx::ZETA:         fxGlyphZeta();        break;
    default:                    fxGlyphNone();        break;
  }
}

void cycleGlyphFx() {
  uint8_t next = (static_cast<uint8_t>(currentGlyphFx) + 1) % static_cast<uint8_t>(GlyphFx::COUNT);
  currentGlyphFx = static_cast<GlyphFx>(next);
  SERIAL_PRINT("display effect -> ");
  SERIAL_PRINTLN(glyphFxName(currentGlyphFx));
}

// Show the current letter: render it cleanly, then push it through the
// selected distortion pass.
void showLetterOnDisplay(char c) {
  drawLetterToCanvas(c);
  presentGlyphFx();
}

void showIdlePrompt() {
  // Seed the idle CRT-snow animation: fill the display buffer with random
  // noise so the snow appears immediately (updateIdleDisplay evolves it).
  uint8_t* buf = display.getBuffer();
  const uint16_t N = (uint16_t)(SCREEN_WIDTH * SCREEN_HEIGHT) / 8;
  for (uint16_t i = 0; i < N; i++) buf[i] = (uint8_t)random(256);
  display.display();
}

// Evolve one frame of TV-snow: stochastically flip each pixel in the existing
// display buffer. No extra per-pixel memory is required; the buffer itself is
// the state, and the on/off probabilities set the steady density + pixel lifetime.
void updateIdleDisplay() {
  uint8_t* buf = display.getBuffer();
  const uint16_t N = (uint16_t)(SCREEN_WIDTH * SCREEN_HEIGHT) / 8;
  for (uint16_t i = 0; i < N; i++) {
    uint8_t b = buf[i];
    uint8_t nb = 0;
    for (uint8_t bit = 0; bit < 8; bit++) {
      bool on = b & (1u << bit);
      if (on) {
        if (random(100) >= IDLE_NOISE_OFF) nb |= (1u << bit);
      } else {
        if (random(100) < IDLE_NOISE_ON)  nb |= (1u << bit);
      }
    }
    buf[i] = nb;
  }
  display.display();
}

void drawStartupAnimation() {
  display.clearDisplay();
  int maxRadius = max(display.width(), display.height()) / 2;
  for (int16_t r = maxRadius; r > 0; r -= 3) {
    display.fillCircle(display.width() / 2, display.height() / 2, r, SSD1306_INVERSE);
    display.display();
  }
}


// =============================================================================
// SERVO CONTROL
// =============================================================================
// Approximate inverse kinematics for the 5R parallel arm: a 45-degree rotation
// turns XY coordinates into the two servo angles.
void calculateRotatedServoTargets(int16_t px, int16_t py) {
  constexpr float COS_45 = 0.70710678f;

  int16_t x = px - (JOY_CENTER + X_COORD_TRIM);
  int16_t y = py - (JOY_CENTER + Y_COORD_TRIM);

  float rotatedX = static_cast<float>(x - y) * COS_45;
  float rotatedY = static_cast<float>(x + y) * -COS_45;

  int16_t servo1Value = constrain(static_cast<float>(rotatedX + JOY_CENTER), 0, JOY_MAX);
  int16_t servo2Value = constrain(static_cast<float>(rotatedY + JOY_CENTER), 0, JOY_MAX);

  leftServo.targetPos  = fmap(servo1Value, 0, JOY_MAX, SERVO_MIN, SERVO_MAX);
  rightServo.targetPos = fmap(servo2Value, 0, JOY_MAX, SERVO_MAX, SERVO_MIN);
}

// Move the magnet up and down a small amount when idle ("breathing").
void updateMagnetPulse() {
  static uint32_t lastUpdate = 0;
  static int8_t magnetStep = 3;
  if (millis() - lastUpdate < MAGNET_SERVO_UPDATE_INTERVAL) return;
  if (magnetServo.targetPos >= MAGNET_MAX) {
    magnetStep = -3;
  } else if (magnetServo.targetPos <= MAGNET_MIN) {
    magnetStep = 3;
  }
  magnetServo.targetPos = constrain(magnetServo.targetPos + (float)magnetStep, MAGNET_MIN, MAGNET_MAX);
  magnetServo.setSmoothing(0.9);
  magnetServo.moveTo(magnetServo.targetPos);
  lastUpdate = millis();
}


// =============================================================================
// SERVO HELPERS
// =============================================================================
void attachArms(uint16_t staggerMs) {
  leftServo.attach();  delay(staggerMs);
  rightServo.attach(); delay(staggerMs);
}
void attachAll(uint16_t staggerMs) {
  leftServo.attach();  delay(staggerMs);
  rightServo.attach(); delay(staggerMs);
  magnetServo.attach(); delay(staggerMs);
}

// Keep the power button responsive during long blocking moves.
void servicePowerButton(bool allowShutdown) {
  static uint32_t pressedSince = 0;
  carrierPCB.update();
  if (carrierPCB.isPressed(PowerLink::BTN_POWER)) {
    if (pressedSince == 0) pressedSince = millis();
    else if (allowShutdown && millis() - pressedSince >= SOFT_SHUTDOWN_TIME) carrierPCB.shutdown();
  } else {
    pressedSince = 0;
  }
}


// =============================================================================
// CONSTANT-SPEED STEPPED MOVE (entry/exit moves only)
// =============================================================================
struct StepMove {
  ServoWrapper *servo;
  float        targetAngle;
  uint32_t     startAt;
};

static void steppedMove(StepMove moves[], uint8_t count, bool allowShutdown, float degPerSec = SLOW_MOVE_DPS) {
  uint32_t moveStart = millis();
  uint32_t lastStep = millis();
  bool moving = true;
  while (moving && millis() - moveStart < BLOCKING_MOVE_TIMEOUT_MS) {
    servicePowerButton(allowShutdown);
    uint32_t now = millis();
    if (now - lastStep < SLOW_STEP_INTERVAL_MS) { delay(1); continue; }
    lastStep = now;

    float stepDeg = degPerSec * SLOW_STEP_INTERVAL_MS / 1000.0f;
    if (stepDeg < 0.05f) stepDeg = 0.05f;

    moving = false;
    for (uint8_t i = 0; i < count; i++) {
      StepMove &m = moves[i];
      float cur = m.servo->getCurrentPos();
      float diff = m.targetAngle - cur;
      if (fabsf(diff) <= 0.01f) continue;
      moving = true;
      if ((int32_t)(now - m.startAt) < 0) continue;
      m.servo->write((fabsf(diff) <= stepDeg) ? m.targetAngle
                                              : cur + ((diff > 0) ? stepDeg : -stepDeg));
    }
  }
}


// Park the magnet away from the ferrofluid before powering off.
void putItInPark() {
  SERIAL_PRINTLN("parking");

  attachAll();

  StepMove phase1[] = {
    { &leftServo,  0.0f,   0 },
    { &rightServo, 180.0f, millis() + SLOW_STAGGER_MS },
  };
  steppedMove(phase1, 2, false, SLOW_MOVE_DPS * 2.0f);
  delay(100);

  StepMove phase2[] = {
    { &magnetServo, (float)MAGNET_PARK_POS, 0 },
  };
  steppedMove(phase2, 1, false);
  delay(20);

  leftServo.write(180.0f);
  delay(100);
  rightServo.write(0.0f);
  delay(500);

  leftServo.detach(); rightServo.detach(); magnetServo.detach();
  leftServo.targetPos = leftServo.getCurrentPos();
  rightServo.targetPos = rightServo.getCurrentPos();
  magnetServo.targetPos = magnetServo.getCurrentPos();

  magnetParked = true;
  currentMode = Mode::IDLE;
}


// Move to the bottom center of the jar and lower the magnet to grab the fluid.
void pickUpFerrofluid() {
  magnetServo.targetPos = MAGNET_MAX;
  calculateRotatedServoTargets(JOY_CENTER, JOY_CENTER + 500);

  attachAll(150);

  StepMove moves[] = {
    { &leftServo,   leftServo.targetPos,  0 },
    { &rightServo,  rightServo.targetPos, millis() + SLOW_STAGGER_MS },
    { &magnetServo, (float)MAGNET_MAX,    millis() + 2 * SLOW_STAGGER_MS },
  };
  steppedMove(moves, 3, true);
  delay(250);

  leftServo.targetPos = leftServo.getCurrentPos();
  rightServo.targetPos = rightServo.getCurrentPos();
  magnetServo.targetPos = magnetServo.getCurrentPos();
}


// =============================================================================
// LED FUNCTIONS
// =============================================================================
void updateRainbowCycle() {
  static uint16_t wheelOffset = 0;

  for (int i = 0; i < NUM_LEDS; i++) {
    leds[i] = colorWheel(((i * 256 / 32) + wheelOffset) & 255);
  }
  FastLED.show();

  wheelOffset++;
  if (wheelOffset >= 256 * 5) wheelOffset = 0;
}

CRGB colorWheel(uint8_t position) {
  position = 255 - position;
  if (position < 85) {
    return CRGB(255 - position * 3, 0, position * 3);
  }
  if (position < 170) {
    position -= 85;
    return CRGB(0, position * 3, 255 - position * 3);
  }
  position -= 170;
  return CRGB(position * 3, 255 - position * 3, 0);
}


// =============================================================================
// UTILITY
// =============================================================================
int32_t randomInRange(int32_t min, int32_t max) {
  return min + (esp_random() % (max - min + 1));
}
