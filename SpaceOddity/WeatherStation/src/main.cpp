// =============================================================================
// Hack Pack: Space Oddity - Weather Indicator
// =============================================================================
// This firmware turns the Space Oddity ferrofluid creature into an abstract
// weather forecast indicator. It connects to WiFi (credentials supplied by the
// end user through a captive portal, never stored in source), fetches the local
// forecast from wttr.in, and drives the ferrofluid and RGB LEDs from that data:
//   - hot / sunny  -> redder light, faster swim, faster magnet pulse
//   - cold / rainy -> bluer light, slower swim, slower pulse, magnet dwells low
// The behavior lives in a single WEATHER mode implemented as a finite state
// machine state so it can be dropped back into other builds alongside SWIM,
// DRAW, DANCE, etc.
// =============================================================================

// =============================================================================
// WiFi Setup Instructions:
//
// The device connects to your home WiFi via a captive-portal (WiFiManager).
// Credentials are stored in NVS (non-volatile flash) -- they are NEVER in source
// and they survive reboots, so you only enter them once.
//
// First boot (or after the saved network is forgotten/erased):
//   1. Power on the device. It cannot find saved WiFi, so it opens a config
//      portal and starts its own access point named "SpaceOddity-Setup".
//   2. On your phone or laptop, open WiFi settings and join that network.
//      Passphrase: "configureme"
//   3. A setup page should pop up automatically (captive portal). If it does
//      not, open a browser and go to http://192.168.4.1
//   4. In the portal, choose your home WiFi network (SSID), type its password,
//      and tap Save / Connect.
//   5. The device reboots, connects to your WiFi, and starts fetching weather.
//
// Notes:
//   - You have 300 seconds (5 min) in the portal. If it times out with no
//     config, the device runs OFFLINE with neutral (calm) behavior and retries
//     on the next reboot.
//   - To reconfigure or join a different network, clear the saved credentials
//     (erase NVS / factory-reset the board) -- there is no in-app reset button
//     yet, so a fresh flash or `esptool erase_flash` reopens the portal.
// =============================================================================

#include <Arduino.h>
#include <cctype>
#include <configuration.h>      // behavior configuration variables
#include <hardware_map.h>       // hardware variables

#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>       // graphics library
#include <Adafruit_SSD1306.h>   // display management library
#include <ezButton.h>           // button debouncing
#include <FastLED.h>  // pinned at version 3.7.8; newer versions don't work well with ESP32-C3 toolchain yet
#include <ServoWrapper.h>       // some useful functions for controlling servos
#include <AgileStateMachine.h>  // a very nice state machine library
#include <PowerLink.h>          // interface with the power and front panel buttons via the coprocessor

#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <WiFiManager.h>        // captive-portal WiFi provisioning (credentials stored in NVS, not in source)


// =============================================================================
// POWER AND FRONT BUTTONS CONFIGURATION
// =============================================================================
PowerLink carrierPCB;


// =============================================================================
// MODE DEFINITION
// =============================================================================
// The robot is a single weather-driven mode. IDLE/PARK exist only so the
// power-down path (putItInPark) has a valid state to transition into.
enum class Mode
{
  IDLE,
  PARK,
  WEATHER
};


// =============================================================================
// WEATHER DATA
// =============================================================================
enum class Condition
{
  SUNNY,
  CLOUDY,
  PRECIP,   // rain / snow / storm
  FOG,      // fog / mist / haze
  UNKNOWN
};

struct WeatherSample
{
  int8_t    tempC;
  Condition cond;
  bool      valid;
};

WeatherSample weather = { 20, Condition::UNKNOWN, false };

// Derived parameters, refreshed whenever a forecast arrives.
float    gWarmth        = 0.5f;   // 0 = cold, 1 = hot
uint8_t  gHue           = 140;    // FastLED HSV hue (0 red .. 160 blue)
float    gSwimSpeed     = 1.8f;
uint16_t gMagnetPulseMs = 200;
int16_t  gVertBias      = 0;      // shifts the swim center down (toward jar bottom) when cold/wet
float    gServoSmooth   = 0.999f; // arm-servo tracking speed (weather-driven)
uint32_t lastWeatherFetch = 0;


// =============================================================================
// GLOBAL OBJECTS
// =============================================================================
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

CRGB leds[NUM_LEDS];

ezButton joystickButton(PIN_JOYSTICK_BUTTON);


// =============================================================================
// STATE VARIABLES
// =============================================================================
int16_t joyXval, joyYval;
int8_t  joyXAxis = 0, joyYAxis = 0;

int16_t eyeScreenX, eyeScreenY;

// Pond-ripple OLED effect state
struct Ripple { int16_t x, y; uint32_t startMs; };
static Ripple ripples[RIPPLE_MAX];
static uint8_t rippleCount = 0;
static uint32_t lastRippleSpawn = 0;
static uint8_t rippleBuf[SCREEN_WIDTH * SCREEN_HEIGHT];  // grayscale intensity for dither
static uint32_t lastRippleDraw = 0;

Mode currentMode = Mode::IDLE;

// State machine stuff
bool newStateEntry = false;
bool performNewEntryTimedPhase = false;

// Swim algorithm state
int16_t swimX = JOY_CENTER;
int16_t swimY = JOY_CENTER;
int16_t swimTargetX = JOY_CENTER;
int16_t swimTargetY = JOY_CENTER;
float   swimHeading;
bool    getAngry = false;

uint32_t lastPatternUpdate = 0;
uint32_t lastServoUpdate = 0;


// =============================================================================
// FUNCTION PROTOTYPES
// =============================================================================
void updateJoystick();

void handleWeatherPage();

void calculateRotatedServoTargets(int16_t px, int16_t py);
void updateMagnetPulse();

void drawStartupAnimation();
void drawRipples(int16_t tx, int16_t ty);
void updateWeatherLEDs(uint8_t hue);
void updateRainbowCycle();
CRGB colorWheel(uint8_t position);
void Fire2012();

void servicePowerButton(bool allowShutdown = true);

void handleSerialCommands();
Condition conditionFromWord(const String &w);

void attachArms(uint16_t staggerMs = 100);
void attachAll(uint16_t staggerMs = 100);
void blockingMoveArms(float closeEnough, bool allowShutdown = true);
void blockingMoveServo(ServoWrapper &s, float closeEnough, bool allowShutdown = true);
void pickUpFerrofluid();
void putItInPark();

ServoWrapper leftServo(PIN_SERVO_LEFT, SERVO_LEFT_TRIM), rightServo(PIN_SERVO_RIGHT, SERVO_RIGHT_TRIM), magnetServo(PIN_SERVO_MAGNET, MAGNET_SERVO_TRIM);

int16_t smoothMotion(int16_t current, int16_t target, float filterStrength = 0.0);
int32_t randomInRange(int32_t min = INT32_MIN, int32_t max = INT32_MAX);

inline float fmap(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void connectWiFi();
void fetchWeather();
void parseWeather(const String &payload);
void mapWeatherToParams();


// =============================================================================
// STATE MACHINE STUFF
// =============================================================================
bool magnetParked = false;
void pickUpFerrofluid();
void putItInPark();

bool wantWeather() { return currentMode == Mode::WEATHER; }
bool wantIdle()   { return currentMode == Mode::IDLE; }
bool wantPark()   { return currentMode == Mode::PARK; }
bool wantNever()  { return false; }

StateMachine fsm;

void setupStateMachine() {
  State* st_Weather = fsm.addState("Weather", 0, 0, pickUpFerrofluid, nullptr, handleWeatherPage);
  st_Weather->addAction(Action::Type::RE, newStateEntry);
  st_Weather->addAction(Action::Type::L, performNewEntryTimedPhase, 500);
  st_Weather->addTransition(st_Weather, wantNever);
  fsm.setInitialState(st_Weather);
  fsm.start();
}


// =============================================================================
// SETUP
// =============================================================================
void setup() {
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

  FastLED.addLeds<WS2812B, PIN_LEDS, GRB>(leds, NUM_LEDS);
  FastLED.setBrightness(LED_BRIGHTNESS);

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

  eyeScreenX = display.width() / 2;
  eyeScreenY = display.height() / 2;

  joystickButton.setDebounceTime(DEBOUNCE_MS);
  randomSeed(micros());
  swimHeading = randomInRange(0, 360) * DEG_TO_RAD;

  connectWiFi();

  currentMode = Mode::WEATHER;
  setupStateMachine();
  pickUpFerrofluid();

  delay(500);
  SERIAL_PRINTLN("weather mode online");
}

// =============================================================================
// MAIN LOOP
// =============================================================================
void loop() {
  static uint32_t powerButtonPressedTime = 0, stayAngryTimer = 0;

  carrierPCB.update();

  if (carrierPCB.stateChanged())
  {
    if (carrierPCB.buttonPressed(PowerLink::BTN_POWER)) powerButtonPressedTime = millis();
    if (carrierPCB.buttonReleased(PowerLink::BTN_POWER)) powerButtonPressedTime = 0;
  }

  if (powerButtonPressedTime != 0 && millis() - powerButtonPressedTime >= SOFT_SHUTDOWN_TIME)
  {
    if (!magnetParked) putItInPark();
    delay(500);
    carrierPCB.shutdown();
  }

  if (carrierPCB.stateChanged())
  {
    // Button 0: Lambda ( λ ) - a brief "angry" flare (LED only, not a mode change)
    if (carrierPCB.buttonPressed(PowerLink::BTN_0))
    {
      SERIAL_PRINTLN("λ");
      getAngry = true;
      stayAngryTimer = millis();
    }
  }

  if (millis() - lastWeatherFetch > WEATHER_REFRESH_MS)
  {
    fetchWeather();
    lastWeatherFetch = millis();
  }

  if (millis() - lastPatternUpdate > PATTERN_INTERVAL_MS)
  {
    FastLED.setBrightness(LED_BRIGHTNESS);
    if (getAngry && !magnetParked && millis() - stayAngryTimer <= stayAngryDuration)
    {
      Fire2012();
    }
    else if (weather.valid)
    {
      updateWeatherLEDs(gHue);
    }
    else
    {
      updateRainbowCycle();
      getAngry = false;
    }
    lastPatternUpdate = millis();
  }

  updateJoystick();
  handleSerialCommands();
  fsm.execute();
}

// =============================================================================
// INPUT HANDLING
// =============================================================================
void updateJoystick() {
  joystickButton.loop();
  joyXval = (4096 - analogRead(PIN_JOY_X)) >> 2;
  joyYval = (4096 - analogRead(PIN_JOY_Y)) >> 2;

  joyXAxis = joyXval < static_cast<int16_t>(JOY_CENTER - JOY_THRESHOLD) ? -1 : (joyXval > static_cast<int16_t>(JOY_CENTER + JOY_THRESHOLD) ? 1 : 0);
  joyYAxis = joyYval < static_cast<int16_t>(JOY_CENTER - JOY_THRESHOLD) ? -1 : (joyYval > static_cast<int16_t>(JOY_CENTER + JOY_THRESHOLD) ? 1 : 0);
}

// =============================================================================
// SERIAL TEST HARNESS
// =============================================================================
// Lets you override the forecast from the serial monitor for a single cycle
// without touching the network. Send:  inject <tempC> <condition>
//   e.g.  inject 32 sunny     (hot + clear)
//         inject -5 rain      (cold + precip)
// The next WEATHER_REFRESH_MS fetch from wttr.in overwrites it (one-shot).
Condition conditionFromWord(const String &w)
{
  if (w.indexOf("rain") >= 0 || w.indexOf("snow") >= 0 ||
      w.indexOf("storm") >= 0 || w.indexOf("drizzle") >= 0 ||
      w.indexOf("sleet") >= 0 || w.indexOf("shower") >= 0 ||
      w.indexOf("precip") >= 0) return Condition::PRECIP;
  if (w.indexOf("fog") >= 0 || w.indexOf("mist") >= 0 || w.indexOf("haze") >= 0) return Condition::FOG;
  if (w.indexOf("cloud") >= 0) return Condition::CLOUDY;
  if (w.indexOf("sun") >= 0 || w.indexOf("clear") >= 0 || w.indexOf("fair") >= 0) return Condition::SUNNY;
  return Condition::UNKNOWN;
}

void handleSerialCommands()
{
#ifdef USE_SERIAL
  static String line;
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (line.length() > 0) {
        line.toLowerCase();
        if (line.startsWith("inject ")) {
          String rest = line.substring(6);
          int sp = rest.indexOf(' ');
          String t = (sp > 0) ? rest.substring(0, sp) : rest;
          String w = (sp > 0) ? rest.substring(sp + 1) : "";
          int8_t tempC = (int8_t)t.toInt();
          Condition cond = conditionFromWord(w);
          weather.tempC = tempC;
          weather.cond = cond;
          weather.valid = true;
          mapWeatherToParams();
          SERIAL_PRINT("injected temp=");
          SERIAL_PRINT(tempC);
          SERIAL_PRINT(" cond=");
          SERIAL_PRINTLN((int)cond);
        }
        line = "";
      }
    } else {
      line += c;
    }
  }
#endif
}

// =============================================================================
// WEATHER MODE
// =============================================================================
// The magnet wanders through the fluid much like the old SWIM mode, but every
// parameter is fed by the forecast. The OLED eye mirrors the ferrofluid's
// position so you can see where it "is" inside the jar.
void handleWeatherPage()
{
  if (joystickButton.isPressed()) {
    // reserved
  }

  int16_t centerY = JOY_CENTER + gVertBias;

  static bool reattachServos = false;
  if (!reattachServos && newStateEntry) reattachServos = true;

  if (millis() - lastServoUpdate >= UPDATE_INTERVAL_MS)
  {
    // Pick a new wander target once we've essentially reached the current one
    // (or on first entry). The excursion distance is driven by the weather: a cold
    // forecast chooses a target close to the magnet's current spot (so it drifts
    // slowly), a hot forecast chooses a far target (so it roams widely). The
    // per-step rate stays constant, so temperature changes how FAR it travels
    // rather than how fast it steps -- which keeps the eye and ferrofluid coupled.
    int32_t dx = swimTargetX - swimX;
    int32_t dy = swimTargetY - swimY;
    if (newStateEntry || (dx * dx + dy * dy) <= (int32_t)SWIM_REACH * SWIM_REACH)
    {
      float angle = randomInRange(0, 360) * DEG_TO_RAD;
      float dist = fmap(gWarmth, 0.0f, 1.0f, (float)SWIM_COLD_DIST, (float)SWIM_HOT_DIST);
      swimTargetX = constrain(swimX + lroundf(cosf(angle) * dist), SWIM_MARGIN, JOY_MAX - SWIM_MARGIN);
      swimTargetY = constrain(swimY + lroundf(sinf(angle) * dist), SWIM_MARGIN, JOY_MAX - SWIM_MARGIN);
    }

    float speedReducer = performNewEntryTimedPhase ? 0.001 : 1.0;

    if (getAngry) speedReducer = 70.0;

    int16_t step = lroundf(gSwimSpeed * speedReducer);
    if (step < 1) step = 1;
    swimX += constrain(swimTargetX - swimX, -step, step);
    swimY += constrain(swimTargetY - swimY, -step, step);

    swimX = constrain(swimX, 0, JOY_MAX);
    swimY = constrain(swimY, 0, JOY_MAX);

    if (reattachServos)
    {
      calculateRotatedServoTargets(JOY_CENTER, centerY);
      attachArms(100);
      leftServo.write(leftServo.targetPos);
      rightServo.write(rightServo.targetPos);
      reattachServos = false;
    }
    else
    {
      calculateRotatedServoTargets(swimX, swimY);
    }

    leftServo.moveTo(leftServo.targetPos);
    rightServo.moveTo(rightServo.targetPos);

    lastServoUpdate = millis();
  }

  if (getAngry)
  {
    leftServo.setSmoothing(0.8); rightServo.setSmoothing(0.8);
  }
  else
  {
    leftServo.setSmoothing(gServoSmooth);
    rightServo.setSmoothing(gServoSmooth);
  }

  updateMagnetPulse();
  drawRipples(swimTargetX, swimTargetY);
}

// =============================================================================
// SERVO CONTROL
// =============================================================================
void calculateRotatedServoTargets(int16_t px, int16_t py) {
  constexpr float COS_45 = 0.70710678f;

  int16_t x = (MIRROR_MAGNET_X ? (JOY_CENTER + X_COORD_TRIM) - px
                               : px - (JOY_CENTER + X_COORD_TRIM));
  int16_t y = py - (JOY_CENTER + Y_COORD_TRIM);

  float rotatedX = static_cast<float>(x - y) * COS_45;
  float rotatedY = static_cast<float>(x + y) * -COS_45;

  int16_t servo1Value = constrain(static_cast<float>(rotatedX + JOY_CENTER), 0, JOY_MAX);
  int16_t servo2Value = constrain(static_cast<float>(rotatedY + JOY_CENTER), 0, JOY_MAX);

  leftServo.targetPos = fmap(servo1Value, 0, JOY_MAX, SERVO_MIN, SERVO_MAX);
  rightServo.targetPos = fmap(servo2Value, 0, JOY_MAX, SERVO_MAX, SERVO_MIN);
}

void updateMagnetPulse() {
  static uint32_t lastUpdate = 0;
  static int8_t magnetStep = 3;
  if (millis() - lastUpdate < gMagnetPulseMs) return;
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
// DISPLAY FUNCTIONS
// =============================================================================
void drawStartupAnimation() {
  display.clearDisplay();
  int maxRadius = max(display.width(), display.height()) / 2;
  for (int16_t r = maxRadius; r > 0; r -= 3) {
    display.fillCircle(display.width() / 2, display.height() / 2, r, SSD1306_INVERSE);
    display.display();
  }
}

// Pond-ripple OLED effect: concentric rings expand from the swim target like
// stones dropped in a pond. A new stone lands on the (smoothed) target every
// RIPPLE_SPAWN_MS; each ring keeps expanding until it is entirely off the display,
// so older stones overlap naturally. The screen is 1-bit, so rings are plain
// outlines (no grayscale/dither).
// A ring of radius r centered at (cx,cy) intersects the rectangular screen only
// if r reaches at least the nearest screen edge and no farther than the farthest
// corner. Stones are always centered on-screen, so the lower bound is 0 and the
// only real cull is r > farthest-corner distance. This bounds the work: rings
// that have fully left the display are skipped instead of being drawn to a huge
// (and ever-growing) radius.
static bool ringVisible(int16_t cx, int16_t cy, int16_t r) {
  if (r <= 0) return false;
  int32_t dx1 = cx,                dx2 = (SCREEN_WIDTH  - 1) - cx;
  int32_t dy1 = cy,                dy2 = (SCREEN_HEIGHT - 1) - cy;
  int32_t m1 = dx1 * dx1 + dy1 * dy1;
  int32_t m2 = dx2 * dx2 + dy1 * dy1;
  int32_t m3 = dx1 * dx1 + dy2 * dy2;
  int32_t m4 = dx2 * dx2 + dy2 * dy2;
  int32_t maxSq = m1;
  if (m2 > maxSq) maxSq = m2;
  if (m3 > maxSq) maxSq = m3;
  if (m4 > maxSq) maxSq = m4;
  return (int32_t)r * r <= maxSq;
}

// Add one ring's ink into the intensity buffer (thin circle) using an integer
// midpoint-circle so we avoid the (software, no-FPU) float trig per pixel.
// Overlapping rings sum, so intersections read back brighter -- that's what
// makes them merge.
static void rippleAddRing(int16_t cx, int16_t cy, int16_t r, uint8_t amt) {
  if (r <= 0) return;
  auto plot = [&](int16_t x, int16_t y) {
    if (x >= 0 && x < SCREEN_WIDTH && y >= 0 && y < SCREEN_HEIGHT) {
      int idx = y * SCREEN_WIDTH + x;
      int v = rippleBuf[idx] + amt;
      rippleBuf[idx] = v > 255 ? 255 : (uint8_t)v;
    }
  };
  int16_t x = r, y = 0, err = 1 - r;
  while (x >= y) {
    plot(cx + x, cy + y); plot(cx + y, cy + x);
    plot(cx - y, cy + x); plot(cx - x, cy + y);
    plot(cx - x, cy - y); plot(cx - y, cy - x);
    plot(cx + y, cy - x); plot(cx + x, cy - y);
    y++;
    if (err < 0) err += 2 * y + 1;
    else { x--; err += 2 * (y - x) + 1; }
  }
}

// Box blur the intensity buffer to soften the rings into a dithered blur.
static void rippleBlur() {
  static uint8_t tmp[SCREEN_WIDTH * SCREEN_HEIGHT];
  int br = RIPPLE_BLUR_RADIUS;
  for (int y = 0; y < SCREEN_HEIGHT; y++) {
    for (int x = 0; x < SCREEN_WIDTH; x++) {
      int sum = 0, cnt = 0;
      for (int k = -br; k <= br; k++) {
        int xx = x + k;
        if (xx >= 0 && xx < SCREEN_WIDTH) { sum += rippleBuf[y * SCREEN_WIDTH + xx]; cnt++; }
      }
      tmp[y * SCREEN_WIDTH + x] = (uint8_t)(sum / cnt);
    }
  }
  for (int y = 0; y < SCREEN_HEIGHT; y++) {
    for (int x = 0; x < SCREEN_WIDTH; x++) {
      int sum = 0, cnt = 0;
      for (int k = -br; k <= br; k++) {
        int yy = y + k;
        if (yy >= 0 && yy < SCREEN_HEIGHT) { sum += tmp[yy * SCREEN_WIDTH + x]; cnt++; }
      }
      rippleBuf[y * SCREEN_WIDTH + x] = (uint8_t)(sum / cnt);
    }
  }
}

// 8x8 ordered Bayer matrix (0..63) for dithering intensity -> 1 bit.
static const uint8_t bayer8[8][8] = {
  { 0, 32,  8, 40,  2, 34, 10, 42},
  {48, 16, 56, 24, 50, 18, 58, 26},
  {12, 44,  4, 36, 14, 46,  6, 38},
  {60, 28, 52, 20, 62, 30, 54, 22},
  { 3, 35, 11, 43,  1, 33,  9, 41},
  {51, 19, 59, 27, 49, 17, 57, 25},
  {15, 47,  7, 39, 13, 45,  5, 37},
  {63, 31, 55, 23, 61, 29, 53, 21}
};

// Pond-ripple OLED effect: concentric rings expand from the swim target like
// stones dropped in a pond. Rings are accumulated into a grayscale intensity
// buffer, box-blurred, then dithered to the 1-bit display with a Bayer matrix so
// overlapping rings merge naturally. A new stone lands every RIPPLE_SPAWN_MS.
void drawRipples(int16_t tx, int16_t ty) {
  int16_t targetX = map(tx, 0, JOY_MAX, 28, 100);
  int16_t targetY = map(ty, 0, JOY_MAX, 8, 56);

  constexpr float FILTER_STRENGTH = 0.8f;
  eyeScreenX = smoothMotion(eyeScreenX, targetX, FILTER_STRENGTH);
  eyeScreenY = smoothMotion(eyeScreenY, targetY, FILTER_STRENGTH);

  uint32_t now = millis();

  if (now - lastRippleSpawn >= RIPPLE_SPAWN_MS) {
    lastRippleSpawn = now;
    if (rippleCount < RIPPLE_MAX) {
      ripples[rippleCount].x = eyeScreenX;
      ripples[rippleCount].y = eyeScreenY;
      ripples[rippleCount].startMs = now;
      rippleCount++;
    } else {
      for (uint8_t i = 1; i < RIPPLE_MAX; i++) ripples[i - 1] = ripples[i];
      ripples[RIPPLE_MAX - 1].x = eyeScreenX;
      ripples[RIPPLE_MAX - 1].y = eyeScreenY;
      ripples[RIPPLE_MAX - 1].startMs = now;
    }
  }

  // Throttle the (heavier) redraw to ~30fps; the OLED holds the last frame.
  if (now - lastRippleDraw < RIPPLE_FRAME_MS) return;
  lastRippleDraw = now;

  memset(rippleBuf, 0, sizeof(rippleBuf));

  for (uint8_t i = 0; i < rippleCount; i++) {
    int16_t cx = ripples[i].x, cy = ripples[i].y;
    uint32_t age = now - ripples[i].startMs;
    if (age < 200) {
      for (int8_t dy = -2; dy <= 2; dy++)
        for (int8_t dx = -2; dx <= 2; dx++) {
          int x = cx + dx, y = cy + dy;
          if (x >= 0 && x < SCREEN_WIDTH && y >= 0 && y < SCREEN_HEIGHT) {
            int idx = y * SCREEN_WIDTH + x;
            int v = rippleBuf[idx] + 200;
            rippleBuf[idx] = v > 255 ? 255 : (uint8_t)v;
          }
        }
    }
    for (uint8_t n = 0; n < RIPPLE_MAX_RINGS; n++) {
      int16_t rad = (int16_t)(age * RIPPLE_SPEED) - (int16_t)(n * RIPPLE_SPACING);
      if (rad <= 0) continue;
      if (!ringVisible(cx, cy, rad)) continue;
      rippleAddRing(cx, cy, rad, RIPPLE_INTENSITY);
    }
  }

  rippleBlur();

  display.clearDisplay();
  uint8_t* fb = display.getBuffer();
  for (int y = 0; y < SCREEN_HEIGHT; y++) {
    for (int x = 0; x < SCREEN_WIDTH; x++) {
      uint8_t v = rippleBuf[y * SCREEN_WIDTH + x];
      uint8_t t = bayer8[y & 7][x & 7] * 4;
      if (v > t) fb[y / 8 * SCREEN_WIDTH + x] |= (uint8_t)(1 << (y & 7));
    }
  }
  display.display();

  // Retire a stone only once its SMALLEST ring (highest n) has expanded fully off
  // the display. Every other ring keeps growing until it too is entirely off-screen.
  uint8_t w = 0;
  for (uint8_t i = 0; i < rippleCount; i++) {
    uint32_t age = now - ripples[i].startMs;
    int16_t lastRad = (int16_t)(age * RIPPLE_SPEED) - (int16_t)((RIPPLE_MAX_RINGS - 1) * RIPPLE_SPACING);
    if (lastRad > 0 && !ringVisible(ripples[i].x, ripples[i].y, lastRad)) continue;
    ripples[w++] = ripples[i];
  }
  rippleCount = w;
}

// =============================================================================
// LED FUNCTIONS
// =============================================================================
void updateWeatherLEDs(uint8_t hue) {
  fill_solid(leds, NUM_LEDS, CHSV(hue, 255, LED_BRIGHTNESS));
  FastLED.show();
}

void updateRainbowCycle() {
  static uint16_t wheelOffset = 0;

  for (int i = 0; i < NUM_LEDS; i++) {
    leds[i] = colorWheel(((i * 256 / 32) + wheelOffset) & 255);
  }
  FastLED.show();

  wheelOffset++;
  if (wheelOffset >= 256 * 5) wheelOffset = 0;

  lastPatternUpdate = millis();
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

void servicePowerButton(bool allowShutdown) {
  static uint32_t pressedSince = 0;
  carrierPCB.update();
  if (carrierPCB.isPressed(PowerLink::BTN_POWER))
  {
    if (pressedSince == 0) pressedSince = millis();
    else if (allowShutdown && millis() - pressedSince >= SOFT_SHUTDOWN_TIME) carrierPCB.shutdown();
  }
  else
  {
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

static void steppedMove(StepMove moves[], uint8_t count, bool allowShutdown, float degPerSec = SLOW_MOVE_DPS)
{
  uint32_t moveStart = millis();
  uint32_t lastStep = millis();
  bool moving = true;
  while (moving && millis() - moveStart < BLOCKING_MOVE_TIMEOUT_MS)
  {
    servicePowerButton(allowShutdown);
    uint32_t now = millis();
    if (now - lastStep < SLOW_STEP_INTERVAL_MS) { delay(1); continue; }
    lastStep = now;

    float stepDeg = degPerSec * SLOW_STEP_INTERVAL_MS / 1000.0f;
    if (stepDeg < 0.05f) stepDeg = 0.05f;

    moving = false;
    for (uint8_t i = 0; i < count; i++)
    {
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

// =============================================================================
// SHARED SERVO HELPERS
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

void blockingMoveArms(float closeEnough, bool allowShutdown) {
  uint32_t moveStart = millis();
  bool moving = true;
  while (moving && millis() - moveStart < BLOCKING_MOVE_TIMEOUT_MS)
  {
    servicePowerButton(allowShutdown);
    leftServo.moveTo(leftServo.targetPos);
    rightServo.moveTo(rightServo.targetPos);
    moving = (fabsf(leftServo.getCurrentPos() - leftServo.targetPos) <= closeEnough &&
              fabsf(rightServo.getCurrentPos() - rightServo.targetPos) <= closeEnough);
  }
}

void blockingMoveServo(ServoWrapper &s, float closeEnough, bool allowShutdown) {
  uint32_t moveStart = millis();
  bool moving = true;
  while (moving && millis() - moveStart < BLOCKING_MOVE_TIMEOUT_MS)
  {
    servicePowerButton(allowShutdown);
    s.moveTo(s.targetPos);
    moving = fabsf(s.getCurrentPos() - s.targetPos) <= closeEnough;
  }
}

void putItInPark()
{
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

void pickUpFerrofluid()
{
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

  swimX = JOY_CENTER;
  swimY = JOY_CENTER + 500;
  swimHeading = atan2f(JOY_CENTER - swimY, JOY_CENTER - swimX);
}

int16_t smoothMotion(int16_t current, int16_t target, float filterStrength) {
  if (current == target) return current;
  float smoothed = current * filterStrength + target * (1.0f - filterStrength);
  int16_t result = static_cast<int16_t>(lroundf(smoothed));
  if (result == current) {
    result += (target > current) ? 1 : -1;
  }
  return result;
}

void Fire2012()
{
  static uint8_t heat[NUM_LEDS];
  static bool gReverseDirection = false;
  constexpr uint32_t FRAMES_PER_SECOND = 8;

  static uint32_t fireTime = 0, blurTime = 0;

  constexpr uint8_t COOLING = 20;
  constexpr uint8_t SPARKING = 50;

  if (millis() - fireTime >= 1000 / FRAMES_PER_SECOND)
  {
    for( int i = 0; i < NUM_LEDS; i++) {
      heat[i] = qsub8( heat[i],  random8(0, ((COOLING * 10) / NUM_LEDS) + 2));
    }

    for( int k= NUM_LEDS - 1; k >= 2; k--) {
      heat[k] = (heat[k - 1] + heat[k - 2] + heat[k - 2] ) / 3;
    }

    if( random8() < SPARKING ) {
      int y = random8(NUM_LEDS);
      heat[y] = qadd8( heat[y], random8(160,255) );
    }

    for( int j = 0; j < NUM_LEDS; j++) {
      CRGB color = HeatColor( heat[j]);
      int pixelnumber;
      if( gReverseDirection ) {
        pixelnumber = (NUM_LEDS-1) - j;
      } else {
        pixelnumber = j;
      }
      leds[pixelnumber] = color;
    }

    FastLED.show();
    fireTime = millis();
  }

  if (millis() - blurTime >= 100 / (FRAMES_PER_SECOND)) { blur1d(leds, NUM_LEDS, 120); FastLED.show(); blurTime = millis(); }
}

// =============================================================================
// WIFI + WEATHER
// =============================================================================
void connectWiFi()
{
  WiFi.mode(WIFI_STA);
  WiFiManager wm;
  wm.setConfigPortalTimeout(300);
  wm.setDebugOutput(false);

  bool connected = wm.autoConnect("SpaceOddity-Setup", "configureme");
  if (!connected) {
    SERIAL_PRINTLN("WiFi not configured; running offline with neutral params");
    WiFi.mode(WIFI_OFF);
  } else {
    SERIAL_PRINTLN(WiFi.localIP());
    fetchWeather();
    lastWeatherFetch = millis();
  }
}

void fetchWeather()
{
  if (WiFi.status() != WL_CONNECTED) {
    SERIAL_PRINTLN("wifi down; skipping fetch");
    return;
  }

  static WiFiClientSecure client;
  static HTTPClient https;
  client.setInsecure();

  const char* url = "https://wttr.in/?format=%C+%t+%w&m&lang=en";
  if (!https.begin(client, url)) {
    SERIAL_PRINTLN("https begin failed");
    return;
  }

  int httpResponseCode = https.GET();
  if (httpResponseCode == HTTP_CODE_OK) {
    String payload = https.getString();
    SERIAL_PRINT("forecast: ");
    SERIAL_PRINTLN(payload);
    parseWeather(payload);
  } else {
    SERIAL_PRINT("forecast http error: ");
    SERIAL_PRINTLN(httpResponseCode);
  }
  https.end();
}

void parseWeather(const String &payload)
{
  int cIdx = payload.indexOf("°C");
  if (cIdx < 0) cIdx = payload.indexOf("°F");
  if (cIdx < 0) {
    SERIAL_PRINTLN("unparsed forecast");
    return;
  }

  int tStart = cIdx;
  while (tStart > 0) {
    char c = payload[tStart - 1];
    if (isdigit(c) || c == '+' || c == '-' || c == ' ') tStart--;
    else break;
  }

  String cond = payload.substring(0, tStart);
  cond.trim();
  int8_t tempC = (int8_t)payload.substring(tStart, cIdx).toInt();

  Condition condition = Condition::UNKNOWN;
  cond.toLowerCase();
  if (cond.indexOf("rain") >= 0 || cond.indexOf("snow") >= 0 ||
      cond.indexOf("storm") >= 0 || cond.indexOf("drizzle") >= 0 ||
      cond.indexOf("sleet") >= 0 || cond.indexOf("shower") >= 0) {
    condition = Condition::PRECIP;
  } else if (cond.indexOf("fog") >= 0 || cond.indexOf("mist") >= 0 || cond.indexOf("haze") >= 0) {
    condition = Condition::FOG;
  } else if (cond.indexOf("cloud") >= 0) {
    condition = Condition::CLOUDY;
  } else if (cond.indexOf("sun") >= 0 || cond.indexOf("clear") >= 0 || cond.indexOf("fair") >= 0) {
    condition = Condition::SUNNY;
  }

  weather.tempC = tempC;
  weather.cond  = condition;
  weather.valid = true;
  mapWeatherToParams();

  SERIAL_PRINT("temp=");
  SERIAL_PRINT(tempC);
  SERIAL_PRINT(" cond=");
  SERIAL_PRINTLN((int)condition);
}

void mapWeatherToParams()
{
  gWarmth = constrain((weather.tempC - WARMTH_MIN_C) / (float)(WARMTH_MAX_C - WARMTH_MIN_C), 0.0f, 1.0f);

  float h = 160.0f * (1.0f - gWarmth);
  if (weather.cond == Condition::PRECIP || weather.cond == Condition::FOG) h = max(h, 175.0f);
  gHue = (uint8_t)h;

  gSwimSpeed = fmap(gWarmth, 0.0f, 1.0f, 0.8f, 4.5f);

  gMagnetPulseMs = (uint16_t)fmap(gWarmth, 0.0f, 1.0f, 1000, 30);
  if (weather.cond == Condition::PRECIP || weather.cond == Condition::FOG) gMagnetPulseMs = 1100;

  gVertBias = (gWarmth < 0.4f || weather.cond == Condition::PRECIP || weather.cond == Condition::FOG) ? 350 : 0;

  gServoSmooth = fmap(gWarmth, 0.0f, 1.0f, SERVO_SMOOTH_COLD, SERVO_SMOOTH_HOT);
}

int32_t randomInRange(int32_t min, int32_t max) {
  if (min > max) { int32_t t = min; min = max; max = t; }
  return min + (int32_t)random((uint32_t)(max - min + 1));
}
