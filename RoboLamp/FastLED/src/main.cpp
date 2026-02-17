#pragma region README
/*
 ************************************************************************************
 * Robolamp Stock Code
 * 05.13.2025
 * Version: 1.0.6
 * Author: Crunchlabs LLC
 *
 *   This firmware powers Hack 3 for the Crunchlabs RoboLamp, an animated and "illuminating"
 *   desk companion. This hack introduces an awesome library called FastLED to control the 
 *   lamp's Neopixel ring, enabling vibrant color animations and effects. This should really
 *   just be seen as an introduction to the idea of palettes, which have replaced the solid
 *   colors from the main code. Now, when you click on "orange," for example, you get a fiery
 *   blend of reds and yellows instead of a single static hue. Pressing the joystick "up" and
 *   "down" in the color selection menu will change the speed of the animation instead of the
 *   brightness. For other cool hacks, check out our Discord, or submit your own and earn your
 *   rightful place in our trophy case!
 *
 ************************************************************************************
 */
#pragma endregion README

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#pragma region LICENSE
/*
  ************************************************************************************
  * MIT License
  *
  * Copyright (c) 2024 Crunchlabs LLC (DEALR Code)

  * Permission is hereby granted, free of charge, to any person obtaining a copy
  * of this software and associated documentation files (the "Software"), to deal
  * in the Software without restriction, including without limitation the rights
  * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  * copies of the Software, and to permit persons to whom the Software is furnished
  * to do so, subject to the following conditions:
  *
  * The above copyright notice and this permission notice shall be included in all
  * copies or substantial portions of the Software.
  *
  * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
  * INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
  * PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
  * HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF
  * CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE
  * OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
  *
  ************************************************************************************
*/
#pragma endregion LICENSE

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
LIBRARIES AND HEADER FILES
*/
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#pragma region LIBRARIES AND HEADER FILES

#include <Arduino.h>
#include <avr/pgmspace.h>
#include <OneButton.h> // Include OneButton library
#include <Adafruit_TiCoServo.h>
#include <EEPROM.h>
#include "Types.h"
#include "Config.h"
#include "Animations.h"
#include <FastLED.h>
#include "ColorPalettes.h"

#pragma endregion LIBRARIES AND HEADER FILES

//////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
PINS DEFINITIONS
*/
//////////////////////////////////////////////////////////////////////////////////////////////////////////

#pragma region PIN DEFINITIONS

OneButton button1(8, true);     // Preset button 1
OneButton button2(7, true);     // Preset button 2
OneButton button3(2, true);     // Preset button 3
OneButton button4(4, true);     // Preset button 4
OneButton downButton(A1, true); // Up button handler (A1 = D15)
OneButton upButton(A2, true);   // Down button handler (A2 = D16)
OneButton joyButton(A3, true);  // Joystick button handler (A3 = D17)

const uint8_t neopixelPin = 3;        // Data pin for neopixels
const uint8_t warmLEDpin = 5;         // PWM for [warm] LED brightness
const uint8_t coolLEDpin = 6;         // PWM for [cool] LED brightness
const uint8_t servoYawPWMPin = 10;    // PWM for [yaw] servo = D10
const uint8_t servoPitchPWMPin = 9;   // PWM for [pitch] servo
const uint8_t motorSpeedPin = 11;     // PWM signal for motor speed
const uint8_t motorDirectionPin = 12; // Motor direction control pin
const uint8_t buttonUpPin = A1;       // Pin for up button
const uint8_t buttonDownPin = A2;     // Pin for down button
const uint8_t JoyYawPin = A6;         // Joystick yaw axis potentiometer (A6 = D20)
const uint8_t JoyPitchPin = A7;       // Joystick pitch axis potentiometer (A7 = D21)
const uint8_t heightEncoderPin = A0;  // Pot encoder for height of lamp

#define DATA_PIN 3  // Or your actual Neopixel data pin
#define NUM_LEDS 12 // Or however many pixels you have
#define LED_TYPE WS2812B
#define COLOR_ORDER GRB

#pragma endregion PIN DEFINITIONS

//////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
LIBRARY ASSIGNMENTS
*/
//////////////////////////////////////////////////////////////////////////////////////////////////////////

#pragma region LIBRARY ASSIGNMENTS

Adafruit_TiCoServo servoYaw;
Adafruit_TiCoServo servoPitch;

#pragma endregion LIBRARY ASSIGNMENTS

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
GLOBAL VARIABLES
*/
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#pragma region GLOBAL VARIABLES

// LED CONTROL

#pragma region LED Control

const uint8_t neopixelRotationOffset = 6;         // Which neopixel in the ring counts as the top middle one (not necessarily the first in the sequence)
bool useColoredLight = true;                      // Tracks whether or not colored Neopixels are being used
uint16_t hue = 0;                                 // Stores current hue value for LEDs
uint8_t logicalSelectedIndex = 0;                 // Global variable to keep track of the logical color selection
uint8_t currentBrightness = 128;                  // Initialize to mid-bright-brightness
unsigned long lastNeopixelUpdate = 0;             // Store the last time Neopixels were updated
const unsigned long neopixelUpdateInterval = 100; // LED update interval in milliseconds
uint32_t neopixelBuffer[NUM_LEDS];                // Buffer for storing Neopixel colors
uint8_t selectedPixel = 0;                        // The current selected neopixel
const uint8_t dimPixel = 10;                      // Brightness level for non-selected pixels during color selection (used to visually de-emphasize inactive options)
bool neopixelInterpolating = false;               // Flag for resetting our color interpolation function
bool fadingStarted = false;

const uint8_t brightnessLevels[] PROGMEM = {
    40,
    64,
    100,
    180,
    255};

uint8_t currentBrightnessIndex = 2; // Index to track the current brightness level

struct NeopixelInterpolator
{
  bool active = false;        // Whether a hue/brightness fade is currently in progress
  uint32_t startTime = 0;     // Timestamp when the interpolation began (in millis)
  uint16_t duration = 1000;   // Total duration of the interpolation (in milliseconds)
  uint16_t fromHue = 0;       // Starting hue (0–65535 HSV range)
  uint16_t toHue = 0;         // Target hue to fade to
  uint8_t fromBrightness = 0; // Starting brightness (0–255)
  uint8_t toBrightness = 0;   // Target brightness to fade to
};

const uint16_t customHueRing[9] = {
    10,    // red - red != 0 so "0" can be used as "unset"
    3800,  // orange
    8000,  // yellow
    15500, // lime
    22000, // green
    31000, // cyan
    42000, // blue
    48000, // violet
    62000  // pink
};

#define NUM_COLOR_OPTIONS 12 // 9 palettes + 3 white lights
#define NUM_CUSTOM_COLORS 9  // Just the palette section

NeopixelInterpolator pixelFade;

// SMILEY VARIABLES

SmileyEyeDirection currentSmileyDirection = LOOK_LEFT;

// Used to control random blinking behavior of the smiley face (when active)
unsigned long lastSmileyBlinkTime = 0;  // Time when the last blink completed
unsigned long nextSmileyBlinkDelay = 0; // Random delay before the next blink
bool smileyBlinkInProgress = false;     // True while the eyes are "closed"
unsigned long smileyBlinkStart = 0;     // Time when the current blink started
bool eyesSuppressed = 0;                // True while we want both eyes closed for dramatic movements

// Used to control side-to-side eye movement (shifting glance)
bool smileyShiftInProgress = false;     // True while the eyes are shifted to the side
unsigned long lastSmileyShiftTime = 0;  // Time when the last eye shift finished
unsigned long nextSmileyShiftDelay = 0; // Random delay before next eye shift
unsigned long smileyShiftStart = 0;     // Time when the current eye shift started

bool smileyDisabledAfterUserInput = false; // If true, disables the smiley face after joystick or button interaction

// Excited Mood: Height Bounce Timing
bool excitedHeightBounceActive = false;        // True if the lamp is currently doing a "height bounce" motion
unsigned long excitedHeightBounceStart = 0;    // Time when the height bounce began
unsigned long excitedHeightBounceDuration = 0; // Duration of the current bounce cycle
uint16_t excitedOriginalHeight = 0;            // Height to return to after bounce completes

#pragma endregion LED Control

//////////////////////////////////////////////////////////////////////////////////////////////////////////
// BUTTON PRESETS
//////////////////////////////////////////////////////////////////////////////////////////////////////////

#pragma region Button Presets

bool presetActive = false; // Flag to indicate whether the preset is currently active

// LAMP PRESET STRUCT
// Stores a snapshot of the lamp's visual and positional state.
// Each preset includes servo angles, height, color/light mode, and brightness.
struct LampPreset
{
  uint8_t yawPosition;   // Saved yaw (left-right) angle of the lamp head (0–180)
  uint8_t pitchPosition; // Saved pitch (up-down) angle of the lamp head (0–180)
  uint16_t height;       // Saved height of the lamp body (raw encoder value)
  bool useColoredLight;  // True = use colored Neopixels; False = use white LEDs
  uint8_t brightness;    // Brightness level (0–255) applied to either light type
  uint16_t hue;          // Hue value (0–65535) for colored Neopixel mode
  uint8_t whiteMode;     // White light type: 1 = warm, 2 = soft, 3 = cool. Only used if useColoredLight == false
};
LampPreset presets[4]; // Array to store 4 presets

#pragma endregion Button Presets

//////////////////////////////////////////////////////////////////////////////////////////////////////////
// STATE CONTROLS
//////////////////////////////////////////////////////////////////////////////////////////////////////////

#pragma region State Control

// Motor Motion States
bool isMoving = false; // True if the lamp height is currently adjusting via motor

// Mood Snapback State
bool snapbackAbandoned = false;     // True if the user moved the joystick again before snapback finished
bool pendingDistractedSnap = false; // True if a bump-to-return ("snapback") is queued in distracted or startup mode

// User Interaction Timings
unsigned long nextStartupWanderTime = 0;              // When the lamp is next allowed to perform idle wandering after startup
unsigned long lastUserInputTime = 0;                  // Timestamp of the last user input (joystick or button)
bool manualControlActive = false;                     // True if the user is currently moving the head with the joystick
bool colorSelectionMode = false;                      // True if the color selection menu is active (disables mood/servo control)
const unsigned long userNotControllingInterval = 500; // Time (ms) to wait before assuming joystick is idle
bool wanderingAfterStartup = false;                   // True if idle wander is active after startup animation and no mood is selected
bool userHasTakenControl = false;                     // Another flag for detecting when a user has taken control

#pragma endregion State Control

//////////////////////////////////////////////////////////////////////////////////////////////////////////
// SERVO CONTROLS
//////////////////////////////////////////////////////////////////////////////////////////////////////////

#pragma region Servo Control

// Servo Update Rate
const uint8_t servoUpdateInterval = 20; // Interval (ms) between servo update cycles (applies to motion smoothing)

// === Target and Current Head Angles ===
int16_t targetServoYaw = 90;    // Target yaw angle for autonomous or preset-driven head motion
int16_t targetServoPitch = 90;  // Target pitch angle for autonomous or preset-driven head motion
int16_t currentServoYaw = 90;   // Current interpolated yaw position of the lamp head
int16_t currentServoPitch = 90; // Current interpolated pitch position of the lamp head

// === Original (pre-animation) Head Angles ===
int16_t originalServoYaw = 90;   // Yaw angle at the start of a mood or preset (used for return-to-origin)
int16_t originalServoPitch = 90; // Pitch angle at the start of a mood or preset (used for return-to-origin)

// === Interpolation Start Values ===
int16_t startYaw = 90;   // Yaw position at the start of an interpolation (used for easing)
int16_t startPitch = 90; // Pitch position at the start of an interpolation (used for easing)

// === Lamp Height Control ===
uint16_t currentHeight = 0; // Current height of the lamp (read from encoder)
uint16_t targetHeight = 0;  // Target height during autonomous transitions (e.g. excited mode bounce)

// === Servo Output Tracking ===
int16_t lastServoYawWritten = -1;   // Last written yaw PWM value (used to avoid redundant writes)
int16_t lastServoPitchWritten = -1; // Last written pitch PWM value (used to avoid redundant writes)

// === Output Filtering (Smooth Motion) ===
float filteredServoYaw = 90;   // Low-pass filtered yaw output to suppress jitter
float filteredServoPitch = 90; // Low-pass filtered pitch output to suppress jitter

// === User Input / Bump Detection ===
bool userMovedHeadDeliberately = false;    // True if joystick input was intentional (not noise)
const uint16_t snapbackBumpThreshold = 50; // Joystick magnitude threshold to trigger a bump return

#pragma endregion Servo Control

//////////////////////////////////////////////////////////////////////////////////////////////////////////
// JOYSTICK CONTROLS
//////////////////////////////////////////////////////////////////////////////////////////////////////////

#pragma region Joystick Control

int16_t joystickCenterYaw = 512;                // Default center for Yaw-axis
int16_t joystickCenterPitch = 512;              // Default center for Pitch-axis
const uint8_t deadZoneThreshold = 100;          // Define dead zone threshold
unsigned long lastManualControlTime = 0;        // Tracks last time a user deliberately controlled the joystick
const unsigned long manualControlInterval = 20; // Joystick update interval in ms

#pragma endregion Joystick Control

#pragma Interpolation Controls

//////////////////////////////////////////////////////////////////////////////////////////////////////////
// PID AND INTERPOLATION CONTROLS
//////////////////////////////////////////////////////////////////////////////////////////////////////////

float integralYaw = 0;
float integralPitch = 0;
float previousErrorX = 0;
float previousErrorY = 0;

InterpolationMode currentInterpMode = INTERP_LINEAR;

//////////////////////////////////////////////////////////////////////////////////////////////////////////

AnimationType currentAnimationType = ANIM_STARTUP; // default

const uint8_t MAX_KEYFRAMES = 13;         // Max possible number of keyframes in an animation. Can be increased at the cost of memory.
Keyframe startupAnimation[MAX_KEYFRAMES]; // Startup animation array.
uint8_t startupKeyframeCount = 0;         // For tracking count of startup keyframe animation.
uint8_t currentKeyframeIndex = 0;         // For tracking the current keyframe we're on.
unsigned long keyframeStartTime = 0;      // For tracking the start time of any given keyframe animation.
bool playingStartupAnimation = false;     // For tracking whether or not we're currently doing our startup animation.
uint16_t startHeight = 0;                 // For tracking the starting height of the lamp head.
uint16_t targetHeightDuringAnimation = 0;

const unsigned long frameInterval = 5;
unsigned long interpolationStartTime = 0;
int16_t interpolationDuration = 1500; // in milliseconds

#pragma endregion Interpolation Controls

#pragma region Mood Management

//////////////////////////////////////////////////////////////////////////////////////////////////////////
// MOOD CONSTANTS
// General Mood Variables
WhiteMode currentWhiteMode = WHITE_SOFT;

// Distracted Variables
unsigned long lastJoystickMotionTime = 0;
int16_t originalHeadYaw = 90;
int16_t originalHeadPitch = 90;
unsigned long lastWanderTime = 0;
unsigned long nextWanderInterval = 0;
bool isWandering = false;
bool distractedHasReturnedToOrigin = true;
bool readyToUpdateOriginalHead = false;
bool moodDrivingServos = false;
bool isReturningFromWander = false;
bool userSelectedMood = false;

// Excited Variables
unsigned long nextExcitedInterval = 0;
unsigned long lastExcitedChangeTime = 0;
Keyframe excitedAnimations[2][MAX_KEYFRAMES];
uint8_t excitedKeyframeCounts[6]; // Number of keyframes in each animation
uint16_t excitedOriginalHue = 0;
uint8_t currentExcitedIndex = 0;
uint8_t excitedOriginalBrightness = 0;
uint8_t excitedOriginalWhiteMode = WHITE_SOFT;
bool excitedOriginalUseColoredLight = true;
bool excitedLightOverridden = false;

// Moody Constants
uint8_t originalBrightness = 128;
uint16_t originalHue = 0;
unsigned long moodyTransitionStartTime = 0;
const uint16_t moodyPhaseDuration = 4000; // 4 seconds per phase
uint8_t moodyTransitionPhase = 0;         // 0 = fade out white, 1 = fade in blue
bool pendingMoodySnap = false;
bool moodyDroopDown = true;
bool moodyInTransition = false;
bool moodyInitialCenterReached = false;
bool whiteLightsAreOff = false;
unsigned long whiteFadeStartTime = 0;

// Saved original state
uint8_t originalMoodyYaw;
uint8_t originalMoodyPitch;
uint16_t originalMoodyHeight;
bool originalUseColoredLight;
WhiteMode originalWhiteMode;

// Moody timing
unsigned long moodyStartTime = 0;
unsigned long moodyPhaseStartTime = 0;
bool moodyInDroop = false;
bool moodyInPause = false;

// Moody Mood State
bool moodyInDelay = false;
bool moodyDescending = false;
bool moodyPaused = false;
bool moodyRising = false;
bool moodyLooping = false;
bool moodySnapbackCompleted = false;

bool hasRestoredMoodyOnce = false;
bool moodyStartedInterpToCenter = false;

unsigned long lastMoodyPauseTime = 0;
unsigned long lastMoodyLoopResetTime = 0;
bool moodyHeightRestoreComplete = false;
bool moodyWhiteLightsDisabled = false;

MoodState currentMood = MOOD_FOCUSED; // Start in default "focused" mood

#pragma endregion Mood Management

//////////////////////////////////////////////////////////////////////////////////////////////////////////
// EEPROM VARIABLES

#pragma region EEPROM

const uint16_t EEPROM_MAGIC_ADDR = 512;
const byte EEPROM_MAGIC = 0xA5;

#pragma endregion EEPROM

#pragma region FastLED Support

CRGB leds[NUM_LEDS];

unsigned long lastPaletteUpdate = 0;
const unsigned long paletteInterval = 15;
bool isPaletteAnimationEnabled = false;
uint8_t selectedPaletteIndex = 0; // Set when confirming color
uint8_t colorOffset = 0;

#pragma endregion FastLED Support

#pragma endregion GLOBAL VARIABLES

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
FUNCTION PROTOTYPES
Function Prototypes let a program know what functions are going to be defined later on. This isn't always necessary in every IDE, but it's good practice, and
can serve as a kind of table of contents for what to expect later.
*/
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#pragma region FUNCTION PROTOTYPES
// Setup Functions and Helpers
void initializeSerial();
void initializePins();
void initializeServos();
void initializeStartupLighting();
void initPalettes();
void initializeHeightEncoder();
void initializeButtons();
void attachColorSelectionHandler();
void handleColorSelectionInput();
uint8_t calculatePulseBrightness(uint8_t baseBrightness);
void attachPresetShortcuts();
void attachMoodSelectionShortcuts();
void attachMotorButtons();
void initializeEEPROM();
void calibrateJoystick();
void initializeInterpolationDefaults();

// Presets
void applyPreset(uint8_t presetIndex);
void initializePresetState();
void calculatePresetInterpolation(const LampPreset &preset);
void applyPresetLighting(const LampPreset &preset);
void updateCurrentBrightnessIndex(uint8_t brightness);
void saveCurrentValuesToPreset(uint8_t presetIndex);
void syncGlobalsFromPreset(uint8_t presetIndex);

// MOTION - Head Servos
void controlServos();
bool headMovedSignificantly(int oldYaw, int oldPitch, int newYaw, int newPitch, float thresholdDegrees = 5.0f);
void handleManualControl(unsigned long now, int16_t deltaYaw, int16_t deltaPitch);
void handleSnapbackTriggers(unsigned long now, unsigned long bumpDuration);
void beginSnapback(unsigned long now, int16_t yaw, int16_t pitch, InterpolationMode mode);
void resumeOriginalPosition();
void restoreMoodySnapback();
void restoreStartupSnapback();
void restoreDefaultSnapback();
void applySnapbackLighting();
void restoreServoPosition(float duration, InterpolationMode mode);
void moveServosToTarget();
float applyEasing(float t, InterpolationMode mode);
void cancelMoodMotion();
void completeServoMotion();
inline void clampHeadToDistractedBounds(int16_t &yaw, int16_t &pitch);
inline void clampHeadToDistractedBounds() { clampHeadToDistractedBounds(originalHeadYaw, originalHeadPitch); }
inline void clampHeadToExcitedBounds(uint16_t &yaw, uint16_t &pitch);

// MOTION - Resume Original Position Helpers

// MOTION - Height Motor
void motorControl(uint8_t speed, bool direction);
void moveToHeight(int16_t targetHeight);

// EEPROM
void savePresetsToEEPROM();
bool loadPresetsFromEEPROM();

// MOODS - Setters
void setMoodDistracted(bool silent = true);
void setMoodHappy(bool silent = true);
void setMoodMoody(bool silent = true);
void setMoodFocused(bool silent = true);

// MOODS - Updates
// Distracted Mood Updates and Helper Functions
void updateDistractedMood();
void updateDistractedHeadOriginIfIdle(unsigned long now);
void triggerDistractedWanderIfReady(unsigned long now);
void beginInterpolatedWander(unsigned long now, unsigned long durationMin, unsigned long durationMax,
                             unsigned long intervalMin, unsigned long intervalMax);

// Happy Mood Updates and Helper Functions
void updateHappyMood();
void updateSmileyLightingIfNeeded();
void updateSmileyBlinkAndShiftIfEnabled();
void updateExcitedHeadOriginIfIdle(unsigned long now);
void triggerExcitedWanderIfReady(unsigned long now);
void handleHeightBounceCycle(unsigned long now);
void maybeTriggerExcitedAnimation(unsigned long now);

// Moody Mood Updates and Helper Functions
void updateMoodyMood(unsigned long now);
void handleMoodyDelayPhase(unsigned long now);
void fadeOutWhiteLightsDuringTransition(unsigned long now);
void beginMoodyNodCycleIfCentered(unsigned long now);
void handleMoodyNodCycle(unsigned long now);

// MOODS -Indicators
void indicateActivatedMood(MoodBlinkType blinkType, uint32_t color = 0, uint8_t brightness = 255);

// LED_CONTROL_UTILS
// Shared
uint8_t getCurrentBrightness();
void turnOffAllLights();
void clearPixels();
void renderColorMenu(uint8_t offset);
void restoreLightStateFromPreset(const LampPreset &preset);
void restorePreferredLighting();

// White Light
void setWhiteLightMode(WhiteMode mode, uint8_t brightness, bool disableSmiley = false);
void fadeWhiteLightsToBlack(float t, WhiteMode mode, uint8_t originalBrightness);
void resetWhiteFadeState();
void captureOriginalLightState();

// Neopixels
void setNeopixelColor(uint16_t hue, uint8_t brightness, bool force = false);
void applyColorLighting(uint16_t hue, uint8_t brightness);
void applyPalette(uint16_t hue);
void updatePaletteAnimation(uint16_t hue, uint8_t brightness);
void updateSelectedPixel(uint8_t currentBrightness);
void applyBufferedNeopixels();
uint8_t calculatePulseBrightness();
uint16_t interpolateHue(uint16_t fromHue, uint16_t toHue, float t);
void restoreNeopixelBuffer(const uint32_t *colorBuffer, uint8_t brightness);
void restoreWhiteLightMode(WhiteMode mode, uint8_t brightness);
void startNeopixelInterpolation(uint16_t fromHue, uint8_t fromBrightness,
                                uint16_t toHue, uint8_t toBrightness,
                                uint16_t durationMs = 1000);
void updateNeopixelInterpolation();
void renderColorSelectionMenu();

// Smiley Face UI
bool isSmileyModeActive();
void applySmileyLighting();
void showStartupSmiley();
void updateSmileyBlinkAndShift();
uint32_t getSmileyColor();
void finishStartupWithoutAnimation();

// ANIMATIONS
void loadStartupAnimation();
void playKeyframeAnimation();
void handleSmileyDuringStartup();
void finishStartupAnimation();
void updateCurrentKeyframeAnimation();
void loadExcitedAnimation(uint8_t index);
float interpolateQuadraticBezier(float p0, float p1, float p2, float t);
void clampKeyframe(Keyframe &kf);

// Loop Helper Functions (find them homes)
void tickAllButtons();
void handleMoodIdleTrigger(unsigned long now);
bool shouldUpdateIdle();
void handleDistractedIdle(unsigned long now);
void handleHappyIdle(unsigned long now);
void handleMoodyIdle(unsigned long now);
bool shouldRestoreMoodyAfterSnapback(unsigned long now);
void restoreMoodyAfterSnapback(unsigned long now);
void resetMoodyState();
bool shouldUpdateDistractedSnapback(unsigned long now);
void updateDistractedSnapbackOrigin(unsigned long now);
bool shouldBeginFocusedStartupWander(unsigned long now);
void beginFocusedStartupWander(unsigned long now);
void updateSmileyIfActive();
void updateFrameBasedMovement(unsigned long now);
void updateStartupKeyframeAnimation();

static inline uint8_t wrapIndex(int16_t v, uint8_t mod);

#pragma endregion FUNCTION PROTOTYPES

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
SETUP FUNCTION
In the setup for this build, we make sure our sensors are attached and working, and we run a few startup routines.
*/
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#pragma region SETUP

void setup()
{
  initializeSerial();
  initializePins();
  initializeServos();
  initializeStartupLighting();

  FastLED.addLeds<LED_TYPE, DATA_PIN, COLOR_ORDER>(leds, NUM_LEDS);
  FastLED.clear();
  FastLED.show();

  initPalettes();
  initializeHeightEncoder();
  initializeButtons();
  initializeEEPROM();
  calibrateJoystick();
  initializeInterpolationDefaults();

  if (Config::useStartupAnim)
  {
    loadStartupAnimation(); // Play keyframe sequence
  }
  else
  {
    finishStartupWithoutAnimation();
  }
}

#pragma endregion SETUP

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
MAIN LOOP
*/
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#pragma region LOOP

void loop()
{
  unsigned long now = millis();

  if (!colorSelectionMode && isPaletteAnimationEnabled && millis() - lastPaletteUpdate >= paletteInterval)
  {
    updatePaletteAnimation(selectedPaletteIndex, getCurrentBrightness());
  }

  if (colorSelectionMode)
  {
    handleColorSelectionInput();
    updateSelectedPixel(getCurrentBrightness()); // uses updated index
    FastLED.show();
  }
  tickAllButtons();
  handleMoodIdleTrigger(now);
  updateSmileyIfActive();
  updateNeopixelInterpolation();
  updateFrameBasedMovement(now);
  updateStartupKeyframeAnimation();
}

#pragma endregion LOOP

//////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
FUNCTION DEFINITIONS
*/
//////////////////////////////////////////////////////////////////////////////////////////////////////////

#pragma region FUNCTIONS

//////////////////////////////////////////////////////////////////////////////////////////////////////////
// LIGHT, DISPLAY, AND MENU FUNCTIONS

#pragma region LED_CONTROL_UTILS

void setWhiteLightMode(WhiteMode mode, uint8_t brightness, bool disableSmiley)
{
  useColoredLight = false;
  selectedPixel = static_cast<uint8_t>(mode);

  if (disableSmiley)
  {
    smileyDisabledAfterUserInput = true;
  }

  wanderingAfterStartup = false;

  switch (mode)
  {
  case WHITE_WARM:
    analogWrite(warmLEDpin, brightness);
    analogWrite(coolLEDpin, 0);
    break;
  case WHITE_SOFT:
    analogWrite(warmLEDpin, brightness / 2);
    analogWrite(coolLEDpin, brightness / 2);
    break;
  case WHITE_COOL:
    analogWrite(warmLEDpin, 0);
    analogWrite(coolLEDpin, brightness);
    break;
  default:
    analogWrite(warmLEDpin, brightness / 2);
    analogWrite(coolLEDpin, brightness / 2);
    break;
  }
}

void setNeopixelColor(uint16_t hue, uint8_t brightness, bool force)
{
  if (!force && isSmileyModeActive())
    return;

  CRGBPalette16 palette = getPaletteForIndex(selectedPaletteIndex);

  for (uint8_t i = 0; i < NUM_LEDS; i++)
  {
    uint8_t index = (i * 255) / NUM_LEDS;
    leds[i] = ColorFromPalette(palette, index, brightness, LINEARBLEND);
  }

  FastLED.show();
}

void applyColorLighting(uint16_t hue, uint8_t brightness)
{
  analogWrite(coolLEDpin, 0);
  analogWrite(warmLEDpin, 0);
  setNeopixelColor(hue, brightness);
}

void applyPalette(uint16_t hue)
{
  CRGBPalette16 palette = getPaletteForIndex(selectedPaletteIndex);

  for (uint8_t i = 0; i < NUM_LEDS; i++)
  {
    uint8_t index = (i * 255) / NUM_LEDS;
    leds[i] = ColorFromPalette(palette, index, 255, LINEARBLEND);
  }

  FastLED.show();
}

void updatePaletteAnimation(uint16_t paletteIndex, uint8_t brightness)
{
  CRGBPalette16 palette = getPaletteForIndex(paletteIndex);

  uint16_t speedDivisor = map(brightness, 1, 255, 32, 1);

  for (uint8_t i = 0; i < NUM_LEDS; i++)
  {
    uint8_t index = inoise8(i * 50, millis() / speedDivisor);
    leds[i] = ColorFromPalette(palette, index, brightness, LINEARBLEND);
  }

  FastLED.show();
}

void clearPixels()
{
  for (uint8_t i = 0; i < NUM_LEDS; i++)
  {
    leds[i] = CRGB::Black;
  }
  FastLED.show();
}

void turnOffAllLights()
{
  analogWrite(coolLEDpin, 0);
  analogWrite(warmLEDpin, 0);
  clearPixels();
}

void renderColorMenu(uint8_t offset)
{
  for (uint8_t i = 0; i < NUM_LEDS; i++)
  {
    uint8_t hue = (i * 255 / NUM_LEDS + offset) & 0xFF;
    leds[i] = CHSV(hue, 255, 255); // Full saturation & brightness
  }
  FastLED.show();
}

void restoreLightStateFromPreset(const LampPreset &preset)
{
  smileyDisabledAfterUserInput = true;

  if (preset.useColoredLight)
  {
    applyColorLighting(preset.hue, preset.brightness);
  }
  else
  {
    setWhiteLightMode(static_cast<WhiteMode>(preset.whiteMode), preset.brightness, false);
    setNeopixelColor(hue, 0);
  }
}

void restorePreferredLighting()
{
  if (isSmileyModeActive())
  {
    applySmileyLighting();
  }
  else if (useColoredLight)
  {
    setNeopixelColor(hue, getCurrentBrightness());
  }
  else if (!useColoredLight && currentWhiteMode != 0)
  {
    clearPixels();
    FastLED.show();
    setWhiteLightMode(currentWhiteMode, getCurrentBrightness(), false);
  }
  else if ((wanderingAfterStartup && !userHasTakenControl) &&
           Config::useSmiley && !smileyDisabledAfterUserInput)
  {
    applySmileyLighting(); // Fallback for startup smiley
  }
  else
  {
    if (Config::useSmiley)
    {
      turnOffAllLights(); // Turn everything off
    }
    else
    {
      applyPresetLighting(presets[0]); // Fallback to preset
    }
  }
}

void updateSelectedPixel(uint8_t currentBrightness)
{
  const uint8_t fixedBlinkingPixel = (6 + neopixelRotationOffset) % NUM_LEDS;

  uint8_t dimmedBrightness = constrain(currentBrightness / 2, 16, currentBrightness);

  for (uint8_t i = 0; i < NUM_COLOR_OPTIONS; i++)
  {
    uint8_t displayIndex = (i + colorOffset) % NUM_COLOR_OPTIONS;
    uint8_t brightness = (i == fixedBlinkingPixel)
                             ? calculatePulseBrightness(currentBrightness)
                             : dimmedBrightness;
    CRGB color;

    if (displayIndex < NUM_CUSTOM_COLORS)
    {
      uint16_t hue = customHueRing[displayIndex];
      color = CHSV(hue / 256, 255, brightness);
    }
    else
    {
      // Whites — simulated using CHSV with low saturation
      switch (displayIndex)
      {
      case 9:
        color = CHSV(8000 / 256, 200, brightness);
        break; // Warm white
      case 10:
        color = CHSV(6000 / 256, 50, brightness);
        break; // Soft white
      case 11:
        color = CHSV(43700 / 256, 60, brightness);
        break; // Cool white
      default:
        color = CRGB::Black;
        break;
      }
    }

    uint8_t pixelIndex = (i + neopixelRotationOffset) % NUM_LEDS;
    leds[pixelIndex] = color;
  }

  // Clear unused pixels (if any)
  for (uint8_t i = NUM_COLOR_OPTIONS; i < NUM_LEDS; i++)
  {
    leds[i] = CRGB::Black;
  }
}

void applyBufferedNeopixels()
{
  for (uint8_t i = 0; i < NUM_LEDS; i++)
  {
    uint32_t color = neopixelBuffer[i];

    uint8_t r = (color >> 16) & 0xFF;
    uint8_t g = (color >> 8) & 0xFF;
    uint8_t b = color & 0xFF;

    leds[i] = CRGB(r, g, b);
  }

  FastLED.show(); // Display the buffered colors
}

uint8_t calculatePulseBrightness() // Calculate a brightness value based on a sine wave
{
  unsigned long currentTime = millis();                                            // Use the current time to create a pulsing effect
  float pulseFactor = (sin(currentTime * 0.02) + 1) / 2;                           // Create a value between 0 and 1
  uint8_t minBrightness = 30;                                                      // Minimum brightness during pulse
  uint8_t maxBrightness = 255;                                                     // Maximum brightness during pulse
  return (uint8_t)(minBrightness + (maxBrightness - minBrightness) * pulseFactor); // Interpolate between min and max brightness
}

void restoreNeopixelBuffer(const uint32_t *colorBuffer, uint8_t brightness)
{
  FastLED.setBrightness(brightness); // Global brightness for all LEDs

  for (uint8_t i = 0; i < NUM_LEDS; i++)
  {
    uint32_t color = colorBuffer[i];
    uint8_t r = (color >> 16) & 0xFF;
    uint8_t g = (color >> 8) & 0xFF;
    uint8_t b = color & 0xFF;
    leds[i] = CRGB(r, g, b);
  }

  FastLED.show();
}

uint8_t getCurrentBrightness()
{
  return pgm_read_word(&(brightnessLevels[currentBrightnessIndex]));
}

void fadeWhiteLightsToBlack(float t, WhiteMode mode, uint8_t originalBrightness)
{
  t = constrain(t, 0.0f, 1.0f);

  if (t >= 1.0f)
  {
    analogWrite(warmLEDpin, 0);
    analogWrite(coolLEDpin, 0);
    moodyWhiteLightsDisabled = true;
    return;
  }

  int warm = 0;
  int cool = 0;

  switch (mode)
  {
  case WHITE_WARM:
    warm = originalBrightness * (1.0f - t);
    break;

  case WHITE_SOFT:
    warm = (originalBrightness / 2) * (1.0f - t);
    cool = (originalBrightness / 2) * (1.0f - t);
    break;

  case WHITE_COOL:
    cool = originalBrightness * (1.0f - t);
    break;

  default:
    warm = cool = (originalBrightness / 2) * (1.0f - t);
    break;
  }

  analogWrite(warmLEDpin, warm);
  analogWrite(coolLEDpin, cool);
}

void resetWhiteFadeState()
{
  // Stop any leftover color lighting
  turnOffAllLights();

  // Reset fade flags
  whiteFadeStartTime = 0;
  fadingStarted = false;
  moodyWhiteLightsDisabled = false;
}

void captureOriginalLightState()
{
  originalUseColoredLight = useColoredLight;
  originalBrightness = getCurrentBrightness();
  originalHue = hue;

  // Capture white mode only if not using colored light
  if (!useColoredLight)
  {
    if (selectedPixel == 9)
      currentWhiteMode = WHITE_WARM;
    else if (selectedPixel == 10)
      currentWhiteMode = WHITE_SOFT;
    else if (selectedPixel == 11)
      currentWhiteMode = WHITE_COOL;
    else
      currentWhiteMode = WHITE_SOFT; // fallback

    originalWhiteMode = currentWhiteMode;
  }
}

uint16_t interpolateHue(uint16_t fromHue, uint16_t toHue, float t)
{
  int32_t delta = (int32_t)toHue - (int32_t)fromHue;

  if (abs(delta) > 32768)
  {
    // Take the short path around the hue circle
    if (delta > 0)
      delta -= 65536;
    else
      delta += 65536;
  }

  int32_t interpolated = (int32_t)fromHue + delta * t;

  if (interpolated < 0) // Wrap result to 0–65535 range
    interpolated += 65536;
  else if (interpolated > 65535)
    interpolated -= 65536;

  return (uint16_t)interpolated;
}

void indicateActivatedMood(MoodBlinkType blinkType, uint32_t color, uint8_t brightness)
{
  CRGB originalColors[NUM_LEDS];
  bool ringWasActive = false;

  for (uint8_t i = 0; i < NUM_LEDS; i++)
  {
    originalColors[i] = leds[i];
    if (leds[i] != CRGB::Black)
    {
      ringWasActive = true;
    }
  }

  uint8_t originalBrightness = FastLED.getBrightness();
  uint8_t whiteBrightness = getCurrentBrightness();
  WhiteMode previousWhiteMode = currentWhiteMode;

  turnOffAllLights();

  const unsigned long pulseDuration = 200;
  const uint8_t numPulses = 2;
  const unsigned long totalDuration = pulseDuration * 2 * numPulses;
  unsigned long startTime = millis();

  while (millis() - startTime < totalDuration)
  {
    unsigned long elapsed = millis() - startTime;
    float phase = fmod(elapsed, pulseDuration * 2) / (float)pulseDuration;
    float brightnessFactor = (phase < 1.0f) ? phase : (2.0f - phase);
    uint8_t blinkLevel = brightnessFactor * brightness;

    switch (blinkType)
    {
    case BLINK_COLOR:
    {
      CRGB blinkColor = CRGB(color);
      blinkColor.nscale8_video(blinkLevel);
      fill_solid(leds, NUM_LEDS, blinkColor);
      FastLED.show();
    }
    break;

    case BLINK_WARM:
      setWhiteLightMode(WHITE_WARM, blinkLevel, false);
      break;
    case BLINK_SOFT:
      setWhiteLightMode(WHITE_SOFT, blinkLevel, false);
      break;
    case BLINK_COOL:
      setWhiteLightMode(WHITE_COOL, blinkLevel, false);
      break;
    }
    delayMicroseconds(1000);
  }

  if (isSmileyModeActive())
  {
    showStartupSmiley();
  }
  else if (ringWasActive)
  {
    FastLED.setBrightness(originalBrightness);
    for (uint8_t i = 0; i < NUM_LEDS; i++)
    {
      leds[i] = originalColors[i];
    }
    FastLED.show();
  }
  else
  {
    restoreWhiteLightMode(previousWhiteMode, whiteBrightness);
  }
}

void restoreWhiteLightMode(WhiteMode mode, uint8_t brightness)
{
  clearPixels();
  useColoredLight = false;

  switch (mode)
  {
  case WHITE_WARM:
    analogWrite(warmLEDpin, brightness);
    analogWrite(coolLEDpin, 0);
    break;
  case WHITE_SOFT:
    analogWrite(warmLEDpin, brightness / 2);
    analogWrite(coolLEDpin, brightness / 2);
    break;
  case WHITE_COOL:
    analogWrite(warmLEDpin, 0);
    analogWrite(coolLEDpin, brightness);
    break;
  default:
    analogWrite(warmLEDpin, brightness / 2);
    analogWrite(coolLEDpin, brightness / 2);
    break;
  }
}

void startNeopixelInterpolation(uint16_t fromHue, uint8_t fromBrightness,
                                uint16_t toHue, uint8_t toBrightness,
                                uint16_t durationMs)
{
  pixelFade.active = true;
  pixelFade.startTime = millis();
  pixelFade.duration = durationMs;
  pixelFade.fromHue = fromHue;
  pixelFade.toHue = toHue;
  pixelFade.fromBrightness = fromBrightness;
  pixelFade.toBrightness = toBrightness;

  neopixelInterpolating = true;
}

void updateNeopixelInterpolation()
{
  if (!pixelFade.active)
    return;

  unsigned long now = millis();
  float t = (now - pixelFade.startTime) / (float)pixelFade.duration;
  t = constrain(t, 0.0f, 1.0f);

  uint16_t currentHue = interpolateHue(pixelFade.fromHue, pixelFade.toHue, t);
  currentBrightness = pixelFade.fromBrightness + (pixelFade.toBrightness - pixelFade.fromBrightness) * t;

  if (!isSmileyModeActive())
  {
    setNeopixelColor(currentHue, currentBrightness);
  }

  if (t >= 1.0f)
  {
    pixelFade.active = false;
  }
}

void renderColorSelectionMenu()
{
  const uint8_t numCustomColors = sizeof(customHueRing) / sizeof(customHueRing[0]);
  uint8_t baseBrightness = getCurrentBrightness();
  uint8_t dimmedBrightness = constrain(baseBrightness / 2, 16, baseBrightness);

  for (uint8_t i = 0; i < numCustomColors; i++)
  {
    uint16_t hue = customHueRing[i];
    uint8_t brightness = (i == logicalSelectedIndex) ? baseBrightness : dimmedBrightness;
    leds[i] = CHSV(hue / 256, 255, brightness);
  }
  FastLED.show();
}

#pragma endregion LED_CONTROL_UTILS

//////////////////////////////////////////////////////////////////////////////////////////////////////////
// MOTOR MOVEMENT FUNCTIONS
//////////////////////////////////////////////////////////////////////////////////////////////////////////

#pragma region motorMovementFxns
void controlServos()
{
  unsigned long now = millis();
  if (now - lastManualControlTime < manualControlInterval)
    return;
  lastManualControlTime = now;

  int16_t joystickYaw = analogRead(JoyYawPin);
  int16_t joystickPitch = analogRead(JoyPitchPin);
  int16_t deltaYaw = joystickYaw - joystickCenterYaw;
  int16_t deltaPitch = joystickPitch - joystickCenterPitch;

  bool joystickMoved = abs(deltaYaw) > deadZoneThreshold || abs(deltaPitch) > deadZoneThreshold;
  bool joystickStill = !joystickMoved;

  static bool joystickWasPreviouslyMoved = false;
  static unsigned long joystickEngageStartTime = 0;

  if (joystickMoved)
  {
    if (!joystickWasPreviouslyMoved)
    {
      joystickEngageStartTime = now;
      joystickWasPreviouslyMoved = true;
    }
    handleManualControl(now, deltaYaw, deltaPitch);
  }
  else if (joystickWasPreviouslyMoved)
  {
    userMovedHeadDeliberately = false;
    joystickWasPreviouslyMoved = false;
    unsigned long bumpDuration = now - joystickEngageStartTime;
    handleSnapbackTriggers(now, bumpDuration);
  }

  static unsigned long lastRestoreTime = 0;
  if (currentMood == MOOD_MOODY && !manualControlActive && now - lastRestoreTime > Config::delayBeforeGettingMoody && moodySnapbackCompleted)
  {
    moodySnapbackCompleted = false;
    lastRestoreTime = now;
    restoreMoodyAfterSnapback(now);
  }

  if (pendingDistractedSnap && joystickStill && now - lastJoystickMotionTime > Config::snapbackDebounceTime)
  {
    resumeOriginalPosition();
  }

  if (pendingMoodySnap && joystickStill && now - lastJoystickMotionTime > Config::snapbackDebounceTime)
  {
    isReturningFromWander = false;
    pendingMoodySnap = false;
    snapbackAbandoned = false;
    moodDrivingServos = false;
  }

  if (manualControlActive && !joystickMoved && (now - lastJoystickMotionTime > Config::userNotControllingInterval))
  {
    manualControlActive = false;

    int newYaw = servoYaw.read();
    int newPitch = servoPitch.read();

    if (headMovedSignificantly(originalHeadYaw, originalHeadPitch, newYaw, newPitch))
    {
      originalHeadYaw = newYaw;
      originalHeadPitch = newPitch;
      // Serial.println(F("[USER] Head movement significant — updating original head position."));
    }
    else
    {
      // Serial.println(F("[USER] Minor twitch — ignoring head movement."));
    }
  }
}

bool headMovedSignificantly(int oldYaw, int oldPitch, int newYaw, int newPitch, float thresholdDegrees)
{
  return abs(newYaw - oldYaw) > thresholdDegrees || abs(newPitch - oldPitch) > thresholdDegrees;
}

void handleManualControl(unsigned long now, int16_t deltaYaw, int16_t deltaPitch)
{
  // Record that manual joystick control is in progress
  manualControlActive = true;
  lastUserInputTime = now;
  lastJoystickMotionTime = now;

  // Stop all mood-based head movement
  isWandering = false;
  moodDrivingServos = false;
  isReturningFromWander = false;

  // If user hasn't taken control yet, mark that they now have
  if (!userHasTakenControl)
  {
    wanderingAfterStartup = true; // Allow wandering to resume later
    nextStartupWanderTime = now + Config::idleDelayAfterStartup;
    userHasTakenControl = true;
  }

  // If Moody snapback was pending and the joystick moved (after debounce period),
  // cancel the snapback and mark it as abandoned
  if (currentMood == MOOD_MOODY &&
      pendingMoodySnap &&
      now - lastJoystickMotionTime > Config::snapbackDebounceTime)
  {
    pendingMoodySnap = false;
    snapbackAbandoned = true;
    readyToUpdateOriginalHead = false;
    lastJoystickMotionTime = now;
  }

  // In Moody Mood: if the user has moved the joystick after a snapback restore,
  // update the original height (for next snapback target) and suppress duplicate updates
  if (currentMood == MOOD_MOODY &&
      !readyToUpdateOriginalHead &&
      moodyHeightRestoreComplete &&
      (abs(deltaYaw) > 0 || abs(deltaPitch) > 0))
  {
    originalMoodyHeight = analogRead(heightEncoderPin);
    moodyHeightRestoreComplete = false;
  }

  // Map raw delta values to servo speeds (range: ±4)
  int16_t speedYaw = map(deltaYaw, -512, 512, 4, -4);     // Inverted for correct direction
  int16_t speedPitch = map(deltaPitch, -512, 512, -4, 4); // Natural pitch direction

  // Clamp output angles to servo constraints
  int16_t angleYaw = constrain(servoYaw.read() + speedYaw, Config::servoMinYawConstraint, Config::servoMaxYawConstraint);
  int16_t anglePitch = constrain(servoPitch.read() + speedPitch, Config::servoMinPitchConstraint, Config::servoMaxPitchConstraint);

  // Write servo positions directly (not interpolated)
  servoYaw.write(angleYaw);
  servoPitch.write(anglePitch);

  // Mark that user has moved the head intentionally
  userMovedHeadDeliberately = true;

  if (currentMood == MOOD_MOODY && userMovedHeadDeliberately)
  {
    originalMoodyPitch = servoPitch.read();
    originalMoodyYaw = servoYaw.read();
  }
}

void handleSnapbackTriggers(unsigned long now, unsigned long bumpDuration)
{
  // === DISTRACTED SNAPBACK ===
  if ((currentMood == MOOD_DISTRACTED || wanderingAfterStartup) &&
      bumpDuration < 150 &&
      !isReturningFromWander &&
      !pendingDistractedSnap &&
      !snapbackAbandoned)
  {
    cancelMoodMotion();            // Stop all automated movement
    clampHeadToDistractedBounds(); // Constrain yaw/pitch within safe zone

    // Begin smooth interpolation back to original head position
    beginSnapback(now, originalHeadYaw, originalHeadPitch, INTERP_LINEAR);
    pendingDistractedSnap = true;
  }

  // === MOODY SNAPBACK ===
  else if (currentMood == MOOD_MOODY &&
           bumpDuration < 150 &&
           !isReturningFromWander &&
           !pendingMoodySnap &&
           !snapbackAbandoned)
  {
    beginSnapback(now, originalHeadYaw, originalHeadPitch, INTERP_EASE_IN_OUT);
    pendingMoodySnap = true;

    // Reset all timing and state flags for clean Moody reentry
    resetMoodyState();
    resetWhiteFadeState();

    if (originalUseColoredLight)
    {
      // Restore original hue and brightness for Neopixels
      hue = originalHue;
      setNeopixelColor(originalHue, originalBrightness);
      pixelFade.active = false;
      neopixelInterpolating = false;
      useColoredLight = true;
    }
    else
    {
      // Restore white lighting mode and ensure Neopixels are off
      restoreWhiteLightMode(currentWhiteMode, originalBrightness);
      setNeopixelColor(hue, 0); // Set pixel to black
      useColoredLight = false;
      moodyWhiteLightsDisabled = false; // Only re-enable if we’re restoring white mode
    }

    // Begin restoring original lamp height
    targetHeight = originalMoodyHeight;
    isMoving = true;
    moodyHeightRestoreComplete = false;

    // Allow the system to update stored origin after snapback completes
    readyToUpdateOriginalHead = true;

    // Reset Moody mood animation state for future cycles
    moodyStartTime = now;
    moodyInDelay = true;
    moodyLooping = false;
    moodyInTransition = false;
    moodyPaused = false;
    moodyDescending = false;
    moodyRising = false;
    moodyInitialCenterReached = false;
  }
}

void beginSnapback(unsigned long now, int16_t yaw, int16_t pitch, InterpolationMode mode)
{
  targetServoYaw = yaw;
  targetServoPitch = pitch;
  currentServoYaw = servoYaw.read();
  currentServoPitch = servoPitch.read();

  startYaw = currentServoYaw;
  startPitch = currentServoPitch;

  interpolationDuration = 600.0;
  currentInterpMode = mode;
  interpolationStartTime = 0;

  isWandering = true;
  moodDrivingServos = true;
  isReturningFromWander = true;
  snapbackAbandoned = false;
  lastUserInputTime = now;
}

void motorControl(uint8_t speed, bool direction)
{
  digitalWrite(motorDirectionPin, direction);
  analogWrite(motorSpeedPin, speed);
}

void moveServosToTarget()
{
  if ((!presetActive && !moodDrivingServos) || manualControlActive)
    return;

  float dx = targetServoYaw - currentServoYaw;
  float dy = targetServoPitch - currentServoPitch;
  float dist = sqrt(dx * dx + dy * dy);

  // Start new interpolation if not already started
  if (interpolationStartTime == 0)
  {
    interpolationStartTime = millis();
    startYaw = currentServoYaw;
    startPitch = currentServoPitch;
  }

  float t = (millis() - interpolationStartTime) / (float)interpolationDuration;
  t = constrain(t, 0.0, 1.0);
  float easedT = applyEasing(t, currentInterpMode);

  currentServoYaw = startYaw + (targetServoYaw - startYaw) * easedT;
  currentServoPitch = startPitch + (targetServoPitch - startPitch) * easedT;

  currentServoYaw = constrain(currentServoYaw, Config::servoMinYawConstraint, Config::servoMaxYawConstraint);
  currentServoPitch = constrain(currentServoPitch, Config::servoMinPitchConstraint, Config::servoMaxPitchConstraint);

  // Apply low-pass filtering
  filteredServoYaw = 0.7 * filteredServoYaw + 0.3 * currentServoYaw;
  filteredServoPitch = 0.7 * filteredServoPitch + 0.3 * currentServoPitch;

  filteredServoYaw = constrain(filteredServoYaw, 0.0f, 180.0f);
  filteredServoPitch = constrain(filteredServoPitch, 0.0f, 180.0f);

  // Convert to PWM
  int16_t pulseYaw = map((int)filteredServoYaw, 0, 180, 544, 2400);
  int16_t pulsePitch = map((int)filteredServoPitch, 0, 180, 544, 2400);

  if (pulseYaw != lastServoYawWritten)
  {
    servoYaw.writeMicroseconds(pulseYaw);
    lastServoYawWritten = pulseYaw;
  }

  if (pulsePitch != lastServoPitchWritten)
  {
    servoPitch.writeMicroseconds(pulsePitch);
    lastServoPitchWritten = pulsePitch;
  }

  // Snap to final if close enough
  if (dist < Config::moodSnapThreshold && t > 0.1)
  {
    currentServoYaw = targetServoYaw;
    currentServoPitch = targetServoPitch;
    startYaw = currentServoYaw;
    startPitch = currentServoPitch;

    interpolationStartTime = 0;

    servoYaw.write((int)targetServoYaw);
    servoPitch.write((int)targetServoPitch);
    lastServoYawWritten = (int)targetServoYaw;
    lastServoPitchWritten = (int)targetServoPitch;

    presetActive = false;
    completeServoMotion();
    return;
  }

  // Also complete if interpolation finished
  if (t >= 1.0f)
  {
    presetActive = false;
    completeServoMotion(); // PROBLEM
  }
}

void completeServoMotion()
{
  if (moodDrivingServos)
  {
    isWandering = false;
    moodDrivingServos = false;
    isReturningFromWander = false;
    readyToUpdateOriginalHead = false;

    if (currentMood == MOOD_MOODY)
    {
      if (pendingMoodySnap)
      {
        pendingMoodySnap = false;
        moodySnapbackCompleted = true; // Allow delayed reentry
        // Serial.println(F("[MOODY] Snapback completed, ready to reenter nod cycle."));
      }

      if (!moodyInitialCenterReached)
      {
        moodyInTransition = false;
        moodyInitialCenterReached = true;
        moodyDroopDown = false;
        // Serial.println(F("Moody center reached."));
      }
    }
    else if (currentMood == MOOD_DISTRACTED || wanderingAfterStartup)
    {
      if (wanderingAfterStartup && !userHasTakenControl)
      {
        nextStartupWanderTime = millis() + Config::distractedMinInterval;
      }
    }
  }
}

void moveToHeight(uint16_t targetHeight)

{
  if (isMoving) // Check if we need to adjust the height
  {
    currentHeight = analogRead(heightEncoderPin); // Read the current height value

    if (currentHeight < targetHeight) // Determine the direction and adjust the motor based on the current height and target height
    {
      motorControl(255, true); // Move up
    }
    else if (currentHeight > targetHeight)
    {
      motorControl(255, false); // Move down
    }

    if (abs(targetHeight - currentHeight) <= 15) // Stop the motor if we're within the tolerance range
    {
      motorControl(0, true); // Stop the motor
      isMoving = false;      // Mark as not moving

      if (currentMood == MOOD_MOODY && pendingMoodySnap)
      {
        moodyHeightRestoreComplete = true;
        // Serial.println(F("[MOODY] Height restoration complete — ready to accept new height"));
      }
    }
  }
}

void cancelMoodMotion()
{
  isWandering = false;
  moodDrivingServos = false;
  interpolationStartTime = 0;
}

float applyEasing(float t, InterpolationMode mode) // t ranges from 0.0 to 1.0

{
  switch (mode)
  {
  case INTERP_EASE_IN:
    return t * t; // quadratic ease-in
  case INTERP_EASE_OUT:
    return t * (2 - t); // quadratic ease-out
  case INTERP_EASE_IN_OUT:
    return (t < 0.5) ? 2 * t * t : -1 + (4 - 2 * t) * t;
  case INTERP_LINEAR:
  default:
    return t;
  }
}

float interpolateQuadraticBezier(float p0, float p1, float p2, float t)
{
  return (1 - t) * (1 - t) * p0 + 2 * (1 - t) * t * p1 + t * t * p2;
}

void resumeOriginalPosition()
{
  if (currentMood == MOOD_MOODY)
  {
    restoreMoodySnapback();
    return;
  }

  if (currentMood == MOOD_DISTRACTED || wanderingAfterStartup)
  {
    clampHeadToDistractedBounds();
  }

  if (wanderingAfterStartup && !userHasTakenControl)
  {
    restoreStartupSnapback();
    return;
  }

  if (!wanderingAfterStartup && presetActive)
  {
    smileyDisabledAfterUserInput = true;
  }

  restoreDefaultSnapback();
}

void restoreMoodySnapback()
{
  useColoredLight = originalUseColoredLight;
  hue = originalHue;

  if (useColoredLight)
  {
    setNeopixelColor(originalHue, originalBrightness);
    pixelFade.active = false;
    neopixelInterpolating = false;
  }
  else
  {
    restoreWhiteLightMode(currentWhiteMode, originalBrightness);
    setNeopixelColor(hue, 0);
  }

  restoreServoPosition(600.0, INTERP_EASE_IN_OUT);

  isWandering = true;
  moodDrivingServos = true;
  isReturningFromWander = true;
  pendingMoodySnap = false;
  manualControlActive = false;
}

void restoreStartupSnapback()
{
  applySnapbackLighting(); // Smiley or fallback

  restoreServoPosition(600.0, INTERP_LINEAR);

  isWandering = true;
  moodDrivingServos = true;
  isReturningFromWander = true;
  pendingDistractedSnap = false;
  readyToUpdateOriginalHead = false;
}

void restoreDefaultSnapback()
{
  useColoredLight = originalUseColoredLight;

  if (isSmileyModeActive())
  {
    applySmileyLighting();
  }
  else if (useColoredLight)
  {
    setNeopixelColor(originalHue, originalBrightness);
  }
  else
  {
    restoreWhiteLightMode(currentWhiteMode, originalBrightness);
    setNeopixelColor(hue, 0);
  }

  restoreServoPosition(600.0, INTERP_LINEAR);

  if ((wanderingAfterStartup && !userHasTakenControl) || currentMood == MOOD_DISTRACTED)
  {
    isWandering = true;
    moodDrivingServos = true;
    isReturningFromWander = true;
  }

  pendingDistractedSnap = false;
  manualControlActive = false;
}

void applySnapbackLighting()
{
  if (Config::useSmiley && !smileyDisabledAfterUserInput)
  {
    turnOffAllLights();
    showStartupSmiley();
  }
  else
  {
    restorePreferredLighting(); // Safely fall back
  }
}

void restoreServoPosition(float duration, InterpolationMode mode)
{
  targetServoYaw = originalHeadYaw;
  targetServoPitch = originalHeadPitch;

  currentServoYaw = servoYaw.read();
  currentServoPitch = servoPitch.read();
  startYaw = currentServoYaw;
  startPitch = currentServoPitch;

  interpolationDuration = duration;
  currentInterpMode = mode;
  interpolationStartTime = 0;
}

inline void clampHeadToDistractedBounds(int16_t &yaw, int16_t &pitch)
{
  yaw = constrain(yaw,
                  static_cast<int16_t>(Config::servoMinDistractedYawConstraint),
                  static_cast<int16_t>(Config::servoMaxDistractedYawConstraint));
  pitch = constrain(pitch,
                    static_cast<int16_t>(Config::servoMinDistractedPitchConstraint),
                    static_cast<int16_t>(Config::servoMaxDistractedPitchConstraint));
}

inline void clampHeadToExcitedBounds(int16_t &yaw, int16_t &pitch)
{
  yaw = constrain(yaw, static_cast<int16_t>(Config::excitedMinYaw), static_cast<int16_t>(Config::excitedMaxYaw));
  pitch = constrain(pitch, static_cast<int16_t>(Config::excitedMinPitch), static_cast<int16_t>(Config::excitedMaxPitch));
}

#pragma endregion motorMovementFxns

// SETUP FUNCTIONS

#pragma region setupFunctions

void initializeSerial()
{
  if (Config::useSerial)
  {
    Serial.begin(115200);
    Serial.println(F("HP LIGHT V2.0.0"));
  }
}

void initializePins()
{
  pinMode(JoyYawPin, INPUT);
  pinMode(JoyPitchPin, INPUT);
  pinMode(heightEncoderPin, INPUT);

  pinMode(coolLEDpin, OUTPUT);
  pinMode(warmLEDpin, OUTPUT);

  pinMode(buttonUpPin, INPUT_PULLUP);
  pinMode(buttonDownPin, INPUT_PULLUP);

  pinMode(motorSpeedPin, OUTPUT);     // Set the black gearmotor pins as "outputs"
  pinMode(motorDirectionPin, OUTPUT); // Set the black gearmotor pins as "outputs"
}

void initializeServos()
{
  servoYaw.attach(servoYawPWMPin);
  servoPitch.attach(servoPitchPWMPin);
  delay(500); // Allow time for servos to reach center
}

void initializeStartupLighting()
{
  clearPixels();
  analogWrite(coolLEDpin, 0);
  analogWrite(warmLEDpin, 0);
  hue = Config::startupSmileyHue;
}

void initializeHeightEncoder()
{
  currentHeight = analogRead(heightEncoderPin);
  targetHeight = currentHeight;
}

void initializeButtons()
{
  button1.setClickMs(700);
  button2.setClickMs(700);
  button3.setClickMs(700);
  button4.setClickMs(700);

  button1.setPressMs(900);
  downButton.setPressMs(50);
  upButton.setPressMs(50);
  joyButton.setPressMs(3000); // 3s for factory reset

  attachColorSelectionHandler();
  attachPresetShortcuts();
  attachMoodSelectionShortcuts();
  attachMotorButtons();
}

void attachColorSelectionHandler()
{
  joyButton.attachClick([]()
                        {
    userHasTakenControl = true;
    moodyWhiteLightsDisabled = false;

    if (!colorSelectionMode)
    {
      // Enter color selection mode
      turnOffAllLights();

      originalServoYaw = servoYaw.read();
      originalServoPitch = servoPitch.read();

      targetServoYaw = Config::menuYaw;
      targetServoPitch = Config::menuPitch;

      currentServoYaw = servoYaw.read();
      currentServoPitch = servoPitch.read();
      startYaw = currentServoYaw;
      startPitch = currentServoPitch;

      interpolationDuration = 600.0;
      interpolationStartTime = 0;
      currentInterpMode = INTERP_EASE_IN_OUT;

      moodDrivingServos = true;
      isWandering = true;

      colorSelectionMode = true;
      smileyDisabledAfterUserInput = true;

      isPaletteAnimationEnabled = false;
      renderColorSelectionMenu();  // static wheel rendering
    }
    else
    {
      // Confirm selection and exit color selection mode
      uint8_t confirmedBrightness = getCurrentBrightness();
      updateCurrentBrightnessIndex(confirmedBrightness);

      if (logicalSelectedIndex < NUM_CUSTOM_COLORS)
      {
        // Safe: only index customHueRing when we're in the palette section
        uint16_t confirmedHue = customHueRing[logicalSelectedIndex];

        isPaletteAnimationEnabled = true;
        setNeopixelColor(confirmedHue, confirmedBrightness);
        useColoredLight = true;
        hue = confirmedHue;
        selectedPaletteIndex = logicalSelectedIndex;
      }
      else
      {
        useColoredLight = false;
        isPaletteAnimationEnabled = false;
        setNeopixelColor(0, 0);

        switch (logicalSelectedIndex)
        {
          case 9:
            setWhiteLightMode(WHITE_WARM, confirmedBrightness, true);
            currentWhiteMode = WHITE_WARM;
            break;
          case 10:
            setWhiteLightMode(WHITE_SOFT, confirmedBrightness, true);
            currentWhiteMode = WHITE_SOFT;
            break;
          case 11:
            setWhiteLightMode(WHITE_COOL, confirmedBrightness, true);
            currentWhiteMode = WHITE_COOL;
            break;
        }

        originalUseColoredLight = false;
        smileyDisabledAfterUserInput = true;
        wanderingAfterStartup = false;
      }

      isWandering = false;
      moodDrivingServos = false;
      wanderingAfterStartup = false;
      
      originalUseColoredLight = useColoredLight;
      originalBrightness = getCurrentBrightness();
      originalHue = useColoredLight ? hue : 0;

      targetServoYaw = originalServoYaw;
      targetServoPitch = originalServoPitch;

      currentServoYaw = servoYaw.read();
      currentServoPitch = servoPitch.read();
      startYaw = currentServoYaw;
      startPitch = currentServoPitch;

      interpolationDuration = 600.0;
      interpolationStartTime = 0;
      currentInterpMode = INTERP_EASE_IN_OUT;

      moodDrivingServos = true;
      isWandering = true;

      selectedPixel = logicalSelectedIndex;
      colorSelectionMode = false;

      lastUserInputTime = millis();
      lastJoystickMotionTime = millis();

      if (currentMood == MOOD_MOODY)
      {
        moodyInTransition = false;
        moodyInitialCenterReached = false;
        moodyTransitionPhase = 0;
        moodDrivingServos = false;
        isWandering = false;
      }
    } });
}

void handleColorSelectionInput()
{
  static unsigned long lastBrightnessChangeTime = 0;
  static unsigned long lastWheelRotationTime = 0;

  int16_t joystickYaw = analogRead(JoyYawPin) - joystickCenterYaw;
  int16_t joystickPitch = analogRead(JoyPitchPin) - joystickCenterPitch;
  unsigned long now = millis();

  // --- Rotate the color wheel (left/right on joystick) ---
  if (abs(joystickYaw) > deadZoneThreshold)
  {
    if (now - lastWheelRotationTime > 200) // debounce wheel turn
    {
      if (joystickYaw > deadZoneThreshold)
      {
        // rotate one way
        logicalSelectedIndex = wrapIndex((int16_t)logicalSelectedIndex - 1, NUM_COLOR_OPTIONS);
        colorOffset = wrapIndex((int16_t)colorOffset - 1, NUM_COLOR_OPTIONS);
      }
      else
      {
        // rotate the other way
        logicalSelectedIndex = wrapIndex((int16_t)logicalSelectedIndex + 1, NUM_COLOR_OPTIONS);
        colorOffset = wrapIndex((int16_t)colorOffset + 1, NUM_COLOR_OPTIONS);
      }

      lastWheelRotationTime = now;
    }
  }

  // --- Adjust brightness (up/down on joystick) ---
  if (abs(joystickPitch) > deadZoneThreshold)
  {
    if (now - lastBrightnessChangeTime > 300) // debounce brightness
    {
      if (joystickPitch < -deadZoneThreshold && currentBrightnessIndex < 4)
      {
        currentBrightnessIndex++; // Increase brightness
      }
      else if (joystickPitch > deadZoneThreshold && currentBrightnessIndex > 0)
      {
        currentBrightnessIndex--; // Decrease brightness
      }

      lastBrightnessChangeTime = now;
    }
  }
}

uint8_t calculatePulseBrightness(uint8_t baseBrightness)
{
  // Pulse from 60% to 100% of baseBrightness over ~1.5s
  float pulseSpeed = 0.002f; // adjust to make it faster/slower
  float phase = millis() * pulseSpeed;
  float sine = (sin(phase * TWO_PI) + 1.0f) / 2.0f; // range 0.0–1.0
  float pulse = 0.6f + (0.4f * sine);               // range 0.6–1.0
  return (uint8_t)(baseBrightness * pulse);
}

void attachPresetShortcuts()
{
  button1.attachClick([]()
                      {
    if (button1.getNumberClicks() == 1)
    {
      smileyDisabledAfterUserInput = true;
      moodyWhiteLightsDisabled = false;
      applyPreset(0);
      syncGlobalsFromPreset(0);
    } });

  button1.attachLongPressStart([]()
                               { saveCurrentValuesToPreset(0); });

  button2.attachClick([]()
                      {
    if (button2.getNumberClicks() == 1)
    {
      smileyDisabledAfterUserInput = true;
      moodyWhiteLightsDisabled = false;
      applyPreset(1);
      syncGlobalsFromPreset(1);
    } });

  button2.attachLongPressStart([]()
                               { saveCurrentValuesToPreset(1); });

  button3.attachClick([]()
                      {
    if (button3.getNumberClicks() == 1)
    {
      smileyDisabledAfterUserInput = true;
      moodyWhiteLightsDisabled = false;
      applyPreset(2);
      syncGlobalsFromPreset(2);
    } });

  button3.attachLongPressStart([]()
                               { saveCurrentValuesToPreset(2); });

  button4.attachClick([]()
                      {
    if (button4.getNumberClicks() == 1)
    {
      smileyDisabledAfterUserInput = true;
      moodyWhiteLightsDisabled = false;
      applyPreset(3);
      syncGlobalsFromPreset(3);
    } });

  button4.attachLongPressStart([]()
                               { saveCurrentValuesToPreset(3); });

  joyButton.attachLongPressStart([]()
                                 {
                                   indicateActivatedMood(BLINK_SOFT);

                                   presets[0] = {90, 90, 180, false, 128, 230, WHITE_WARM};
                                   presets[1] = {70, 110, 300, true, 220, 8000, 0};
                                   presets[2] = {130, 80, 480, true, 200, 42000, 0};
                                   presets[3] = {90, 155, 560, true, 128, 62000, 0};
                                   savePresetsToEEPROM();

                                   // if (Config::useSerial)
                                   // {
                                   //   Serial.println(F("Factory presets restored."));
                                   // }
                                 });
}

void attachMoodSelectionShortcuts()
{
  button1.attachDoubleClick([]()
                            { setMoodDistracted(false);
                              userSelectedMood = true; });
  button2.attachDoubleClick([]()
                            { setMoodHappy(false);
                              userSelectedMood = true; });
  button3.attachDoubleClick([]()
                            { setMoodMoody(false);
                              userSelectedMood = true; });
  button4.attachDoubleClick([]()
                            { setMoodFocused(false);
                              userSelectedMood = true; });
}

void attachMotorButtons()
{
  downButton.attachDuringLongPress([]()
                                   {
    if (!colorSelectionMode)
    {
      motorControl(255, false); // Move downward
    } });

  downButton.attachLongPressStop([]()
                                 {
                                   motorControl(0, true); // Stop motor (braking enabled)
                                   originalMoodyHeight = analogRead(heightEncoderPin);
                                   // Serial.println(F("New original height set at "));
                                   // Serial.println(originalMoodyHeight);
                                 });

  upButton.attachDuringLongPress([]()
                                 {
    if (!colorSelectionMode)
    {
      motorControl(255, true); // Move upward
    } });

  upButton.attachLongPressStop([]()
                               {
                                 motorControl(0, true); // Stop motor (braking enabled)
                                 originalMoodyHeight = analogRead(heightEncoderPin);
                                 // Serial.println(F("New original height set at "));
                                 // Serial.println(originalMoodyHeight);
                               });
}

void initializeEEPROM()
{
  if (!loadPresetsFromEEPROM())
  {
    presets[0] = {105, 160, 365, false, 128, 230};
    presets[1] = {70, 110, 670, true, 220, 0};
    presets[2] = {130, 80, 800, true, 200, 90};
    presets[3] = {50, 120, 720, true, 128, 255};
    savePresetsToEEPROM(); // write defaults once if user has never set any of their own presets
  }
}

void calibrateJoystick()
{
  const uint8_t calibrationSamples = 50;
  long totalX = 0, totalY = 0;

  for (uint8_t i = 0; i < calibrationSamples; i++)
  {
    totalX += analogRead(JoyYawPin);
    totalY += analogRead(JoyPitchPin);
    delay(10); // Allow time between readings
  }

  joystickCenterYaw = totalX / calibrationSamples;
  joystickCenterPitch = totalY / calibrationSamples;
}

void initializeInterpolationDefaults()
{
  currentInterpMode = INTERP_LINEAR;
  interpolationStartTime = 0; // Reset any leftover interpolation
  interpolationDuration = 1200.0;
}

#pragma endregion setupFunctions

//////////////////////////////////////////////////////////////////////////////////////////////////////////
// BUTTON FUNCTIONS
//////////////////////////////////////////////////////////////////////////////////////////////////////////

#pragma region buttonFunctions

void tickAllButtons()
{
  joyButton.tick();
  button1.tick();
  button2.tick();
  button3.tick();
  button4.tick();
  downButton.tick();
  upButton.tick();
}

void saveCurrentValuesToPreset(uint8_t presetIndex)
{
  if (presetIndex >= 4)
    return;

  // Save current state
  presets[presetIndex].yawPosition = servoYaw.read();
  presets[presetIndex].pitchPosition = servoPitch.read();
  presets[presetIndex].height = analogRead(heightEncoderPin);
  presets[presetIndex].useColoredLight = useColoredLight;
  presets[presetIndex].brightness = getCurrentBrightness();

  if (useColoredLight)
  {
    presets[presetIndex].hue = customHueRing[logicalSelectedIndex];
    presets[presetIndex].whiteMode = 0;
  }
  else
  {
    presets[presetIndex].hue = 0;
    switch (logicalSelectedIndex)
    {
    case 9:
      presets[presetIndex].whiteMode = WHITE_WARM;
      break;
    case 10:
      presets[presetIndex].whiteMode = WHITE_SOFT;
      break;
    case 11:
      presets[presetIndex].whiteMode = WHITE_COOL;
      break;
    default:
      presets[presetIndex].whiteMode = WHITE_SOFT;
      break;
    }
  }

  for (uint8_t i = 0; i < 2; i++) // Blink confirmation (2 flashes)
  {
    if (useColoredLight)
    {
      setNeopixelColor(presets[presetIndex].hue, presets[presetIndex].brightness);
    }
    else
    {
      restoreWhiteLightMode(static_cast<WhiteMode>(presets[presetIndex].whiteMode),
                            presets[presetIndex].brightness);
    }

    delay(150);
    turnOffAllLights();
    delay(150);
  }

  restoreLightStateFromPreset(presets[presetIndex]); // Restore current light state
  savePresetsToEEPROM();                             // Commit to EEPROM
}

void applyPreset(uint8_t presetIndex)
{
  if (presetIndex >= 4)
    return;

  initializePresetState();
  calculatePresetInterpolation(presets[presetIndex]);
  applyPresetLighting(presets[presetIndex]);
  updateCurrentBrightnessIndex(presets[presetIndex].brightness);
}

void initializePresetState()
{
  smileyDisabledAfterUserInput = true;
  wanderingAfterStartup = false;
  isWandering = false;
  userHasTakenControl = true;
  manualControlActive = false;
  presetActive = true;
}

void calculatePresetInterpolation(const LampPreset &preset)
{
  targetServoYaw = preset.yawPosition;
  targetServoPitch = preset.pitchPosition;

  currentServoYaw = servoYaw.read();
  currentServoPitch = servoPitch.read();

  targetHeight = preset.height;
  isMoving = true;
  currentHeight = analogRead(heightEncoderPin);

  uint16_t heightDelta = abs((int)targetHeight - (int)currentHeight);
  float estimatedHeightDuration = (heightDelta / 200.0f) * 1000.0f;

  float dx = targetServoYaw - currentServoYaw;
  float dy = targetServoPitch - currentServoPitch;
  float servoDistance = sqrt(dx * dx + dy * dy);
  float estimatedServoDuration = (servoDistance / 80.0f) * 1000.0f;

  interpolationDuration = constrain(
      max(estimatedHeightDuration, estimatedServoDuration),
      500.0f, 5000.0f);

  interpolationStartTime = 0;
}

void applyPresetLighting(const LampPreset &preset)
{
  useColoredLight = preset.useColoredLight;
  originalUseColoredLight = preset.useColoredLight;
  originalBrightness = preset.brightness;

  if (useColoredLight)
  {
    analogWrite(coolLEDpin, 0);
    analogWrite(warmLEDpin, 0);
    hue = preset.hue;
    originalHue = preset.hue;
    setNeopixelColor(preset.hue, preset.brightness);
  }
  else
  {
    setNeopixelColor(hue, 0);
    switch (preset.whiteMode)
    {
    case WHITE_WARM:
      analogWrite(coolLEDpin, 0);
      analogWrite(warmLEDpin, preset.brightness);
      break;
    case WHITE_SOFT:
      analogWrite(coolLEDpin, preset.brightness / 2);
      analogWrite(warmLEDpin, preset.brightness / 2);
      break;
    case WHITE_COOL:
      analogWrite(coolLEDpin, preset.brightness);
      analogWrite(warmLEDpin, 0);
      break;
    default:
      analogWrite(coolLEDpin, preset.brightness / 2);
      analogWrite(warmLEDpin, preset.brightness / 2);
      break;
    }
  }
}

void updateCurrentBrightnessIndex(uint8_t brightness)
{
  for (uint8_t i = 0; i < 5; i++)
  {
    if (pgm_read_word(&(brightnessLevels[i])) == brightness)
    {
      currentBrightnessIndex = i;
      break;
    }
  }
}

void syncGlobalsFromPreset(uint8_t presetIndex)
{
  if (presetIndex >= 4)
    return;

  useColoredLight = presets[presetIndex].useColoredLight;
  hue = presets[presetIndex].hue;

  if (!useColoredLight)
  {
    switch (presets[presetIndex].whiteMode)
    {
    case WHITE_WARM:
      selectedPixel = 9;
      break;
    case WHITE_SOFT:
      selectedPixel = 10;
      break;
    case WHITE_COOL:
      selectedPixel = 11;
      break;
    default:
      selectedPixel = 10;
      break;
    }
  }

  for (uint8_t i = 0; i < 5; i++)
  {
    if (pgm_read_word(&(brightnessLevels[i])) == presets[presetIndex].brightness)
    {
      currentBrightnessIndex = i;
      break;
    }
  }
}

#pragma endregion buttonFunctions

//////////////////////////////////////////////////////////////////////////////////////////////////////////
// MOOD FUNCTIONS
//////////////////////////////////////////////////////////////////////////////////////////////////////////

#pragma region moodFunctions

void setMoodDistracted(bool silent)
{
  currentMood = MOOD_DISTRACTED;
  userHasTakenControl = true;

  bool comingFromStartupSmiley = (wanderingAfterStartup && !userHasTakenControl && Config::useSmiley && !smileyDisabledAfterUserInput);
  captureOriginalLightState();
  wanderingAfterStartup = false;
  if (!originalUseColoredLight && !comingFromStartupSmiley) // Don't disable the smiley unless light was explicitly chosen
  {
    smileyDisabledAfterUserInput = true;
  }

  if (!silent)
  {
    indicateActivatedMood(BLINK_COLOR, customHueRing[1]); // Orange
  }

  originalHeadYaw = servoYaw.read();
  originalHeadPitch = servoPitch.read();
  clampHeadToDistractedBounds(originalHeadYaw, originalHeadPitch);

  isWandering = false;
  lastWanderTime = millis();
  nextWanderInterval = random(Config::distractedMinInterval, Config::distractedMaxInterval);

  currentInterpMode = INTERP_EASE_IN_OUT;
  interpolationStartTime = 0; // reset interpolator cleanly

  restorePreferredLighting();
}

void updateDistractedMood()
{
  unsigned long now = millis();
  if (wanderingAfterStartup && now < nextStartupWanderTime)
    return;

  updateDistractedHeadOriginIfIdle(now);
  triggerDistractedWanderIfReady(now);
}

void updateDistractedHeadOriginIfIdle(unsigned long now)
{
  if ((currentMood == MOOD_DISTRACTED || wanderingAfterStartup) &&
      !manualControlActive && userMovedHeadDeliberately &&
      !isWandering && !moodDrivingServos &&
      (now - lastJoystickMotionTime > Config::userNotControllingInterval) &&
      !readyToUpdateOriginalHead)
  {
    originalHeadYaw = servoYaw.read();
    originalHeadPitch = servoPitch.read();
    readyToUpdateOriginalHead = true;
    userMovedHeadDeliberately = false;

    clampHeadToDistractedBounds();

    isWandering = false;
    moodDrivingServos = false;
    lastWanderTime = now;
    nextWanderInterval = random(Config::distractedMinInterval, Config::distractedMaxInterval);
  }
}

void triggerDistractedWanderIfReady(unsigned long now)
{
  if (!isWandering && now - lastWanderTime > nextWanderInterval)
  {
    targetServoYaw = constrain(originalHeadYaw + random(-Config::wanderRange, Config::wanderRange + 1),
                               Config::servoMinDistractedYawConstraint, Config::servoMaxDistractedYawConstraint);
    targetServoPitch = constrain(originalHeadPitch + random(-Config::wanderRange, Config::wanderRange + 1),
                                 Config::servoMinDistractedPitchConstraint, Config::servoMaxDistractedPitchConstraint);

    beginInterpolatedWander(now, 3000, 8000, Config::distractedMinInterval, Config::distractedMaxInterval);
  }
}

void beginInterpolatedWander(unsigned long now, unsigned long durationMin, unsigned long durationMax,
                             unsigned long intervalMin, unsigned long intervalMax)
{
  currentServoYaw = servoYaw.read();
  currentServoPitch = servoPitch.read();
  startYaw = currentServoYaw;
  startPitch = currentServoPitch;

  interpolationDuration = random(durationMin, durationMax);
  currentInterpMode = INTERP_EASE_IN_OUT;
  interpolationStartTime = 0;

  isWandering = true;
  moodDrivingServos = true;

  lastWanderTime = now;
  nextWanderInterval = random(intervalMin, intervalMax);
}

void setMoodHappy(bool silent)
{
  currentMood = MOOD_HAPPY;
  userSelectedMood = true;
  currentAnimationType = ANIM_EXCITED;

  wanderingAfterStartup = false;

  if (!silent)
  {
    indicateActivatedMood(BLINK_COLOR, customHueRing[2]); // Yellow
  }

  smileyDisabledAfterUserInput = false;
  if (isSmileyModeActive())
  {
    applySmileyLighting();
  }

  turnOffAllLights();
  if (isSmileyModeActive())
  {
    showStartupSmiley();
  }

  originalHeadYaw = servoYaw.read();
  originalHeadPitch = servoPitch.read();
  clampHeadToExcitedBounds(originalHeadYaw, originalHeadPitch);

  currentExcitedIndex = random(0, 2); // pick one of your animation banks
  loadExcitedAnimation(currentExcitedIndex);

  lastExcitedChangeTime = millis();
  nextExcitedInterval = random(Config::excitedMinInterval, Config::excitedMaxInterval);

  excitedLightOverridden = true;
}

void updateHappyMood()
{
  unsigned long now = millis();

  updateSmileyLightingIfNeeded();
  updateSmileyBlinkAndShiftIfEnabled();
  updateExcitedHeadOriginIfIdle(now);
  triggerExcitedWanderIfReady(now);
  handleHeightBounceCycle(now);
  maybeTriggerExcitedAnimation(now);
}

void updateSmileyLightingIfNeeded()
{
  if (!excitedLightOverridden && Config::useSmiley && !smileyDisabledAfterUserInput)
  {
    applySmileyLighting();
    excitedLightOverridden = true;
  }
}

void updateSmileyBlinkAndShiftIfEnabled()
{
  if (!smileyDisabledAfterUserInput && Config::useSmiley)
  {
    updateSmileyBlinkAndShift();
  }
}

void updateExcitedHeadOriginIfIdle(unsigned long now)
{
  if (!manualControlActive &&
      now - lastJoystickMotionTime > Config::userNotControllingInterval &&
      !readyToUpdateOriginalHead)
  {
    originalHeadYaw = servoYaw.read();
    originalHeadPitch = servoPitch.read();
    readyToUpdateOriginalHead = true;
    clampHeadToExcitedBounds(originalHeadYaw, originalHeadPitch);
  }
}

void triggerExcitedWanderIfReady(unsigned long now)
{
  if (!isWandering && now - lastWanderTime > nextWanderInterval)
  {
    clampHeadToExcitedBounds(originalHeadYaw, originalHeadPitch);
    targetServoYaw = constrain(originalHeadYaw + random(-Config::wanderRange, Config::wanderRange + 1),
                               Config::excitedMinYaw, Config::excitedMaxYaw);
    targetServoPitch = constrain(originalHeadPitch + random(-Config::wanderRange, Config::wanderRange + 1),
                                 Config::excitedMinPitch, Config::excitedMaxPitch);

    currentServoYaw = servoYaw.read();
    currentServoPitch = servoPitch.read();
    startYaw = currentServoYaw;
    startPitch = currentServoPitch;

    interpolationDuration = random(Config::excitedInterpolationTimeMin, Config::excitedInterpolationTimeMax);
    currentInterpMode = INTERP_EASE_IN_OUT;
    interpolationStartTime = 0;

    isWandering = true;
    moodDrivingServos = true;

    lastWanderTime = now;
    nextWanderInterval = random(Config::excitedWanderMinInterval, Config::excitedWanderMaxInterval);
  }
}

void handleHeightBounceCycle(unsigned long now)
{
  if (excitedHeightBounceActive)
  {
    if (now - excitedHeightBounceStart > excitedHeightBounceDuration)
    {
      targetHeight = excitedOriginalHeight;
      isMoving = true;
      excitedHeightBounceActive = false;
    }
    return; // don't allow other animations during bounce
  }
}

void maybeTriggerExcitedAnimation(unsigned long now)
{
  if (!playingStartupAnimation &&
      !excitedHeightBounceActive &&
      now - lastExcitedChangeTime > nextExcitedInterval)
  {
    lastExcitedChangeTime = now;
    nextExcitedInterval = random(Config::excitedMinInterval, Config::excitedMaxInterval);

    uint8_t newIndex = random(0, 3); // 0–1 = keyframes, 2 = bounce
    if (newIndex == 2)
    {
      excitedOriginalHeight = analogRead(heightEncoderPin);
      targetHeight = Config::excitedMaxHeight;
      isMoving = true;

      excitedHeightBounceStart = now;
      excitedHeightBounceDuration = random(Config::excitedMinStandupInterval, Config::excitedMaxStandupInterval);
      excitedHeightBounceActive = true;
    }
    else
    {
      currentExcitedIndex = newIndex;
      loadExcitedAnimation(currentExcitedIndex);
    }
  }
}

void setMoodMoody(bool silent)
{
  currentMood = MOOD_MOODY;
  userHasTakenControl = true;

  // Cancel startup idle state and smiley
  wanderingAfterStartup = false;
  cancelMoodMotion();
  smileyDisabledAfterUserInput = true;

  hasRestoredMoodyOnce = false;
  moodyStartedInterpToCenter = false;

  if (!silent)
  {
    indicateActivatedMood(BLINK_COLOR, customHueRing[7]); // Purple
  }

  // Capture original state
  originalHeadYaw = servoYaw.read();
  originalHeadPitch = servoPitch.read();
  originalMoodyHeight = analogRead(heightEncoderPin);
  originalUseColoredLight = useColoredLight;
  originalBrightness = getCurrentBrightness();
  originalHue = hue;
  originalWhiteMode = currentWhiteMode;

  // Reset flags
  moodyStartTime = millis();
  moodyInDelay = true;
  moodyInTransition = false;
  moodyInitialCenterReached = false;
  moodyLooping = false;
  moodyPaused = false;
  moodyDescending = false;
  moodyRising = false;
  pendingMoodySnap = false;
}

void updateMoodyMood(unsigned long now)
{
  if (moodyInDelay)
  {
    handleMoodyDelayPhase(now);
    return;
  }

  if (!whiteLightsAreOff)
  {
    fadeOutWhiteLightsDuringTransition(now);

    // Check time-based or value-based fade completion
    if (now - whiteFadeStartTime >= Config::moodyInterpTime || currentBrightness <= 1)
    {
      if (!originalUseColoredLight)
        turnOffAllLights(); // Optional: safety kill if we came from white mode

      whiteLightsAreOff = true;

      // if (Config::useSerial)
      //   Serial.println(F("[MOODY] White lights faded out. Proceeding to nod logic."));
    }

    // Block further progression until fade is done
    return;
  }

  beginMoodyNodCycleIfCentered(now);

  // if (Config::useSerial)
  //   Serial.println(F("[MOODY] ← Finished beginMoodyNodCycleIfCentered()"));

  handleMoodyNodCycle(now);

  // if (Config::useSerial)
  //   Serial.println(F("[MOODY] ← Finished handleMoodyNodCycle()"));
}

void handleMoodyDelayPhase(unsigned long now)
{
  if (now - moodyStartTime >= Config::delayBeforeGettingMoody && !moodyStartedInterpToCenter)
  {
    moodyInDelay = false;
    moodyInTransition = true;
    moodyStartedInterpToCenter = true;

    // Begin slow head move to center
    targetServoYaw = 90;
    targetServoPitch = Config::moodyRiseTargetY;
    interpolationDuration = Config::moodyInterpTime;
    interpolationStartTime = 0; // Let the servo interpolation loop initialize the start time
    presetActive = true;
    moodDrivingServos = true;

    // Start Neopixel fade-in
    startNeopixelInterpolation(originalHue, originalBrightness, Config::moodyHueBlue, 100, Config::moodyInterpTime);
    hue = Config::moodyHueBlue;
    useColoredLight = true;

    moodyPhaseStartTime = now;
  }
}

void fadeOutWhiteLightsDuringTransition(unsigned long now)
{
  if (currentMood == MOOD_MOODY &&
      !originalUseColoredLight &&
      moodyInTransition)
  {
    if (!fadingStarted)
    {
      whiteFadeStartTime = now;
      fadingStarted = true;
    }

    float t = (now - whiteFadeStartTime) / (float)Config::whiteFadeDuration;

    if (t <= 1.0f)
    {
      fadeWhiteLightsToBlack(t, originalWhiteMode, originalBrightness);
    }
    else
    {
      resetWhiteFadeState();
      whiteLightsAreOff = true;
    }
  }
  else
  {
    resetWhiteFadeState();
  }
}

void beginMoodyNodCycleIfCentered(unsigned long now)
{
  if (!moodyLooping && moodyInitialCenterReached)
  {
    moodyInTransition = false;
    moodyLooping = true;
    moodyDescending = true;

    targetServoYaw = 90;
    targetServoPitch = Config::moodyDroopTargetY;
    interpolationDuration = Config::headDescendTime;
    interpolationStartTime = 0;
    presetActive = true;
    moodDrivingServos = true;

    targetHeight = Config::moodySlumpTarget;
    isMoving = true;

    startNeopixelInterpolation(Config::moodyHueBlue, 100, Config::moodyHuePurple, 100, Config::headDescendTime);
    moodyPhaseStartTime = now;

    // Serial.println(F("[MOODY] Transition complete, starting nod cycle: descending"));
  }
}

void handleMoodyNodCycle(unsigned long now)
{
  if (!moodyLooping)
    return;

  // (A) Finished nodding down
  if (moodyDescending && now - moodyPhaseStartTime >= Config::headDescendTime)
  {
    moodyDescending = false;
    moodyPaused = true;
    lastMoodyPauseTime = now;
    // Serial.println(F("[MOODY] Nod down complete → pausing"));
    return;
  }

  // (B) Finished pause → rise back up
  if (moodyPaused && now - lastMoodyPauseTime >= Config::headHangTime)
  {
    moodyPaused = false;
    moodyRising = true;

    targetServoYaw = 90;
    targetServoPitch = Config::moodyRiseTargetY;
    interpolationDuration = Config::headAscendTime;
    interpolationStartTime = 0;
    presetActive = true;
    moodDrivingServos = true;
    currentInterpMode = INTERP_EASE_IN_OUT;

    startNeopixelInterpolation(Config::moodyHuePurple, 100, Config::moodyHueBlue, 100, Config::headAscendTime);
    moodyPhaseStartTime = now;

    // Serial.println(F("[MOODY] Starting nod rise"));
    return;
  }

  // (C) Finished nod up → restart nod down
  if (moodyRising && now - moodyPhaseStartTime >= Config::headAscendTime)
  {
    moodyRising = false;
    moodyDescending = true;

    targetServoYaw = 90;
    targetServoPitch = Config::moodyDroopTargetY;
    interpolationDuration = Config::headDescendTime;
    interpolationStartTime = 0;
    presetActive = true;
    moodDrivingServos = true;
    currentInterpMode = INTERP_EASE_IN_OUT;

    startNeopixelInterpolation(Config::moodyHueBlue, 100, Config::moodyHuePurple, 100, Config::headDescendTime);
    moodyPhaseStartTime = now;

    // Serial.println(F("[MOODY] Restarting nod cycle: descending"));
    return;
  }
}

void setMoodFocused(bool silent)
{
  currentMood = MOOD_FOCUSED;
  userSelectedMood = true;
  userHasTakenControl = true;

  bool comingFromStartupSmiley = (wanderingAfterStartup && !userHasTakenControl && Config::useSmiley && !smileyDisabledAfterUserInput);

  captureOriginalLightState();

  wanderingAfterStartup = false;

  if (!originalUseColoredLight && !comingFromStartupSmiley) // Don't disable the smiley unless light was explicitly chosen

  {
    smileyDisabledAfterUserInput = true;
  }

  if (!silent)
  {
    indicateActivatedMood(BLINK_COLOR, customHueRing[4]);
  }

  isWandering = false;
  moodDrivingServos = false;

  lastWanderTime = 0;
  nextWanderInterval = 0;

  targetServoYaw = servoYaw.read();
  targetServoPitch = servoPitch.read();
  currentServoYaw = targetServoYaw;
  currentServoPitch = targetServoPitch;

  if (Config::useSmiley && !smileyDisabledAfterUserInput)
  {
    applySmileyLighting();
  }
}

void handleMoodIdleTrigger(unsigned long now)
{
  // Static tracking for state-change logging
  static MoodState lastMood = static_cast<MoodState>(-1);
  static bool lastMoodyInTransition = false;
  static bool lastMoodDrivingServos = false;
  static bool lastUserMovedHeadDeliberately = false;
  static bool lastManualControlActive = false;
  static bool lastReadyToUpdateOriginalHead = false;

  bool moodChanged = (currentMood != lastMood ||
                      moodyInTransition != lastMoodyInTransition ||
                      moodDrivingServos != lastMoodDrivingServos ||
                      userMovedHeadDeliberately != lastUserMovedHeadDeliberately ||
                      manualControlActive != lastManualControlActive ||
                      readyToUpdateOriginalHead != lastReadyToUpdateOriginalHead);

  if (moodChanged)
  {
    // Update last-tracked values
    lastMood = currentMood;
    lastMoodyInTransition = moodyInTransition;
    lastMoodDrivingServos = moodDrivingServos;
    lastUserMovedHeadDeliberately = userMovedHeadDeliberately;
    lastManualControlActive = manualControlActive;
    lastReadyToUpdateOriginalHead = readyToUpdateOriginalHead;
  }

  // === Main logic ===

  if (shouldUpdateIdle())
  {
    if (currentMood == MOOD_DISTRACTED || wanderingAfterStartup)
    {
      handleDistractedIdle(now);
    }
    else if (currentMood == MOOD_HAPPY)
    {
      handleHappyIdle(now);
    }
    else if (currentMood == MOOD_MOODY)
    {
      handleMoodyIdle(now);
    }
  }

  if (shouldUpdateDistractedSnapback(now))
    updateDistractedSnapbackOrigin(now);

  if (shouldBeginFocusedStartupWander(now))
    beginFocusedStartupWander(now);

  // Moody Mood snapback recovery
  if (shouldRestoreMoodyAfterSnapback(now) && !hasRestoredMoodyOnce)
  {
    if (!moodyInTransition && !moodyLooping)
    {
      moodyInitialCenterReached = false;
      setMoodMoody(true);
      readyToUpdateOriginalHead = false; // ✅ prevents infinite loop
      hasRestoredMoodyOnce = true;
    }
  }
}

bool shouldUpdateIdle()
{
  return !colorSelectionMode && !manualControlActive &&
         (millis() - lastUserInputTime > Config::moodIdleDelay);
}

void handleDistractedIdle(unsigned long now)
{
  updateDistractedMood();
}

void handleHappyIdle(unsigned long now)
{
  updateHappyMood();
}

void handleMoodyIdle(unsigned long now)
{
  updateMoodyMood(now);
}

bool shouldRestoreMoodyAfterSnapback(unsigned long now)
{
  return currentMood == MOOD_MOODY &&
         !manualControlActive &&
         !moodDrivingServos &&
         (now - lastJoystickMotionTime > Config::userNotControllingInterval) &&
         (now - lastUserInputTime > Config::moodIdleDelay) &&
         readyToUpdateOriginalHead;
}

void restoreMoodyAfterSnapback(unsigned long now)
{
  originalHeadYaw = servoYaw.read();
  originalHeadPitch = servoPitch.read();
  readyToUpdateOriginalHead = true;
  userMovedHeadDeliberately = false;

  // Start moody fade-in transition (blue → purple)
  moodyInTransition = true;
  moodyInitialCenterReached = false;
  moodyTransitionStartTime = now;
  moodyTransitionPhase = 0; // triggers phase 0: current → blue

  // Setup servo movement back to center
  targetServoYaw = 90;
  targetServoPitch = Config::moodyRiseTargetY;

  currentServoYaw = servoYaw.read();
  currentServoPitch = servoPitch.read();
  startYaw = currentServoYaw;
  startPitch = currentServoPitch;

  interpolationDuration = 8000.0;
  currentInterpMode = INTERP_EASE_IN_OUT;
  interpolationStartTime = 0;

  moodDrivingServos = true;
  isWandering = true;
}

void resetMoodyState()
{
  moodyInTransition = false;
  moodyInitialCenterReached = false;
  moodyLooping = false;
  moodyPaused = false;
  moodyDescending = false;
  moodyRising = false;
  moodyWhiteLightsDisabled = false;
  moodySnapbackCompleted = false;
  moodyHeightRestoreComplete = false;
  whiteLightsAreOff = false;
  hasRestoredMoodyOnce = false;
  moodyInDelay = true;
  fadingStarted = false; // problem?
}

bool shouldUpdateDistractedSnapback(unsigned long now)
{
  return (currentMood == MOOD_DISTRACTED || wanderingAfterStartup) &&
         !manualControlActive &&
         userMovedHeadDeliberately &&
         !isWandering && !moodDrivingServos &&
         (now - lastJoystickMotionTime > Config::userNotControllingInterval) &&
         !readyToUpdateOriginalHead;
}

void updateDistractedSnapbackOrigin(unsigned long now)
{
  originalHeadYaw = servoYaw.read();
  originalHeadPitch = servoPitch.read();
  readyToUpdateOriginalHead = true;
  userMovedHeadDeliberately = false;

  clampHeadToDistractedBounds(originalHeadYaw, originalHeadPitch);

  isWandering = false;
  moodDrivingServos = false;
  lastWanderTime = now;
  nextWanderInterval = random(Config::distractedMinInterval, Config::distractedMaxInterval);
}

bool shouldBeginFocusedStartupWander(unsigned long now)
{
  return (wanderingAfterStartup && !userHasTakenControl) &&
         now > nextStartupWanderTime &&
         !manualControlActive &&
         !isWandering &&
         currentMood == MOOD_FOCUSED;
}

void beginFocusedStartupWander(unsigned long now)
{
  {
    if (!readyToUpdateOriginalHead)
    {
      originalHeadYaw = servoYaw.read();
      originalHeadPitch = servoPitch.read();
      readyToUpdateOriginalHead = true;
      clampHeadToDistractedBounds(originalHeadYaw, originalHeadPitch);
    }

    targetServoYaw = constrain(
        originalHeadYaw + random(-Config::wanderRange, Config::wanderRange + 1),
        Config::servoMinDistractedYawConstraint,
        Config::servoMaxDistractedYawConstraint);

    targetServoPitch = constrain(
        originalHeadPitch + random(-Config::wanderRange, Config::wanderRange + 1),
        Config::servoMinDistractedPitchConstraint,
        Config::servoMaxDistractedPitchConstraint);

    currentServoYaw = servoYaw.read();
    currentServoPitch = servoPitch.read();
    startYaw = currentServoYaw;
    startPitch = currentServoPitch;

    interpolationDuration = random(3000, 8000);
    currentInterpMode = INTERP_EASE_IN_OUT;
    interpolationStartTime = 0;

    isWandering = true;
    moodDrivingServos = true;

    nextStartupWanderTime = now + random(Config::distractedMinInterval, Config::distractedMaxInterval);
  }
}

#pragma endregion moodFunctions

//////////////////////////////////////////////////////////////////////////////////////////////////////////
// EEPROM FUNCTIONS
//////////////////////////////////////////////////////////////////////////////////////////////////////////

#pragma region EEPROM functions

void savePresetsToEEPROM()
{
  for (uint8_t i = 0; i < 4; i++)
    EEPROM.put(i * sizeof(LampPreset), presets[i]);
  EEPROM.write(EEPROM_MAGIC_ADDR, EEPROM_MAGIC);
}

bool loadPresetsFromEEPROM()
{
  if (EEPROM.read(EEPROM_MAGIC_ADDR) != EEPROM_MAGIC)
  {
    return false;
  }

  for (uint8_t i = 0; i < 4; i++)
    EEPROM.get(i * sizeof(LampPreset), presets[i]);

  return true;
}

#pragma endregion EEPROM functions

//////////////////////////////////////////////////////////////////////////////////////////////////////////
// ANIMATIONS
//////////////////////////////////////////////////////////////////////////////////////////////////////////

#pragma region Animations

void loadStartupAnimation()
{
  currentAnimationType = ANIM_STARTUP;

  if (!Config::useSmiley)
  {
    applyPresetLighting(presets[0]);
  }

  for (uint8_t i = 0; i < startupAnimationRawLength; i++) // Copy from PROGMEM to RAM
  {
    memcpy_P(&startupAnimation[i], &startupAnimationRaw[i], sizeof(Keyframe));
    clampKeyframe(startupAnimation[i]);
  }

  const int16_t baseHeight = presets[0].height; // Optionally override height with actual value from preset[0]

  for (uint8_t i = 0; i < startupAnimationRawLength; i++)
  {
    if (startupAnimation[i].height == 425)
    {
      startupAnimation[i].height = baseHeight + 60;
    }
    else if (startupAnimation[i].height == 365)
    {
      startupAnimation[i].height = baseHeight;
    }
  }

  startupKeyframeCount = startupAnimationRawLength;
  currentKeyframeIndex = 0;
  keyframeStartTime = millis();
  playingStartupAnimation = true;

  if (Config::useSmiley)
  {
    analogWrite(coolLEDpin, 0);
    analogWrite(warmLEDpin, 0);
    showStartupSmiley();
  }
}

void clampKeyframe(Keyframe &kf)
{
  kf.yaw = constrain(kf.yaw, Config::servoMinYawConstraint, Config::servoMaxYawConstraint);
  kf.pitch = constrain(kf.pitch, Config::servoMinPitchConstraint, Config::servoMaxPitchConstraint);
  kf.controlYaw = constrain(kf.controlYaw, Config::servoMinYawConstraint, Config::servoMaxYawConstraint);
  kf.controlPitch = constrain(kf.controlPitch, Config::servoMinPitchConstraint, Config::servoMaxPitchConstraint);
}

void playKeyframeAnimation()
{
  if (Config::useSmiley && !smileyDisabledAfterUserInput && currentAnimationType == ANIM_STARTUP)
  {
    handleSmileyDuringStartup();
  }

  if (currentKeyframeIndex >= startupKeyframeCount)
  {
    finishStartupAnimation();
    captureOriginalLightState();
    return;
  }

  updateCurrentKeyframeAnimation();
}

void handleSmileyDuringStartup()
{
  if (currentServoPitch < 75 && !eyesSuppressed)
  {
    eyesSuppressed = true;

    // Turn off LEDs 4 through 8
    for (uint8_t i = 4; i <= 8; i++)
    {
      leds[i] = CRGB::Black;
    }

    FastLED.show();
  }
  else if (currentServoPitch >= 75 && eyesSuppressed)
  {
    eyesSuppressed = false;
    showStartupSmiley(); // assumed to also use FastLED
  }
}

void finishStartupAnimation()
{
  playingStartupAnimation = false;
  eyesSuppressed = false;

  if (!userSelectedMood)
  {
    if (Config::useSmiley)
    {
      // Smiley on, lamp idles, then wanders
      setMoodDistracted(true);
      wanderingAfterStartup = true;
      nextStartupWanderTime = millis() + Config::idleDelayAfterStartup;
      smileyDisabledAfterUserInput = false;
    }
    else
    {
      syncGlobalsFromPreset(0);
      applyPreset(0);
      smileyDisabledAfterUserInput = true;

      switch (Config::startupMood)
      {
      case MOOD_HAPPY:
        setMoodHappy(true);
        break;
      case MOOD_MOODY:
        setMoodMoody(true);
        break;
      case MOOD_DISTRACTED:
        setMoodDistracted(true);
        wanderingAfterStartup = true;
        nextStartupWanderTime = millis() + Config::idleDelayAfterStartup;
        break;
      case MOOD_FOCUSED:
        setMoodFocused(true);
        break;
      }
    }
  }

  // Show smiley if appropriate
  if (Config::useSmiley && !smileyDisabledAfterUserInput && !userSelectedMood)
  {
    showStartupSmiley();
  }
}

void updateCurrentKeyframeAnimation()
{
  Keyframe &kf = startupAnimation[currentKeyframeIndex];
  unsigned long now = millis();
  float t = constrain((now - keyframeStartTime) / (float)kf.duration, 0.0f, 1.0f);
  float easedT = applyEasing(t, static_cast<InterpolationMode>(kf.easing));

  if (kf.height != -1)
  {
    int interpolatedHeight = startHeight + (kf.height - startHeight) * easedT;
    targetHeightDuringAnimation = interpolatedHeight;
    moveToHeight(targetHeightDuringAnimation);
    if (abs(interpolatedHeight - analogRead(heightEncoderPin)) > 10)
    {
      isMoving = true;
    }
  }

  if (kf.useBezier)
  {
    currentServoYaw = interpolateQuadraticBezier(startYaw, kf.controlYaw, kf.yaw, easedT);
    currentServoPitch = interpolateQuadraticBezier(startPitch, kf.controlPitch, kf.pitch, easedT);
  }
  else
  {
    currentServoYaw = startYaw + (kf.yaw - startYaw) * easedT;
    currentServoPitch = startPitch + (kf.pitch - startPitch) * easedT;
  }

  servoYaw.write((int)currentServoYaw);
  servoPitch.write((int)currentServoPitch);

  if (t >= 1.0)
  {
    currentKeyframeIndex++;
    {
      startYaw = startupAnimation[currentKeyframeIndex - 1].yaw;
      startPitch = startupAnimation[currentKeyframeIndex - 1].pitch;
      startHeight = analogRead(heightEncoderPin);
      keyframeStartTime = now;
    }
  }
}

void loadExcitedAnimation(uint8_t index)
{
  currentAnimationType = ANIM_EXCITED;

  if (index >= 3)
    return;

  bool smileyShouldBeShown = // Check if smiley should be shown
      Config::useSmiley &&
      !smileyDisabledAfterUserInput &&
      (currentMood == MOOD_HAPPY || currentMood == MOOD_DISTRACTED ||
       currentMood == MOOD_FOCUSED || currentMood == MOOD_MOODY);

  if (smileyShouldBeShown) // Suppress all white LEDs under the smiley
  {
    analogWrite(coolLEDpin, 0);
    analogWrite(warmLEDpin, 0);
    showStartupSmiley();
  }
  else
  {
    if (useColoredLight) // Fallback: restore current lighting
    {
      if (!isSmileyModeActive())
      {
        setNeopixelColor(hue, getCurrentBrightness());
      }
    }
    else
    {
      setWhiteLightMode(static_cast<WhiteMode>(selectedPixel), getCurrentBrightness(), false);
    }
  }

  // Load keyframes from excitedAnimations[index] into startupAnimation[]
  startupKeyframeCount = excitedKeyframeCounts[index];
  for (uint8_t i = 0; i < startupKeyframeCount; i++)
  {
    startupAnimation[i] = excitedAnimations[index][i];
  }

  currentKeyframeIndex = 0;
  keyframeStartTime = millis();
  playingStartupAnimation = true;

  // Smiley mode overrides yellow lighting — so skip any fade to yellow
  excitedLightOverridden = true; // Still block reversion after animation
}

void showStartupSmiley()
{
  const uint8_t mouthIndices[] = {0, 1, 11};
  uint32_t colorVal = getSmileyColor();

  uint8_t r = (colorVal >> 16) & 0xFF;
  uint8_t g = (colorVal >> 8) & 0xFF;
  uint8_t b = colorVal & 0xFF;
  CRGB smiley = CRGB(r, g, b);

  // Clear all pixels
  for (uint8_t i = 0; i < NUM_LEDS; i++)
  {
    leds[i] = CRGB::Black;
  }

  // Eyes
  if (currentSmileyDirection == LOOK_LEFT)
  {
    leds[5] = smiley;
    leds[8] = smiley;
    leds[2] = CRGB::Black;
    leds[10] = smiley;
  }
  else
  {
    leds[4] = smiley;
    leds[7] = smiley;
    leds[2] = smiley;
    leds[10] = CRGB::Black;
  }

  // Mouth
  for (uint8_t i = 0; i < sizeof(mouthIndices); i++)
  {
    leds[mouthIndices[i]] = smiley;
  }

  FastLED.show();
}

void updateSmileyIfActive()
{
  bool isStartupSmiley = Config::useSmiley && !smileyDisabledAfterUserInput && wanderingAfterStartup;
  bool isExcitedSmiley = Config::useSmiley && !smileyDisabledAfterUserInput && currentMood == MOOD_HAPPY;

  if (isSmileyModeActive() || isStartupSmiley || isExcitedSmiley)
  {
    updateSmileyBlinkAndShift();
  }
}

void updateSmileyBlinkAndShift()
{
  unsigned long now = millis();
  uint32_t colorVal = getSmileyColor();
  CRGB smileyColor = CRGB((colorVal >> 16) & 0xFF, (colorVal >> 8) & 0xFF, colorVal & 0xFF);

  // --- Handle Blink ---
  if (smileyBlinkInProgress)
  {
    if (now - smileyBlinkStart >= 160)
    {
      // Restore eyes after blink
      if (currentSmileyDirection == LOOK_LEFT)
      {
        leds[5] = smileyColor;
        leds[8] = smileyColor;
      }
      else
      {
        leds[4] = smileyColor;
        leds[7] = smileyColor;
      }

      FastLED.show();
      smileyBlinkInProgress = false;
      lastSmileyBlinkTime = now;
      nextSmileyBlinkDelay = random(3000, 5000);
    }
  }
  else if (now - lastSmileyBlinkTime >= nextSmileyBlinkDelay)
  {
    // Start blink
    if (currentSmileyDirection == LOOK_LEFT)
    {
      leds[5] = CRGB::Black;
      leds[8] = CRGB::Black;
    }
    else
    {
      leds[4] = CRGB::Black;
      leds[7] = CRGB::Black;
    }

    FastLED.show();
    smileyBlinkStart = now;
    smileyBlinkInProgress = true;
  }

  // --- Handle Eye Shift ---
  if (smileyShiftInProgress)
  {
    if (now - smileyShiftStart >= 3000)
    {
      currentSmileyDirection = LOOK_LEFT;
      showStartupSmiley(); // Redraw face
      smileyShiftInProgress = false;
      lastSmileyShiftTime = now;
      nextSmileyShiftDelay = random(8000, 13000);
    }
  }
  else if (now - lastSmileyShiftTime >= nextSmileyShiftDelay)
  {
    currentSmileyDirection = LOOK_RIGHT;
    showStartupSmiley(); // Redraw face
    smileyShiftStart = now;
    smileyShiftInProgress = true;
  }
}

uint32_t getSmileyColor()
{
  uint16_t smileyHue = 7282;      // Default to yellow
  uint8_t smileySaturation = 255; // Full color by default

  if (currentAnimationType == ANIM_STARTUP)
  {
    smileyHue = Config::startupSmileyHue;
    smileySaturation = 255;
  }
  else if (useColoredLight)
  {
    smileyHue = hue;
    smileySaturation = 255;
  }
  else
  {
    switch (logicalSelectedIndex)
    {
    case 9:
      smileyHue = 8000;
      smileySaturation = 200;
      break; // Warm white
    case 10:
      smileyHue = 6000;
      smileySaturation = 50;
      break; // Soft white
    case 11:
      smileyHue = 43700;
      smileySaturation = 60;
      break; // Cool white
    default:
      smileyHue = 7282;
      smileySaturation = 255;
      break;
    }
  }

  uint8_t smileyHue8 = (smileyHue * 255UL) / 65535UL;
  CRGB rgb = CHSV(smileyHue8, smileySaturation, 255);
  return ((uint32_t)rgb.r << 16) | ((uint32_t)rgb.g << 8) | rgb.b;
}

void finishStartupWithoutAnimation()
{
  if (!userSelectedMood)
  {
    currentMood = static_cast<MoodState>(Config::startupMood);

    switch (currentMood)
    {
    case MOOD_HAPPY:
      setMoodHappy(true);
      break;
    case MOOD_MOODY:
      setMoodMoody(true);
      break;
    case MOOD_DISTRACTED:
      wanderingAfterStartup = true;
      nextStartupWanderTime = millis() + Config::idleDelayAfterStartup;
      break;
    case MOOD_FOCUSED:
      setMoodFocused(true);
      break;
    }
    if (Config::useSmiley)
    {
      smileyDisabledAfterUserInput = false;
    }
  }

  // Move head to preset[0] yaw/pitch
  currentServoYaw = servoYaw.read();
  currentServoPitch = servoPitch.read();

  startYaw = currentServoYaw;
  startPitch = currentServoPitch;

  targetServoYaw = presets[0].yawPosition;
  targetServoPitch = presets[0].pitchPosition;

  interpolationDuration = 1200.0;
  interpolationStartTime = 0;
  currentInterpMode = INTERP_EASE_IN_OUT;

  moodDrivingServos = true;
  isWandering = true;

  // If both smiley and startup animation are disabled, set light from preset
  if (!Config::useSmiley && !Config::useStartupAnim)
  {
    applyPresetLighting(presets[0]);
  }

  // Show smiley only if allowed and not overridden by user input
  if (Config::useSmiley && !smileyDisabledAfterUserInput && !userSelectedMood)
  {
    showStartupSmiley();
  }
}

bool isSmileyModeActive()
{
  return Config::useSmiley &&
         !smileyDisabledAfterUserInput &&
         (wanderingAfterStartup || currentMood == MOOD_FOCUSED || currentMood == MOOD_DISTRACTED || currentMood == MOOD_MOODY || currentMood == MOOD_HAPPY) &&
         !presetActive &&
         !colorSelectionMode;
}

void applySmileyLighting()
{
  useColoredLight = true;
  analogWrite(coolLEDpin, 0);
  analogWrite(warmLEDpin, 0);
  if (!smileyDisabledAfterUserInput)
  {
    showStartupSmiley();
  }
}

void updateFrameBasedMovement(unsigned long now)
{
  static unsigned long lastFrameTime = 0;

  if (now - lastFrameTime >= frameInterval)
  {
    if (!colorSelectionMode)
    {
      controlServos();
    }
    else
    {
      applyBufferedNeopixels();
    }

    moveToHeight(targetHeight);
    if (colorSelectionMode)
    {
      updateSelectedPixel(getCurrentBrightness());
    }
    moveServosToTarget();

    lastFrameTime = now;
  }
}

void updateStartupKeyframeAnimation()
{
  if (playingStartupAnimation)
  {
    playKeyframeAnimation();
  }
}

static inline uint8_t wrapIndex(int16_t v, uint8_t mod)
{
  v %= mod;
  if (v < 0)
    v += mod;
  return (uint8_t)v;
}

#pragma endregion Animations
#pragma endregion FUNCTIONS

//////////////////////////////////////////////////
//  END CODE  //
//////////////////////////////////////////////////