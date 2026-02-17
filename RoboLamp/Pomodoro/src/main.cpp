#pragma region README
/*
 ************************************************************************************
 * Robolamp Stock Code
 * 02.04.2026
 * Version: 1.1.0
 * Author: Crunchlabs LLC
 *
 *   This firmware powers Hack 4 for the Crunchlabs RoboLamp: Pomodoro Mode! This hack is all
 *   about EFFICIENCY. The Pomodoro Timer is a productivity tool that helps you structure work
 *   sessions and breaks. Access this mode by double-clicking the joystick, and exit it by
 *   single-clicking the joystick. A double-click of the joystick inside the mode skips the
 *   current working/break session. You can set your timings in the config file, and the lamp
 *   will visually indicate when it's time to focus and when it's time to take a break. During
 *   focus sessions, the lamp will a countdown using red lights. When time's up, it will give
 *   a little blink, then signal a break with a green countdown. After a preset amount of
 *   cycles, you'll get a longer break in blue. And then the pattern will repeat! A simple and
 *   effective tool for staying on task, but also for rewarding yourself with well-deserved breaks.
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

#include <Arduino.h>            // Core Arduino library
#include <avr/pgmspace.h>       // For storing data in PROGMEM to save RAM
#include <Adafruit_NeoPixel.h>  // Great Neopixel library that disables interrupts in a way that disrupts most servo libraries.
#include <OneButton.h>          // Button handling library
#include <Adafruit_TiCoServo.h> // Servo library compatible with Adafruit_Neopixel.h
#include <EEPROM.h>             // For saving/loading presets to/from EEPROM
#include "Types.h"              // Custom types used in the project
#include "Config.h"             // Configuration settings
#include "Animations.h"         // Keyframe animation definitions

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
OneButton downButton(A1, true); // Down button handler (A1 = D15)
OneButton upButton(A2, true);   // Up button handler (A2 = D16)
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

#pragma endregion PIN DEFINITIONS

//////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
LIBRARY ASSIGNMENTS
*/
//////////////////////////////////////////////////////////////////////////////////////////////////////////

#pragma region LIBRARY ASSIGNMENTS

Adafruit_TiCoServo servoYaw;                                                                // Yaw servo object
Adafruit_TiCoServo servoPitch;                                                              // Pitch servo object
const uint8_t numPixels = 12;                                                               // Number of neopixels in lamp ring
Adafruit_NeoPixel pixels = Adafruit_NeoPixel(numPixels, neopixelPin, NEO_GRB + NEO_KHZ800); // Neopixel object

#pragma endregion LIBRARY ASSIGNMENTS

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
GLOBAL VARIABLES
*/
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#pragma region GLOBAL VARIABLES

// LED CONTROL

#pragma region LED Control

const uint8_t neopixelRotationOffset = 6;         // Which neopixel in the ring counts as the top middle one (not necessarily the first in the soldered sequence)
bool useColoredLight = true;                      // Tracks whether or not colored Neopixels are being used
uint16_t hue = 0;                                 // Stores current hue value for LEDs
uint8_t logicalSelectedIndex = 0;                 // Global variable to keep track of the logical color selection
uint8_t currentBrightness = 128;                  // Initialize to mid-bright-brightness (brightness affects the human eye on a non-linear scale)
uint8_t brightnessLevelCount = 5;                 // Number of brightness levels available
unsigned long lastNeopixelUpdate = 0;             // Store the last time Neopixels were updated
const unsigned long neopixelUpdateInterval = 100; // LED update interval in milliseconds
uint32_t neopixelBuffer[numPixels];               // Buffer for storing Neopixel colors
uint8_t selectedPixel = 0;                        // The current selected neopixel
const uint8_t dimPixel = 10;                      // Brightness level for non-selected pixels during color selection (used to visually de-emphasize inactive options)
bool neopixelInterpolating = false;               // Flag for resetting our color interpolation function
bool startupHueLocked = false;                    // Prevents multiple random hue rolls
bool fadingStarted = false;                       // Tracks whether a white light fade has started

// Calculate brightness levels using (approximately) exponential scaling
// Using 5 levels from 28-255, but distributed (more-or-less) exponentially
const uint8_t brightnessLevels[] PROGMEM = {
    30,  // Very dim (Under this value we start to lose color data)
    40,  // Dim (40, a little higher than 32 = sqrt(255 * 0.04))
    64,  // Medium (64 = sqrt(255 * 0.16))    ~16% perceived
    128, // Bright (128 = sqrt(255 * 0.64))   ~64% perceived
    255  // Maximum (255 = sqrt(255 * 1.0))   100% perceived
};

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

NeopixelInterpolator pixelFade;

// SMILEY VARIABLES

SmileyEyeDirection currentSmileyDirection = LOOK_LEFT; // Current direction the smiley eyes are looking

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
bool pendingDistractedSnap = false; // True if a bump-to-return ("snapback") is queued in distracted or moody modes

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

// Target and Current Head Angles
int16_t targetServoYaw = 90;    // Target yaw angle for autonomous or preset-driven head motion
int16_t targetServoPitch = 90;  // Target pitch angle for autonomous or preset-driven head motion
int16_t currentServoYaw = 90;   // Current interpolated yaw position of the lamp head
int16_t currentServoPitch = 90; // Current interpolated pitch position of the lamp head

// Original (pre-animation) Head Angles
int16_t originalServoYaw = 90;   // Yaw angle at the start of a mood or preset (used for return-to-origin)
int16_t originalServoPitch = 90; // Pitch angle at the start of a mood or preset (used for return-to-origin)

// Interpolation Start Values
int16_t startYaw = 90;   // Yaw position at the start of an interpolation (used for easing)
int16_t startPitch = 90; // Pitch position at the start of an interpolation (used for easing)

// Lamp Height Control
uint16_t currentHeight = 0; // Current height of the lamp (read from encoder)
uint16_t targetHeight = 0;  // Target height during autonomous transitions (e.g. excited mode bounce)

// Servo Output Tracking
int16_t lastServoYawWritten = -1;   // Last written yaw PWM value (used to avoid redundant writes)
int16_t lastServoPitchWritten = -1; // Last written pitch PWM value (used to avoid redundant writes)

// Output Filtering (Smooth Motion)
float filteredServoYaw = 90;   // Low-pass filtered yaw output to suppress jitter
float filteredServoPitch = 90; // Low-pass filtered pitch output to suppress jitter

// User Input / Bump Detection
bool userMovedHeadDeliberately = false;    // True if joystick input was intentional (not noise)
const uint16_t snapbackBumpThreshold = 50; // Joystick magnitude threshold to trigger a bump return

#pragma endregion Servo Control

//////////////////////////////////////////////////////////////////////////////////////////////////////////
// JOYSTICK CONTROLS
//////////////////////////////////////////////////////////////////////////////////////////////////////////

#pragma region Joystick Control

int16_t joystickCenterYaw = 512;         // Default center for Yaw-axis
int16_t joystickCenterPitch = 512;       // Default center for Pitch-axis
const uint8_t deadZoneThreshold = 100;   // Define dead zone threshold
unsigned long lastManualControlTime = 0; // Tracks last time a user deliberately controlled the joystick
// `manualControlInterval` moved to `Config.h` so users can tune responsiveness

#pragma endregion Joystick Control

#pragma Interpolation Controls

//////////////////////////////////////////////////////////////////////////////////////////////////////////
// PID AND INTERPOLATION CONTROLS
//////////////////////////////////////////////////////////////////////////////////////////////////////////

float integralYaw = 0;    // Integral term for yaw PID controller
float integralPitch = 0;  // Integral term for pitch PID controller
float previousErrorX = 0; // Previous error for yaw PID controller
float previousErrorY = 0; // Previous error for pitch PID controller

InterpolationMode currentInterpMode = INTERP_LINEAR; // Default interpolation mode

//////////////////////////////////////////////////////////////////////////////////////////////////////////

AnimationType currentAnimationType = ANIM_STARTUP; // Current animation type being played

const uint8_t MAX_KEYFRAMES = 13;         // Max possible number of keyframes in an animation. Can be increased at the cost of memory
Keyframe activeKeyframes[MAX_KEYFRAMES];  // Animation array
uint8_t activeKeyframeCount = 0;          // For tracking count of keyframe animations
uint8_t currentKeyframeIndex = 0;         // For tracking the current keyframe we're on
unsigned long keyframeStartTime = 0;      // For tracking the start time of any given keyframe animation
bool isKeyframePlaybackActive = false;    // For tracking whether or not we're currently playing an animation
uint16_t startHeight = 0;                 // For tracking the starting height of the lamp head
uint16_t targetHeightDuringAnimation = 0; // For tracking the target height during an animation

const unsigned long frameInterval = 5;      // Interval (ms) between frame-based movement updates
unsigned long interpolationStartTime = 0;   // Timestamp when the current interpolation began
unsigned long interpolationDuration = 1500; // Duration (ms) of the current interpolation

#pragma endregion Interpolation Controls

#pragma region Mood Management

//////////////////////////////////////////////////////////////////////////////////////////////////////////
// MOOD CONSTANTS
// General Mood Variables
WhiteMode currentWhiteMode = WHITE_SOFT; // Current white light mode

// Distracted Variables
unsigned long lastJoystickMotionTime = 0;  // Last time the joystick was moved
int16_t originalHeadYaw = 90;              // Original yaw position for distracted mood
int16_t originalHeadPitch = 90;            // Original pitch position for distracted mood
unsigned long lastWanderTime = 0;          // Last time a wander motion was performed
unsigned long nextWanderInterval = 0;      // Time until the next wander motion
bool isWandering = false;                  // True if currently performing a wander motion
bool distractedHasReturnedToOrigin = true; // True if the head has returned to its original position
bool readyToUpdateOriginalHead = false;    // True if we should update the original head position after idle
bool moodDrivingServos = false;            // True if the mood is currently controlling the servos
bool isReturningFromWander = false;        // True if returning to original position after wander
bool userSelectedMood = false;             // True if the user has selected a mood

// Excited Variables
unsigned long nextExcitedInterval = 0;         // Time until the next excited wander motion
unsigned long lastExcitedChangeTime = 0;       // Last time an excited motion was performed
Keyframe excitedAnimations[2][MAX_KEYFRAMES];  // Two sets of excited animations
uint8_t excitedKeyframeCounts[6];              // Number of keyframes in each animation
uint16_t excitedOriginalHue = 0;               // Original hue before excited mood
uint8_t currentExcitedIndex = 0;               // Current excited animation index
uint8_t excitedOriginalBrightness = 0;         // Original brightness before excited mood
uint8_t excitedOriginalWhiteMode = WHITE_SOFT; // Original white mode before excited mood
bool excitedOriginalUseColoredLight = true;    // Original light mode before excited mood
bool excitedLightOverridden = false;           // True if the excited mood has overridden the lighting

// Excited Mood: Height Bounce Timing
bool excitedHeightBounceActive = false;        // True if the lamp is currently doing a "height bounce" motion
unsigned long excitedHeightBounceStart = 0;    // Time when the height bounce began
unsigned long excitedHeightBounceDuration = 0; // Duration of the current bounce cycle
uint16_t excitedOriginalHeight = 0;            // Height to return to after bounce completes

// Moody Constants
uint8_t originalBrightness = 128;           // Original brightness before moody mood
uint16_t originalHue = 0;                   // Original hue before moody mood
unsigned long moodyTransitionStartTime = 0; // Time when the moody transition began
const uint16_t moodyPhaseDuration = 4000;   // 4 seconds per phase
uint8_t moodyTransitionPhase = 0;           // 0 = fade out white, 1 = fade in blue
bool pendingMoodySnap = false;              // True if a bump-to-return ("snapback") is queued in moody mode
bool moodyDroopDown = true;                 // True if the moody head is currently drooping down
bool moodyInTransition = false;             // True if currently in transition to moody mood
bool moodyInitialCenterReached = false;     // True if the moody head has reached center position during transition
bool whiteLightsAreOff = false;             // True if white lights have been faded out
unsigned long whiteFadeStartTime = 0;       // Time when white light fade started

// Saved original state
uint8_t originalMoodyYaw;     // Original yaw before moody mood
uint8_t originalMoodyPitch;   // Original pitch before moody mood
uint16_t originalMoodyHeight; // Original height before moody mood
bool originalUseColoredLight; // Original light mode before moody mood
WhiteMode originalWhiteMode;  // Original white mode before moody mood

// Moody timing
unsigned long moodyStartTime = 0;      // Time when moody mood started
unsigned long moodyPhaseStartTime = 0; // Time when the current moody phase started
bool moodyInDroop = false;             // True if currently in droop phase
bool moodyInPause = false;             // True if currently in pause phase

// Moody Mood State
bool moodyInDelay = false;           // True if in initial delay phase
bool moodyDescending = false;        // True if the head is currently descending
bool moodyPaused = false;            // True if the head is currently paused at the bottom
bool moodyRising = false;            // True if the head is currently rising back up
bool moodyLooping = false;           // True if the moody nod cycle is active
bool moodySnapbackCompleted = false; // True if the moody snapback has completed

bool hasRestoredMoodyOnce = false;       // True if we've restored moody position at least once
bool moodyStartedInterpToCenter = false; // True if we've started interpolating to center during moody transition

unsigned long lastMoodyPauseTime = 0;     // Time when the last moody pause started
unsigned long lastMoodyLoopResetTime = 0; // Time when the last moody loop reset occurred
bool moodyHeightRestoreComplete = false;  // True if the height has been restored after moody nodding
bool moodyWhiteLightsDisabled = false;    // True if white lights have been disabled during moody mood

MoodState currentMood = MOOD_FOCUSED; // Start in default "focused" mood

#pragma endregion Mood Management

//////////////////////////////////////////////////////////////////////////////////////////////////////////
// EEPROM VARIABLES

#pragma region EEPROM

const uint16_t EEPROM_MAGIC_ADDR = 100; // Address to store magic byte
const byte EEPROM_MAGIC = 0xA5;         // Magic byte to verify EEPROM validity

#pragma endregion EEPROM

#pragma region Hack 4: Pomodoro Timer
// --------------------------------------------------------------------
// HACK 4: Pomodoro Timer
// --------------------------------------------------------------------
bool pomodoroModeActive = false;
bool pomodoroOnBreak = false;
uint8_t pomodoroCompletedWorkSessions = 0;

unsigned long pomodoroStartTime = 0;
unsigned long pomodoroIntervalDuration = 0;

// Saved state so we can restore cleanly when Pomodoro exits
MoodState pomodoroPrevMood = MOOD_DISTRACTED;
bool pomodoroPrevSmileyDisabled = false;
bool pomodoroPrevUserSelectedMood = false;
bool pomodoroPrevPresetActive = false;
bool pomodoroPrevColorSelectionMode = false;
bool pomodoroCurrentBreakIsLong = false;
unsigned long pomodoroLastSkipMs = 0;

#pragma endregion Hack 4 : Pomodoro Timer

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
void initializeSerial();                // Initializes serial monitor if useSerial = true in Config.h
void initializePins();                  // Sets pin modes for all used pins
void initializeServos();                // Attaches servos and sets initial positions
void initializeHeightEncoder();         // Sets up the height encoder reading
void initializeButtons();               // Sets up button handlers
void attachColorSelectionHandler();     // Attaches joystick button for color selection mode
void attachPresetShortcuts();           // Attaches long-press handlers for preset buttons
void attachMoodSelectionShortcuts();    // Attaches double-click handlers for mood selection
void attachMotorButtons();              // Attaches up/down button handlers
void initializeEEPROM();                // Prepares EEPROM and loads presets if valid
void calibrateJoystick();               // Calibrates joystick center positions
void initializeInterpolationDefaults(); // Sets default interpolation parameters
void seedRandomIfNeeded();              // Seeds random number generator if needed
void applyStartupLightingFromConfig();  // Applies startup lighting based on configuration
uint16_t pickStartupHue();              // Picks a random startup hue

// Presets
void applyPreset(uint8_t presetIndex);                       // Applies the specified preset
void initializePresetState();                                // Initializes preset states at startup
void calculatePresetInterpolation(const LampPreset &preset); // Calculates interpolation parameters for preset application
void applyPresetLighting(const LampPreset &preset);          // Applies lighting settings from a preset
void updateCurrentBrightnessIndex(uint8_t brightness);       // Updates the current brightness index based on a brightness value
void saveCurrentValuesToPreset(uint8_t presetIndex);         // Saves current lamp state to the specified preset
void syncGlobalsFromPreset(uint8_t presetIndex);             // Syncs global variables from the specified preset

// MOTION - Head Servos
void controlServos();                                                                                           // Main servo control loop
bool headMovedSignificantly(int oldYaw, int oldPitch, int newYaw, int newPitch, float thresholdDegrees = 5.0f); // Checks if head moved significantly
void handleManualControl(unsigned long now, int16_t deltaYaw, int16_t deltaPitch);                              // Handles manual joystick control
void handleSnapbackTriggers(unsigned long now, unsigned long bumpDuration);                                     // Handles snapback triggers
void beginSnapback(unsigned long now, int16_t yaw, int16_t pitch, InterpolationMode mode);                      // Begins snapback motion
void resumeOriginalPosition();                                                                                  // Resumes original head position after snapback
void restoreMoodySnapback();                                                                                    // Restores moody mood after snapback
void restoreStartupSnapback();                                                                                  // Restores startup wander after snapback
void restoreDefaultSnapback();                                                                                  // Restores default focused mood after snapback
void applySnapbackLighting();                                                                                   // Applies lighting changes during snapback
void restoreServoPosition(float duration, InterpolationMode mode);                                              // Restores servo position with interpolation
void moveServosToTarget();                                                                                      // Moves servos towards target positions
float applyEasing(float t, InterpolationMode mode);                                                             // Applies easing function based on interpolation mode
void cancelMoodMotion();                                                                                        // Cancels any ongoing mood-driven motion
void completeServoMotion();                                                                                     // Completes any ongoing servo motion immediately
inline void clampHeadToDistractedBounds(int16_t &yaw, int16_t &pitch);                                          // Clamps head angles to distracted mood bounds
inline void clampHeadToDistractedBounds() { clampHeadToDistractedBounds(originalHeadYaw, originalHeadPitch); }  // Overload for current distracted bounds
inline void clampHeadToExcitedBounds(uint16_t &yaw, uint16_t &pitch);                                           // Clamps head angles to excited mood bounds

// MOTION - Height Motor
void motorControl(uint8_t speed, bool direction); // Controls motor speed and direction
void moveToHeight(int16_t targetHeight);          // Moves lamp to target height

// EEPROM
void savePresetsToEEPROM();   // Saves current presets to EEPROM
bool loadPresetsFromEEPROM(); // Loads presets from EEPROM

// MOODS - Setters
void setMoodDistracted(bool confirmationBlinkActive = false); // Sets the lamp to distracted mood
void setMoodHappy(bool confirmationBlinkActive = false);      // Sets the lamp to happy mood
void setMoodMoody(bool confirmationBlinkActive = false);      // Sets the lamp to moody mood
void setMoodFocused(bool confirmationBlinkActive = false);    // Sets the lamp to focused mood

// MOODS - Updates
// Distracted Mood Updates and Helper Functions
void updateDistractedMood();                              // Main update loop for distracted mood
void updateDistractedHeadOriginIfIdle(unsigned long now); // Updates original head position if joystick is idle
void triggerDistractedWanderIfReady(unsigned long now);   // Triggers wander motion if ready
void beginInterpolatedWander(unsigned long now, unsigned long durationMin, unsigned long durationMax,
                             unsigned long intervalMin, unsigned long intervalMax); // Begins interpolated wander motion
void handleDistractedIdle(unsigned long now);                                       // Handles distracted idle behavior
bool shouldUpdateDistractedSnapback(unsigned long now);                             // Checks if distracted snapback should be updated
void updateDistractedSnapbackOrigin(unsigned long now);                             // Updates distracted snapback origin

// Happy Mood Updates and Helper Functions
void updateHappyMood();                                // Main update loop for happy mood
void updateSmileyLightingIfNeeded();                   // Updates smiley lighting if active
void updateSmileyBlinkAndShiftIfEnabled();             // Updates smiley blink and shift if enabled
void updateExcitedHeadOriginIfIdle(unsigned long now); // Updates original head position if joystick is idle
void triggerExcitedWanderIfReady(unsigned long now);   // Triggers excited wander motion if ready
void handleHeightBounceCycle(unsigned long now);       // Handles height bounce cycle
void maybeTriggerExcitedAnimation(unsigned long now);  // Maybe triggers an excited animation
void handleHappyIdle(unsigned long now);               // Handles happy idle behavior

// Moody Mood Updates and Helper Functions
void updateMoodyMood(unsigned long now);                    // Main update loop for moody mood
void handleMoodyDelayPhase(unsigned long now);              // Handles initial delay phase of moody mood
void fadeOutWhiteLightsDuringTransition(unsigned long now); // Fades out white lights during moody transition
void beginMoodyNodCycleIfCentered(unsigned long now);       // Begins moody nod cycle if head is centered
void handleMoodyNodCycle(unsigned long now);                // Handles moody nod cycle
void handleMoodyIdle(unsigned long now);                    // Handles moody idle behavior
bool shouldRestoreMoodyAfterSnapback(unsigned long now);    // Checks if moody mood should be restored after snapback
void restoreMoodyAfterSnapback(unsigned long now);          // Restores moody mood after snapback
void resetMoodyState();                                     // Resets moody mood state

// Focused Mood Updates and Helper Functions
bool shouldBeginFocusedStartupWander(unsigned long now); // Checks if focused startup wander should begin
void beginFocusedStartupWander(unsigned long now);       // Begins focused startup wander

// MOODS -Indicators
void indicateActivatedMood(MoodBlinkType blinkType, uint32_t color = 0, uint8_t brightness = 255); // Visual indication of mood activation

// LED_CONTROL_UTILS
// Shared
uint8_t getCurrentBrightness();                             // Returns the current brightness value
void turnOffAllLights();                                    // Turns off all lights (Neopixels and white LEDs)
void restoreLightStateFromPreset(const LampPreset &preset); // Restores light state from a preset
void restorePreferredLighting();                            // Restores preferred lighting based on current mood

// White Light
void setWhiteLightMode(WhiteMode mode, uint8_t brightness, bool disableSmiley = false); // Sets white light mode
void fadeWhiteLightsToBlack(float t, WhiteMode mode, uint8_t originalBrightness);       // Fades white lights to black
void resetWhiteFadeState();                                                             // Resets white light fade state
void captureOriginalLightState();                                                       // Captures the original light state before a fade

// Neopixels
void setNeopixelColor(uint16_t hue, uint8_t brightness, bool force = false); // Sets Neopixel color
void applyColorLighting(uint16_t hue, uint8_t brightness);                   // Applies color lighting
void updateSelectedPixel(uint8_t currentBrightness);                         // Updates selected pixel during color selection
void applyBufferedNeopixels();                                               // Applies buffered Neopixel colors
uint8_t calculatePulseBrightness();                                          // Calculates brightness for pulsing effect
uint16_t interpolateHue(uint16_t fromHue, uint16_t toHue, float t);          // Interpolates between two hues
void restoreNeopixelBuffer(const uint32_t *colorBuffer, uint8_t brightness); // Restores Neopixel buffer from saved colors
void restoreWhiteLightMode(WhiteMode mode, uint8_t brightness);              // Restores white light mode
void startNeopixelInterpolation(uint16_t fromHue, uint8_t fromBrightness,
                                uint16_t toHue, uint8_t toBrightness,
                                uint16_t durationMs = 1000); // Starts Neopixel interpolation
void updateNeopixelInterpolation();                          // Updates Neopixel interpolation

// Smiley Face UI
bool isSmileyModeActive();            // Checks if smiley mode is active
void applySmileyLighting();           // Applies smiley face lighting
void drawSmileyFace();                // Draws the smiley face on Neopixels
void updateSmileyBlinkAndShift();     // Updates smiley blink and shift behavior
uint32_t getSmileyColor();            // Gets the current smiley color based on hue
void finishStartupWithoutAnimation(); // Finishes startup sequence without animation
void updateSmileyIfActive();          // Updates smiley face if active

// ANIMATIONS
void loadStartupAnimation();                                             // Loads and starts the startup animation
void playKeyframeAnimation();                                            // Plays the loaded keyframe animation
void handleSmileyDuringStartup();                                        // Handles smiley face during startup animation
void finishKeyframePlayback();                                           // Finishes keyframe playback
void updateCurrentKeyframeAnimation();                                   // Updates the current keyframe animation
void defineExcitedAnimations();                                          // Defines excited mood animations
void loadExcitedAnimation(uint8_t index);                                // Loads the specified excited animation
float interpolateQuadraticBezier(float p0, float p1, float p2, float t); // Interpolates a quadratic Bezier curve
void clampKeyframe(Keyframe &kf);                                        // Clamps keyframe values to valid ranges
void handleStartupSequenceFinished();                                    // Handles actions after startup sequence is finished
void updateFrameBasedMovement(unsigned long now);                        // Updates frame-based movement
void updateKeyframePlayback();                                           // Updates keyframe playback

// Loop Helper Functions
void tickAllButtons();                         // Ticks all button handlers
void handleMoodIdleTrigger(unsigned long now); // Handles mood idle triggers
bool shouldUpdateIdle();                       // Checks if idle update is needed

// Pomodoro
void startPomodoroMode();
void stopPomodoroMode();
void updatePomodoroMode(unsigned long now);
static inline uint32_t pomodoroColorHSV(uint16_t hue);
void advancePomodoroPeriod(unsigned long now, bool fromUserSkip);
void skipPomodoroPeriod(unsigned long now);

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

  if (Config::useSerial)
  {
    Serial.println(F("=== STARTUP CONFIG ==="));
    Serial.print(F("startupMood (0 - 3): "));
    Serial.println(Config::startupMood);
    Serial.print(F("useSmiley: "));
    Serial.println(Config::useSmiley);
    Serial.print(F("useStartupAnim: "));
    Serial.println(Config::useStartupAnim);
  }

  initializePins();
  initializeServos();
  seedRandomIfNeeded();
  applyStartupLightingFromConfig();
  initializeHeightEncoder();
  initializeButtons();
  initializeEEPROM();
  calibrateJoystick();
  initializeInterpolationDefaults();
  defineExcitedAnimations();

  if (Config::useStartupAnim)
  {
    loadStartupAnimation(); // Play keyframe sequence if useStartupAnim is true in Config.h
  }
  else
  {
    finishStartupWithoutAnimation(); // Skip animation and go to idle state if useStartupAnim is false in Config.h
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
  unsigned long now = millis();  // Check the time at the start of the loop
  tickAllButtons();              // Update button states (see if a button was pressed, double-pressed, released, etc.)
  handleMoodIdleTrigger(now);    // Handle idle triggers for the current mood
  updateSmileyIfActive();        // Update smiley face if active
  updateNeopixelInterpolation(); // Update Neopixel interpolation if active
  updateFrameBasedMovement(now); // Update frame-based movement for animations
  updateKeyframePlayback();      // Update keyframe playback if active
  updatePomodoroMode(now);
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
  currentWhiteMode = mode;

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

  for (uint8_t i = 0; i < numPixels; i++)
  {
    pixels.setPixelColor(i, pixels.ColorHSV(hue, 255, brightness));
  }
  pixels.show();
}

void applyColorLighting(uint16_t newHue, uint8_t brightness)
{
  // Turn off white LEDs any time we switch to colored mode
  analogWrite(coolLEDpin, 0);
  analogWrite(warmLEDpin, 0);

  // Keep the global hue updated
  hue = newHue;

  // If smiley mode is currently active, setNeopixelColor() may early-return.
  // That's OK: the smiley will render using hue via getSmileyColor().
  setNeopixelColor(newHue, brightness, false);
}

void turnOffAllLights()
{
  analogWrite(coolLEDpin, 0);
  analogWrite(warmLEDpin, 0);
  pixels.clear();
  pixels.show();
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
  else if (currentWhiteMode != 0)
  {
    pixels.clear();
    pixels.show();
    setWhiteLightMode(currentWhiteMode, getCurrentBrightness(), false);
  }
  else if ((wanderingAfterStartup && !userHasTakenControl) &&
           Config::useSmiley && !smileyDisabledAfterUserInput)
  {
    applySmileyLighting(); // Fallback for startup smiley
  }
  else
  {
    if (Config::useSmiley) // Final fallback only if smiley is enabled
    {
      turnOffAllLights();
    }
    else // Fallback to preset[0] lighting if smiley is disabled and nothing is active
    {
      // Smiley disabled: restore "normal" lighting state, not preset0.
      if (useColoredLight)
      {
        setNeopixelColor(hue, getCurrentBrightness(), true);
      }
      else
      {
        setWhiteLightMode(currentWhiteMode, getCurrentBrightness(), false);
      }
    }
  }
}

void updateSelectedPixel(uint8_t currentBrightness)
{
  static uint8_t colorOffset = 0; // Keeps track of the rotation of the color ring
  const uint8_t fixedBlinkingPixel = (0 + neopixelRotationOffset) % numPixels;

  if (!colorSelectionMode)
    return;

  unsigned long currentTime = millis();

  if (currentTime - lastNeopixelUpdate < neopixelUpdateInterval)
    return; // Skip updates if interval hasn't passed
  lastNeopixelUpdate = currentTime;

  // Read joystick values
  int16_t joystickYaw = analogRead(JoyYawPin) - joystickCenterYaw;       // Center the joystick reading (yaw-axis)
  int16_t joystickPitch = analogRead(JoyPitchPin) - joystickCenterPitch; // Center the joystick reading (pitch-axis)

  // X-axis movement: Rotate the color wheel
  if (abs(joystickYaw) > deadZoneThreshold)
  {
    if (joystickYaw > deadZoneThreshold)
    {
      colorOffset = (colorOffset - 1 + numPixels) % numPixels; // Rotate counter-clockwise
      logicalSelectedIndex = (logicalSelectedIndex - 1 + numPixels) % numPixels;
    }
    else if (joystickYaw < -deadZoneThreshold)
    {
      colorOffset = (colorOffset + 1) % numPixels; // Rotate clockwise
      logicalSelectedIndex = (logicalSelectedIndex + 1) % numPixels;
    }
    delay(100); // Add a small delay to prevent rapid changes
  }

  if (abs(joystickPitch) > deadZoneThreshold) // Y-axis movement: Adjust brightness
  {
    static unsigned long lastBrightnessChangeTime = 0; // Timestamp for last brightness adjustment

    if (currentTime - lastBrightnessChangeTime > 300) // 300ms debounce for brightness adjustment
    {
      if (joystickPitch > deadZoneThreshold && currentBrightnessIndex > 0)
      {
        currentBrightnessIndex--; // Decrease brightness
      }
      else if (joystickPitch < -deadZoneThreshold && currentBrightnessIndex < 4)
      {
        currentBrightnessIndex++; // Increase brightness
      }
      lastBrightnessChangeTime = currentTime;
    }
  }

  bool isWhiteSelected = (logicalSelectedIndex >= 9);

  for (uint8_t i = 0; i < numPixels; i++) // Adjust brightness and apply pulsing for the fixed blinking pixel
  {
    uint8_t pixelIndex = (i + colorOffset + neopixelRotationOffset) % numPixels;
    uint32_t color;
    uint8_t brightness = (i == fixedBlinkingPixel) ? calculatePulseBrightness() : getCurrentBrightness();

    if (pixelIndex < 9) // Colored Neopixels
    {
      uint8_t adjustedBrightness = isWhiteSelected ? dimPixel : brightness;
      color = pixels.ColorHSV(customHueRing[pixelIndex], 255, adjustedBrightness);
    }
    else // White Neopixels
    {
      uint8_t adjustedBrightness = isWhiteSelected ? brightness : dimPixel;
      switch (pixelIndex)
      {
      case 9:
        color = pixels.ColorHSV(8000, 200, adjustedBrightness); // Warm white
        break;
      case 10:
        color = pixels.ColorHSV(6000, 50, adjustedBrightness); // Soft white
        break;
      case 11:
        color = pixels.ColorHSV(43700, 60, adjustedBrightness); // Cool white
        break;
      }
    }
    neopixelBuffer[i] = color;
  }
}

void applyBufferedNeopixels()
{
  for (uint8_t i = 0; i < numPixels; i++)
  {
    pixels.setPixelColor(i, neopixelBuffer[i]);
  }
  pixels.show(); // Show the buffered colors
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
  pixels.setBrightness(brightness);
  for (uint8_t i = 0; i < numPixels; i++)
  {
    pixels.setPixelColor(i, colorBuffer[i]);
  }
  pixels.show();
}

uint8_t getCurrentBrightness()
{
  return pgm_read_byte(&(brightnessLevels[currentBrightnessIndex]));
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
  // Step 1: Backup current pixel buffer — even if smiley or white
  uint32_t originalColors[numPixels];
  bool ringWasActive = false;

  for (uint8_t i = 0; i < numPixels; i++)
  {
    originalColors[i] = pixels.getPixelColor(i);
    if (originalColors[i] != 0)
    {
      ringWasActive = true;
    }
  }

  originalBrightness = pixels.getBrightness();
  uint8_t whiteBrightness = getCurrentBrightness();
  WhiteMode previousWhiteMode = currentWhiteMode; // Backup current white mode

  // Step 2: Turn everything off to prepare for blink
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
    int blinkLevel = brightnessFactor * brightness;

    switch (blinkType)
    {
    case BLINK_COLOR:
      setNeopixelColor(color, blinkLevel, true);
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

  // Step 3: Restore whatever was on screen before the blink
  if (isSmileyModeActive())
  {
    drawSmileyFace(); // or applySmileyLighting(), depending on where you're at
  }
  else if (ringWasActive)
  {
    restoreNeopixelBuffer(originalColors, originalBrightness);
  }
  else
  {
    restoreWhiteLightMode(previousWhiteMode, whiteBrightness);
  }
}

void restoreWhiteLightMode(WhiteMode mode, uint8_t brightness)
{
  pixels.clear();
  pixels.show();
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

#pragma endregion LED_CONTROL_UTILS

//////////////////////////////////////////////////////////////////////////////////////////////////////////
// MOTOR MOVEMENT FUNCTIONS
//////////////////////////////////////////////////////////////////////////////////////////////////////////

#pragma region motorMovementFxns
void controlServos()
{
  unsigned long now = millis();
  if (now - lastManualControlTime < Config::manualControlInterval)
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

    // Reset PID accumulators when joystick control ends
    integralYaw = 0.0f;
    integralPitch = 0.0f;
    previousErrorX = 0.0f;
    previousErrorY = 0.0f;

    int newYaw = servoYaw.read();
    int newPitch = servoPitch.read();

    if (headMovedSignificantly(originalHeadYaw, originalHeadPitch, newYaw, newPitch))
    {
      originalHeadYaw = newYaw;
      originalHeadPitch = newPitch;
    }
  }
}

bool headMovedSignificantly(int oldYaw, int oldPitch, int newYaw, int newPitch, float thresholdDegrees = 5.0f)
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

  // Get current servo positions
  int16_t currentYaw = servoYaw.read();
  int16_t currentPitch = servoPitch.read();

  // Calculate target angles from joystick input
  int16_t targetYaw = constrain(currentYaw + speedYaw, Config::servoMinYawConstraint, Config::servoMaxYawConstraint);
  int16_t targetPitch = constrain(currentPitch + speedPitch, Config::servoMinPitchConstraint, Config::servoMaxPitchConstraint);

  // PID control for smooth joystick response
  // Yaw axis PID
  int16_t errorYaw = targetYaw - currentYaw;
  integralYaw += errorYaw;
  integralYaw = constrain(integralYaw, -200.0f, 200.0f); // Limit integral wind-up
  float derivativeYaw = errorYaw - previousErrorX;
  float pidOutputYaw = (Config::Kp * errorYaw) + (Config::Ki * integralYaw) + (Config::Kd * derivativeYaw);
  previousErrorX = errorYaw;

  // Pitch axis PID
  int16_t errorPitch = targetPitch - currentPitch;
  integralPitch += errorPitch;
  integralPitch = constrain(integralPitch, -200.0f, 200.0f); // Limit integral wind-up
  float derivativePitch = errorPitch - previousErrorY;
  float pidOutputPitch = (Config::Kp * errorPitch) + (Config::Ki * integralPitch) + (Config::Kd * derivativePitch);
  previousErrorY = errorPitch;

  // Apply PID output and clamp to valid servo range
  int16_t angleYaw = constrain(currentYaw + (int16_t)pidOutputYaw, Config::servoMinYawConstraint, Config::servoMaxYawConstraint);
  int16_t anglePitch = constrain(currentPitch + (int16_t)pidOutputPitch, Config::servoMinPitchConstraint, Config::servoMaxPitchConstraint);

  // Write servo positions
  servoYaw.write(angleYaw);
  servoPitch.write(anglePitch);

  // Mark that user has moved the head intentionally
  userMovedHeadDeliberately = true;

  if (currentMood == MOOD_MOODY)
  {
    originalMoodyPitch = servoPitch.read();
    originalMoodyYaw = servoYaw.read();

    // if (Config::useSerial)
    // {
    //   Serial.print(F("New original moody head position = "));
    //   Serial.print(originalMoodyPitch);
    //   Serial.print(F("|"));
    //   Serial.println(originalMoodyYaw);
    // }
  }
}

void handleSnapbackTriggers(unsigned long now, unsigned long bumpDuration)
{
  // DISTRACTED SNAPBACK
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

  // MOODY SNAPBACK
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
    completeServoMotion();
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
        // if (Config::useSerial)
        // {
        //   Serial.println(F("[MOODY] Snapback completed, ready to reenter nod cycle."));
        // }
      }

      if (!moodyInitialCenterReached)
      {
        moodyInTransition = false;
        moodyInitialCenterReached = true;
        moodyDroopDown = false;
        // if (Config::useSerial)
        // {
        //   Serial.println(F("Moody center reached."));
        // }
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

    // if(Config::useSerial){
    //   Serial.print(F("Height = "));
    //   Serial.println(currentHeight);
    // }

    if (abs(targetHeight - currentHeight) <= 15) // Stop the motor if we're within the tolerance range
    {
      motorControl(0, true); // Stop the motor
      isMoving = false;      // Mark as not moving

      if (currentMood == MOOD_MOODY && pendingMoodySnap)
      {
        moodyHeightRestoreComplete = true;
        // if(Config::useSerial)
        // {
        //   Serial.println(F("[MOODY] Height restoration complete — ready to accept new height"));
        // }
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
    drawSmileyFace();
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

  pinMode(motorSpeedPin, OUTPUT);
  pinMode(motorDirectionPin, OUTPUT);

  pixels.begin();
  pixels.clear();
  pixels.show();
}

void initializeServos()
{
  servoYaw.attach(servoYawPWMPin);
  servoPitch.attach(servoPitchPWMPin);
  delay(500); // Allow time for servos to reach center
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
  joyButton.setClickMs(Config::pomodoroDoubleClickMs);

  attachColorSelectionHandler();
  attachPresetShortcuts();
  attachMoodSelectionShortcuts();
  attachMotorButtons();

  joyButton.attachDoubleClick([]()
                              {
  if (!pomodoroModeActive) startPomodoroMode();
  else stopPomodoroMode(); });
}

void attachColorSelectionHandler()
{
  joyButton.attachClick([]()
                        {
    userHasTakenControl = true;
    moodyWhiteLightsDisabled = false;

      // If Pomodoro is running, single-click skips to next period
    if (pomodoroModeActive) {
      skipPomodoroPeriod(millis());
      return;
    }

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
    }
    else
    {
      // Confirm selection and exit color selection mode
      uint16_t confirmedHue = customHueRing[logicalSelectedIndex];
      uint8_t confirmedBrightness = getCurrentBrightness();

      if (logicalSelectedIndex < 9)
      {
        setNeopixelColor(confirmedHue, confirmedBrightness);
        useColoredLight = true;
        hue = confirmedHue;
      }
      else
      {
        useColoredLight = false;
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

                                   presets[0] = {90, 90, 230, false, 128, 230, WHITE_WARM};
                                   presets[1] = {70, 105, 360, true, 220, 8000, 0};
                                   presets[2] = {125, 75, 480, true, 200, 42000, 0};
                                   presets[3] = {90, 115, 560, true, 128, 62000, 0};
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
                            { setMoodDistracted(true);
                              userSelectedMood = true; });
  button2.attachDoubleClick([]()
                            { setMoodHappy(true);
                              userSelectedMood = true; });
  button3.attachDoubleClick([]()
                            { setMoodMoody(true);
                              userSelectedMood = true; });
  button4.attachDoubleClick([]()
                            { setMoodFocused(true);
                              userSelectedMood = true; });
}

void attachMotorButtons()
{
  downButton.attachDuringLongPress([]()
                                   {
    if (!colorSelectionMode)
    {
      motorControl(255, false); // Move downward
      currentHeight = analogRead(heightEncoderPin);
    //       if(Config::useSerial){
    //   Serial.print(F("Height = "));
    //   Serial.println(currentHeight);
    // }
    } });

  downButton.attachLongPressStop([]()
                                 {
                                   motorControl(0, true); // Stop motor
                                   originalMoodyHeight = analogRead(heightEncoderPin); });

  upButton.attachDuringLongPress([]()
                                 {
                                  if (!colorSelectionMode)
                                  {
                                    motorControl(255, true); // Move upward
                                          currentHeight = analogRead(heightEncoderPin);

    //                                     if(Config::useSerial){
    //   Serial.print(F("Height = "));
    //   Serial.println(currentHeight);
    // }
                                  } });

  upButton.attachLongPressStop([]()
                               {
                                 motorControl(0, true); // Stop motor (braking enabled)
                                 originalMoodyHeight = analogRead(heightEncoderPin); });
}

void initializeEEPROM()
{
  if (!loadPresetsFromEEPROM())
  {
    presets[0] = {90, 90, 230, false, 128, 230, WHITE_WARM};
    presets[1] = {70, 105, 360, true, 220, 8000, 0};
    presets[2] = {125, 75, 480, true, 200, 42000, 0};
    presets[3] = {90, 115, 560, true, 128, 62000, 0};
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

void seedRandomIfNeeded()
{
  if (Config::startupHueMode == Config::STARTUP_HUE_RANDOM)
  {
    uint16_t seedSource = analogRead(A5) ^ (uint16_t)micros();
    randomSeed(seedSource);

    if (Config::useSerial)
    {
      Serial.print(F("Random seed generated: "));
      Serial.println(seedSource);
    }
  }
  else
  {
    if (Config::useSerial)
    {
      Serial.println(F("Startup hue mode: FIXED (no RNG seed)"));
    }
  }
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
    if (pgm_read_byte(&(brightnessLevels[i])) == brightness)
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

void setMoodDistracted(bool confirmationBlinkActive)
{
  currentMood = MOOD_DISTRACTED;
  userHasTakenControl = true;

  if (Config::useSerial)
  {
    Serial.println(F(">>> setMoodDistracted() CALLED"));
  }

  bool comingFromStartupSmiley = (wanderingAfterStartup && !userHasTakenControl && Config::useSmiley && !smileyDisabledAfterUserInput);
  captureOriginalLightState();
  wanderingAfterStartup = false;
  if (!originalUseColoredLight && !comingFromStartupSmiley) // Don't disable the smiley unless light was explicitly chosen
  {
    smileyDisabledAfterUserInput = true;
  }

  if (confirmationBlinkActive)
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
  interpolationStartTime = 0; // Reset interpolator cleanly

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

void setMoodHappy(bool confirmationBlinkActive)
{
  currentMood = MOOD_HAPPY;
  userSelectedMood = true;
  currentAnimationType = ANIM_EXCITED;

  if (Config::useSerial)
  {
    Serial.println(F("setMoodHappy() called"));
  }

  wanderingAfterStartup = false;

  if (confirmationBlinkActive)
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
    drawSmileyFace();
  }

  originalHeadYaw = servoYaw.read();
  originalHeadPitch = servoPitch.read();
  clampHeadToExcitedBounds(originalHeadYaw, originalHeadPitch);

  currentExcitedIndex = random(0, 2); // Pick a random animation
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
    return; // Don't allow other animations during bounce
  }
}

void maybeTriggerExcitedAnimation(unsigned long now)
{
  if (!isKeyframePlaybackActive &&
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

void setMoodMoody(bool confirmationBlinkActive)
{
  currentMood = MOOD_MOODY;
  userHasTakenControl = true;

  // Cancel startup idle state and smiley
  wanderingAfterStartup = false;
  cancelMoodMotion();
  smileyDisabledAfterUserInput = true;

  hasRestoredMoodyOnce = false;
  moodyStartedInterpToCenter = false;

  if (confirmationBlinkActive)
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
        turnOffAllLights();

      whiteLightsAreOff = true;
    }

    // Block further progression until fade is done
    return;
  }

  beginMoodyNodCycleIfCentered(now);
  handleMoodyNodCycle(now);
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
  }
}

void handleMoodyNodCycle(unsigned long now)
{
  if (!moodyLooping)
    return;

  // 1: Finished nodding down
  if (moodyDescending && now - moodyPhaseStartTime >= Config::headDescendTime)
  {
    moodyDescending = false;
    moodyPaused = true;
    lastMoodyPauseTime = now;
    return;
  }

  // 2: Finished pause -> rise back up
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

    return;
  }

  // 3: Finished nod up -> restart nod down
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
    return;
  }
}

void setMoodFocused(bool confirmationBlinkActive)
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

  if (confirmationBlinkActive)
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

  // MAIN LOGIC

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
      readyToUpdateOriginalHead = false;
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

  // Start moody fade-in transition (blue -> purple)
  moodyInTransition = true;
  moodyInitialCenterReached = false;
  moodyTransitionStartTime = now;
  moodyTransitionPhase = 0; // triggers phase 0: current -> blue

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

  for (uint8_t i = 0; i < startupAnimationRawLength; i++) // Copy from PROGMEM to RAM
  {
    memcpy_P(&activeKeyframes[i], &startupAnimationRaw[i], sizeof(Keyframe));
    clampKeyframe(activeKeyframes[i]);
  }

  const int16_t baseHeight = presets[0].height; // Optionally override height with actual value from preset[0]

  for (uint8_t i = 0; i < startupAnimationRawLength; i++)
  {
    if (activeKeyframes[i].height == 425)
    {
      activeKeyframes[i].height = baseHeight + 60;
    }
    else if (activeKeyframes[i].height == 365)
    {
      activeKeyframes[i].height = baseHeight;
    }
  }

  activeKeyframeCount = startupAnimationRawLength;
  currentKeyframeIndex = 0;
  keyframeStartTime = millis();
  isKeyframePlaybackActive = true;

  if (Config::useSmiley)
  {
    analogWrite(coolLEDpin, 0);
    analogWrite(warmLEDpin, 0);
    drawSmileyFace();
  }
}

void clampKeyframe(Keyframe &kf)
{
  kf.yaw = constrain(kf.yaw, Config::servoMinYawConstraint, Config::servoMaxYawConstraint);
  kf.pitch = constrain(kf.pitch, Config::servoMinPitchConstraint, Config::servoMaxPitchConstraint);
  kf.controlYaw = constrain(kf.controlYaw, Config::servoMinYawConstraint, Config::servoMaxYawConstraint);
  kf.controlPitch = constrain(kf.controlPitch, Config::servoMinPitchConstraint, Config::servoMaxPitchConstraint);
}

void handleStartupSequenceFinished()
{
  if (userSelectedMood)
    return;

  if (Config::useSerial)
  {
    Serial.println(F("--- handleStartupSequenceFinished() ---"));
    Serial.print(F("userSelectedMood: "));
    Serial.println(userSelectedMood);
    Serial.print(F("startupMood (raw): "));
    Serial.println(Config::startupMood);
  }

  // Smiley enabled/disabled should not force a mood choice
  smileyDisabledAfterUserInput = !Config::useSmiley;

  if (Config::useSerial)
  {
    Serial.println(F("Applying startupMood..."));
  }

  switch (static_cast<MoodState>(Config::startupMood))
  {
  case MOOD_HAPPY:
    Serial.println(F("startupMood = HAPPY"));
    setMoodHappy(false);
    break;

  case MOOD_MOODY:
    Serial.println(F("startupMood = MOODY"));
    setMoodMoody(false);
    break;

  case MOOD_DISTRACTED:
    Serial.println(F("startupMood = DISTRACTED"));
    setMoodDistracted(false);
    wanderingAfterStartup = true;
    nextStartupWanderTime = millis() + Config::idleDelayAfterStartup;
    break;

  case MOOD_FOCUSED:
    Serial.println(F("startupMood = FOCUSED"));
    setMoodFocused(false);
    break;
  }

  // If smiley is enabled and allowed, draw it (optional)
  if (Config::useSmiley && !smileyDisabledAfterUserInput && !userSelectedMood)
  {
    drawSmileyFace();
  }
}

void playKeyframeAnimation()
{
  if (Config::useSmiley && !smileyDisabledAfterUserInput && currentAnimationType == ANIM_STARTUP)
  {
    handleSmileyDuringStartup();
  }

  if (currentKeyframeIndex >= activeKeyframeCount)
  {
    finishKeyframePlayback();
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
    for (uint8_t i = 4; i < 9; i++)
    {
      pixels.setPixelColor(i, 0);
    }
    pixels.show();
  }
  else if (currentServoPitch >= 75 && eyesSuppressed)
  {
    eyesSuppressed = false;
    drawSmileyFace();
  }
}

void finishKeyframePlayback()
{
  isKeyframePlaybackActive = false;
  eyesSuppressed = false;

  if (currentAnimationType == ANIM_STARTUP)
  {
    handleStartupSequenceFinished();
  }
}

void updateCurrentKeyframeAnimation()
{
  Keyframe &kf = activeKeyframes[currentKeyframeIndex];
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
      startYaw = activeKeyframes[currentKeyframeIndex - 1].yaw;
      startPitch = activeKeyframes[currentKeyframeIndex - 1].pitch;
      startHeight = analogRead(heightEncoderPin);
      keyframeStartTime = now;
    }
  }
}

void defineExcitedAnimations()
{
  for (uint8_t i = 0; i < animation0Length; i++) // Copy excitedAnimation0 from PROGMEM into RAM buffer
  {
    memcpy_P(&excitedAnimations[0][i], &animation0[i], sizeof(Keyframe));
    clampKeyframe(excitedAnimations[0][i]); // safety clamp
  }
  excitedKeyframeCounts[0] = animation0Length;

  for (uint8_t i = 0; i < animation1Length; i++) // Copy excitedAnimation1 from PROGMEM into RAM buffer
  {
    memcpy_P(&excitedAnimations[1][i], &animation1[i], sizeof(Keyframe));
    clampKeyframe(excitedAnimations[1][i]); // safety clamp
  }
  excitedKeyframeCounts[1] = animation1Length;
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
    drawSmileyFace();
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

  // Load keyframes from excitedAnimations[index] into activeKeyframes[]
  activeKeyframeCount = excitedKeyframeCounts[index];
  for (uint8_t i = 0; i < activeKeyframeCount; i++)
  {
    activeKeyframes[i] = excitedAnimations[index][i];
  }

  currentKeyframeIndex = 0;
  keyframeStartTime = millis();
  isKeyframePlaybackActive = true;

  // Smiley mode overrides yellow lighting — so skip any fade to yellow
  excitedLightOverridden = true; // Still block reversion after animation
}

void drawSmileyFace()
{
  const uint8_t mouthIndices[] = {0, 1, 11};
  const uint32_t smileyColor = getSmileyColor();

  for (uint8_t i = 0; i < numPixels; i++)
  {
    pixels.setPixelColor(i, 0);
  }

  if (currentSmileyDirection == LOOK_LEFT)
  {
    pixels.setPixelColor(5, smileyColor);
    pixels.setPixelColor(8, smileyColor);
    pixels.setPixelColor(2, 0);
    pixels.setPixelColor(10, smileyColor);
  }
  else
  {
    pixels.setPixelColor(4, smileyColor);
    pixels.setPixelColor(7, smileyColor);
    pixels.setPixelColor(2, smileyColor);
    pixels.setPixelColor(10, 0);
  }

  for (uint8_t i = 0; i < sizeof(mouthIndices); i++)
  {
    pixels.setPixelColor(mouthIndices[i], smileyColor);
  }

  pixels.show();
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
  const uint32_t smileyColor = getSmileyColor();

  if (smileyBlinkInProgress) // Handle Blink
  {
    if (now - smileyBlinkStart >= 160)
    {
      if (currentSmileyDirection == LOOK_LEFT) // Restore eyes after blink
      {
        pixels.setPixelColor(5, smileyColor);
        pixels.setPixelColor(8, smileyColor);
      }
      else
      {
        pixels.setPixelColor(4, smileyColor);
        pixels.setPixelColor(7, smileyColor);
      }

      pixels.show();
      smileyBlinkInProgress = false;
      lastSmileyBlinkTime = now;
      nextSmileyBlinkDelay = random(3000, 5000);
    }
  }
  else if (now - lastSmileyBlinkTime >= nextSmileyBlinkDelay)
  {
    if (currentSmileyDirection == LOOK_LEFT) // Start blink
    {
      pixels.setPixelColor(5, 0);
      pixels.setPixelColor(8, 0);
    }
    else
    {
      pixels.setPixelColor(4, 0);
      pixels.setPixelColor(7, 0);
    }

    pixels.show();
    smileyBlinkStart = now;
    smileyBlinkInProgress = true;
  }

  if (smileyShiftInProgress) // Handle Eye Shift
  {
    if (now - smileyShiftStart >= 3000)
    {
      currentSmileyDirection = LOOK_LEFT;
      drawSmileyFace(); // Redraw face with new direction
      smileyShiftInProgress = false;
      lastSmileyShiftTime = now;
      nextSmileyShiftDelay = random(8000, 13000);
    }
  }
  else if (now - lastSmileyShiftTime >= nextSmileyShiftDelay)
  {
    currentSmileyDirection = LOOK_RIGHT;
    drawSmileyFace(); // Redraw face with new direction
    smileyShiftStart = now;
    smileyShiftInProgress = true;
  }
}

uint32_t getSmileyColor()
{
  uint16_t smileyHue = 7282;      // Default yellow if all else fails
  uint8_t smileySaturation = 255; // Full color by default

  // Smiley should inherit the current lamp hue (fixed or random) whenever
  // we're using colored lighting OR we're in startup animation.
  if (currentAnimationType == ANIM_STARTUP)
  {
    smileyHue = hue;
    smileySaturation = 255;
  }
  else if (useColoredLight)
  {
    smileyHue = hue;
    smileySaturation = 255;
  }
  else
  {
    // If in white mode, tint the smiley slightly depending on white temperature selection
    switch (logicalSelectedIndex)
    {
    case 9: // Warm
      smileyHue = 8000;
      smileySaturation = 200;
      break;
    case 10: // Soft
      smileyHue = 6000;
      smileySaturation = 50;
      break;
    case 11: // Cool
      smileyHue = 43700;
      smileySaturation = 60;
      break;
    default:
      smileyHue = 7282;
      smileySaturation = 255;
      break;
    }
  }

  return pixels.ColorHSV(smileyHue, smileySaturation, 255);
}

void finishStartupWithoutAnimation()
{
  if (!userSelectedMood)
  {
    if (Config::useSmiley)
    {
      // Smiley on: obey startupMood (do not force distracted)
      smileyDisabledAfterUserInput = false;

      switch (static_cast<MoodState>(Config::startupMood))
      {
      case MOOD_HAPPY:
        setMoodHappy(false);
        break;

      case MOOD_MOODY:
        setMoodMoody(false);
        break;

      case MOOD_DISTRACTED:
        setMoodDistracted(false);
        wanderingAfterStartup = true;
        nextStartupWanderTime = millis() + Config::idleDelayAfterStartup;
        break;

      case MOOD_FOCUSED:
        setMoodFocused(false);
        break;
      }
    }
    else
    {
      // Smiley off: restore preset 0 *visuals* (lighting state) as the baseline.
      // (Avoid applyPreset(0) to keep this path from setting presetActive.)
      syncGlobalsFromPreset(0);
      applyStartupLightingFromConfig();
      updateCurrentBrightnessIndex(presets[0].brightness);

      // With no smiley feature, this should remain true.
      smileyDisabledAfterUserInput = true;

      switch (static_cast<MoodState>(Config::startupMood))
      {
      case MOOD_HAPPY:
        setMoodHappy(false);
        break;

      case MOOD_MOODY:
        setMoodMoody(false);
        break;

      case MOOD_DISTRACTED:
        setMoodDistracted(false);
        wanderingAfterStartup = true;
        nextStartupWanderTime = millis() + Config::idleDelayAfterStartup;
        break;

      case MOOD_FOCUSED:
        setMoodFocused(false);
        break;
      }
    }
  }

  // Move head to preset[0] yaw/pitch using the normal interpolator.
  currentServoYaw = servoYaw.read();
  currentServoPitch = servoPitch.read();

  startYaw = currentServoYaw;
  startPitch = currentServoPitch;

  targetServoYaw = presets[0].yawPosition;
  targetServoPitch = presets[0].pitchPosition;

  interpolationDuration = 1200.0f;
  interpolationStartTime = 0;
  currentInterpMode = INTERP_EASE_IN_OUT;

  moodDrivingServos = true;
  isWandering = true;

  // If smiley is active and allowed, render it.
  if (Config::useSmiley && !smileyDisabledAfterUserInput && !userSelectedMood)
  {
    drawSmileyFace();
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
    drawSmileyFace();
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

void updateKeyframePlayback()
{
  if (isKeyframePlaybackActive)
  {
    playKeyframeAnimation();
  }
}

void applyStartupLightingFromConfig()
{
  // Clear any previous light state (both systems)
  pixels.clear();
  pixels.show();
  analogWrite(coolLEDpin, 0);
  analogWrite(warmLEDpin, 0);

  // Clamp and apply startup brightness *index* (0..brightnessLevelCount-1)
  uint8_t idx = Config::startupBrightnessIndex;
  if (idx >= brightnessLevelCount)
  {
    idx = brightnessLevelCount - 1;
  }
  currentBrightnessIndex = idx;

  const uint8_t brightness = getCurrentBrightness();

  if (Config::startupUseColoredLight)
  {
    useColoredLight = true;

    // Pick startup hue ONCE per boot.
    if (!startupHueLocked)
    {
      if (Config::startupHueMode == Config::STARTUP_HUE_FIXED)
      {
        hue = Config::startupHue;
      }
      else // Config::STARTUP_HUE_RANDOM
      {
        hue = pickStartupHue(); // 50% palette, 50% full-range (per your new rule)
      }

      startupHueLocked = true;

      if (Config::useSerial)
      {
        Serial.print(F("Startup hue applied: "));
        Serial.println(hue);
      }
    }

    applyColorLighting(hue, brightness);
  }
  else
  {
    useColoredLight = false;

    // Validate mode (Config stores a WhiteMode)
    uint8_t mode = (uint8_t)Config::startupWhiteMode;
    if (mode < WHITE_WARM || mode > WHITE_COOL)
    {
      mode = WHITE_WARM;
    }
    currentWhiteMode = static_cast<WhiteMode>(mode);

    setWhiteLightMode(currentWhiteMode, brightness);
  }
}

uint16_t pickStartupHue()
{
  // 50/50 mix: standard palette vs full-range random
  const bool usePalette =
      Config::startupRandomPreferPalette &&
      (random(100) < Config::startupPalettePickPercent);

  if (usePalette)
  {
    // Pick a random entry from your palette.
    // Assumes customHueRing is a compile-time array of uint16_t.
    const uint8_t paletteCount = sizeof(customHueRing) / sizeof(customHueRing[0]);
    const uint8_t idx = random(paletteCount);
    return customHueRing[idx];
  }

  // Otherwise, full-range random hue (inclusive).
  const uint16_t lo = Config::startupRandomHueMin;
  const uint16_t hi = Config::startupRandomHueMax;

  uint32_t span32 = (hi >= lo) ? (uint32_t)(hi - lo) + 1u : 0u;
  uint32_t r = (span32 > 0u) ? (uint32_t)random((long)span32) : 0u;

  return (uint16_t)(lo + r);
}

#pragma endregion Animations

#pragma region Hack 2: Pomodoro Timer

// --------------------------------------------------------------------
// HACK 4: Pomodoro Timer
// --------------------------------------------------------------------
static inline void pomodoroDisableWhiteLEDs()
{
  analogWrite(warmLEDpin, 0);
  analogWrite(coolLEDpin, 0);
}

static inline uint32_t pomodoroColorHSV(uint16_t hue)
{
  return pixels.ColorHSV(hue, Config::pomodoroSat, Config::pomodoroVal);
}

static inline uint32_t pomodoroColorHSVScaled(uint16_t hue, uint8_t val)
{
  return pixels.ColorHSV(hue, Config::pomodoroSat, val);
}

static inline void pomodoroFillRingOffset(uint32_t color)
{
  pixels.clear();
  const uint8_t offset = neopixelRotationOffset;
  for (uint8_t i = 0; i < 12; i++)
  {
    uint8_t idx = (i + offset) % 12;
    pixels.setPixelColor(idx, color);
  }
  pixels.show();
}

static inline void pomodoroPulseHue(uint16_t hue,
                                    uint8_t times,
                                    uint16_t pulseMs = 520,
                                    uint8_t minVal = 0,
                                    uint8_t maxVal = Config::pomodoroVal)
{
  // pulseMs is the full up+down time for ONE pulse
  // Keep steps modest to avoid heavy loop blocking / large CPU
  const uint8_t steps = 18; // 18 up + 18 down ~ smooth enough
  const uint16_t stepDelay = pulseMs / (2 * steps);

  for (uint8_t t = 0; t < times; t++)
  {
    // Fade up
    for (uint8_t s = 0; s <= steps; s++)
    {
      uint8_t v = (uint8_t)(minVal + ((uint16_t)(maxVal - minVal) * s) / steps);
      pomodoroFillRingOffset(pomodoroColorHSVScaled(hue, v));
      delay(stepDelay);
    }

    // Fade down
    for (int8_t s = steps; s >= 0; s--)
    {
      uint8_t v = (uint8_t)(minVal + ((uint16_t)(maxVal - minVal) * s) / steps);
      pomodoroFillRingOffset(pomodoroColorHSVScaled(hue, v));
      delay(stepDelay);
    }
  }
}

void startPomodoroMode()
{
  if (pomodoroModeActive)
    return;

  // Save prior state so we can restore cleanly on stop
  pomodoroPrevMood = currentMood;
  pomodoroPrevSmileyDisabled = smileyDisabledAfterUserInput;
  pomodoroPrevUserSelectedMood = userSelectedMood;
  pomodoroPrevPresetActive = presetActive;
  pomodoroPrevColorSelectionMode = colorSelectionMode;

  // Force out of other UI modes
  colorSelectionMode = false;
  presetActive = false;

  // Suppress smiley so it doesn't overwrite our NeoPixels
  smileyDisabledAfterUserInput = true;
  userSelectedMood = true;

  // Enter focused mood (pose/behavior cue)
  setMoodFocused(false);

  pomodoroDisableWhiteLEDs();

  // Start in WORK phase
  pomodoroModeActive = true;
  pomodoroOnBreak = false;
  pomodoroCurrentBreakIsLong = false;
  pomodoroCompletedWorkSessions = 0;

  pomodoroStartTime = millis();
  pomodoroIntervalDuration = Config::workingTime;

  // Blink Work Color 3x before work starts
  pomodoroPulseHue(Config::pomodoroWorkHue, 2, 520, 30, Config::pomodoroVal);

  // Show full WORK ring (configurable hue)
  pomodoroFillRingOffset(pomodoroColorHSV(Config::pomodoroWorkHue));

  if (Config::useSerial)
    Serial.println(F("[POMODORO] START → Work (Focused)."));
}
void stopPomodoroMode()
{
  if (!pomodoroModeActive)
    return;

  pomodoroModeActive = false;

  // Turn off Pomodoro ring + guarantee whites off
  pixels.clear();
  pixels.show();
  pomodoroDisableWhiteLEDs();

  // Restore flags
  smileyDisabledAfterUserInput = pomodoroPrevSmileyDisabled;
  userSelectedMood = pomodoroPrevUserSelectedMood;
  presetActive = pomodoroPrevPresetActive;
  colorSelectionMode = pomodoroPrevColorSelectionMode;

  // Return to previous mood quietly (no blink)
  switch (pomodoroPrevMood)
  {
  case MOOD_DISTRACTED:
    setMoodDistracted(false);
    break;
  case MOOD_HAPPY:
    setMoodHappy(false);
    break;
  case MOOD_MOODY:
    setMoodMoody(false);
    break;
  case MOOD_FOCUSED:
    setMoodFocused(false);
    break;
  }

  // Restore lighting preference (allowed again after Pomodoro ends)
  restorePreferredLighting();

  if (Config::useSerial)
    Serial.println(F("[POMODORO] STOP → Restored previous mood/lighting."));
}

void updatePomodoroMode(unsigned long now)
{
  if (!pomodoroModeActive)
    return;

  // IMPORTANT: avoid stale "now" passed from loop
  now = millis();

  // Keep lamp behavior stable during Pomodoro
  moodDrivingServos = false;
  isWandering = false;

  // Hard guarantee: keep white LEDs off during Pomodoro
  pomodoroDisableWhiteLEDs();

  unsigned long elapsed = now - pomodoroStartTime;
  float progress = (float)elapsed / (float)pomodoroIntervalDuration;
  progress = constrain(progress, 0.0f, 1.0f);

  // Smooth fade math: exactLedsRemaining goes 12 -> 0
  float exactLedsRemaining = 12.0f - (12.0f * progress);
  if (exactLedsRemaining < 0.0f)
    exactLedsRemaining = 0.0f;

  uint8_t wholeLeds = (uint8_t)floor(exactLedsRemaining);
  float fractional = exactLedsRemaining - (float)wholeLeds; // 0..1

  // Choose phase hue (configurable)
  uint16_t phaseHue = Config::pomodoroWorkHue;
  if (pomodoroOnBreak)
  {
    phaseHue = pomodoroCurrentBreakIsLong ? Config::pomodoroLongBreakHue
                                          : Config::pomodoroShortBreakHue;
  }

  // Update ring only when it meaningfully changes
  static uint8_t lastWholeLeds = 12;
  static uint8_t lastPercentBucket = 255;
  static uint8_t lastFadeBucket = 255;

  uint8_t fadeBucket = (uint8_t)(fractional * 20.0f);

  if (wholeLeds != lastWholeLeds || fadeBucket != lastFadeBucket)
  {
    pixels.clear();
    const uint8_t offset = neopixelRotationOffset;

    uint32_t fullColor = pomodoroColorHSV(phaseHue);
    for (uint8_t i = 0; i < wholeLeds; i++)
    {
      uint8_t idx = (i + offset) % 12;
      pixels.setPixelColor(idx, fullColor);
    }

    if (wholeLeds < 12)
    {
      uint8_t val = (uint8_t)(fractional * (float)Config::pomodoroVal);
      uint8_t idx = (wholeLeds + offset) % 12;
      pixels.setPixelColor(idx, pomodoroColorHSVScaled(phaseHue, val));
    }

    pixels.show();
    lastWholeLeds = wholeLeds;
    lastFadeBucket = fadeBucket;
  }

  // Debug output every 10%
  uint8_t percent = (uint8_t)(progress * 100.0f);
  if ((percent / 10) != (lastPercentBucket / 10))
  {
    lastPercentBucket = percent;
    if (Config::useSerial)
    {
      Serial.print(F("[POMODORO] "));
      Serial.print(pomodoroOnBreak ? (pomodoroCurrentBreakIsLong ? F("LongBreak") : F("ShortBreak")) : F("Work"));
      Serial.print(F(" "));
      Serial.print(percent);
      Serial.print(F("%  LEDs="));
      Serial.println(wholeLeds);
    }
  }

  // Interval complete?
  if (elapsed >= pomodoroIntervalDuration)
  {
    advancePomodoroPeriod(now, false);
    return;
  }
}

void skipPomodoroPeriod(unsigned long now)
{
  if (!pomodoroModeActive)
    return;

  // IMPORTANT: use the same time basis as the rest of Pomodoro
  // (callers should pass millis(), but we still guard here)
  unsigned long t = millis();
  if (t > now)
    now = t;

  // cooldown guard for user skips only
  if (now - pomodoroLastSkipMs < 500)
    return;
  pomodoroLastSkipMs = now;

  advancePomodoroPeriod(now, true);
}

void advancePomodoroPeriod(unsigned long now, bool fromUserSkip)
{
  // Reset the start time FIRST (prevents immediate retrigger)
  pomodoroStartTime = now;

  // Keep lamp stable + keep whites off
  moodDrivingServos = false;
  isWandering = false;
  pomodoroDisableWhiteLEDs();

  if (Config::useSerial)
  {
    Serial.print(F("[POMODORO] ADVANCE "));
    if (fromUserSkip)
      Serial.print(F("(user) "));
    Serial.println(pomodoroOnBreak ? F("break -> work") : F("work -> break"));
  }

  if (pomodoroOnBreak)
  {
    // BREAK -> WORK
    pomodoroOnBreak = false;
    pomodoroCurrentBreakIsLong = false;
    pomodoroIntervalDuration = Config::workingTime;

    setMoodFocused(false);
    pomodoroDisableWhiteLEDs();

    // Pulse work hue before work starts
    pomodoroPulseHue(Config::pomodoroWorkHue, 3, 520, 30, Config::pomodoroVal);

    // Full WORK ring
    pomodoroFillRingOffset(pomodoroColorHSV(Config::pomodoroWorkHue));
    return;
  }

  // WORK -> BREAK
  pomodoroCompletedWorkSessions++;
  pomodoroOnBreak = true;

  pomodoroCurrentBreakIsLong =
      (pomodoroCompletedWorkSessions % Config::numBreaksBeforeLongBreak) == 0;

  pomodoroIntervalDuration = pomodoroCurrentBreakIsLong ? Config::longBreakTime
                                                        : Config::breakTime;

  // Pulse break hue after work ends (short or long hue)
  uint16_t nextBreakHue = pomodoroCurrentBreakIsLong ? Config::pomodoroLongBreakHue
                                                     : Config::pomodoroShortBreakHue;

  pomodoroPulseHue(nextBreakHue, 3, 520, 30, Config::pomodoroVal);

  setMoodDistracted(false);
  pomodoroDisableWhiteLEDs();

  // Full BREAK ring
  pomodoroFillRingOffset(pomodoroColorHSV(nextBreakHue));
}

#pragma endregion Hack 2 : Pomodoro Timer

#pragma endregion FUNCTIONS

//////////////////////////////////////////////////
//  END CODE  //
//////////////////////////////////////////////////