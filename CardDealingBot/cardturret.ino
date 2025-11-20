#include <Arduino.h>
#pragma region README

/*
 ************************************************************************************
 * Card Dealing Robot - Card Turret
 * 02.24.2025
 * Version: 2.4.5
 * Author: Crunchlabs LLC
 *
 *   This hack uses the remote control and IR Receiver from the Turret build to make your Card
 *   Dealing Robot a fearsome card-throwing turret. Well, it won't be that scary, but still
 *   pretty cool that you turn your humble card-dealer into a remote-control sentry. Since you
 *   no longer need to go slow enough to sense colors, you can use the faster Domino motor to
 *   really whip around for extra speed. For community updates, check out our Discord. For help
 *   on your hacks, check out Mark Robot on the IDE!
 ************************************************************************************
 */

#pragma endregion README
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
CONFIGURATION
Some handy toggles and values pulled up to the top for ease of access.
*/
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#pragma region CONFIGURATION

// HANDY TOGGLES AND VALUES
bool useSerial = false;                       // Enables serial output for debugging. Set to false to disable serial output. Some statements need manual uncommenting for memory reasons.
uint16_t textSpeedInterval = 160;             // How fast do you read?? Amount of time (in ms) between frames of scrolling text (Lower number = faster text scrolling).
uint16_t textStartHoldTime = 800;             // Amount of time (in ms) scrolling text should pause before advancing.
uint16_t textEndHoldTime = 800;               // Amount of time (in ms) that scrolling text should pause at the end of a scroll.
const unsigned long expressionDuration = 500; // DEALR makes faces when it deals cards. This value determines the amount of time it makes the face for.
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
EDITING DEALR'S DEALING FACES
While dealing, your Card Dealing Robot can make all kinds of faces. You can modify what these look like by editing the symbols between the quotes. Just remember, every
face must be exactly four characters long, including spaces.
*/
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
const char *EFFORT = "X  X";   // The face DEALR makes in unrigged games while dealing a card
const char *LEFT = ">  >";     // The face DEALR makes when it looks left
const char *RIGHT = "<  <";    // The face DEALR makes when it looks right
const char *LOOK_BIG = "O  O"; // The face DEALR makes right before dealing a card in a regular game

struct DisplayAnimation // This little block has to come before the animation definitions, which let you change the faces DEALR makes.
{
  const char **frames;            // Pointer to array of frames
  const unsigned long *intervals; // Pointer to array of timing intervals
  uint8_t numFrames;              // Number of frames in the animation
};

#define ARRAY_SIZE(arr) (sizeof(arr) / sizeof(arr[0])) // This line makes it so we don't have to count how many frames each animation has manually.

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
EDITING ANIMATIONS
DEALR comes stock with two animations: one quick blinking animation it does right on boot, and one "screensaver" animation it does when it's bored.
If you look under "Initial blinking animation," you'll notice a series of symbols in quotes. Each four-character section between the quotes is a "frame."
For example: "O  O" is two wide eyes separated by two spaces. You can change these sections to anything you want, as long as you have exactly 4 characters.
So "X  X" works, "MARK" works, but "GUS" is too short and would need to be " GUS" or "GUS ". Each "frame" corresponds with an interval, or the amount of
time that frame should be displayed for in milliseconds. So if you want a frame to say "MARK" for 1 second, look at the next line, find the corresponding interval,
and type 1000.

The number of frames must equal the number of intervals, so if you add frames to the end of an animation, make sure you remember to add new intervals for those frames.

You can create new animations and call them in the script, but if you're just getting started, try changing some frames in the existing animations to see what happens!
*/
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Initial blinking animation
const char *introFrames[] = {
"O  O", 
"-  -", 
"O  O"};
const unsigned long introIntervals[] = {
1100 , 
75   ,
1100 };
const DisplayAnimation initialBlinking = {introFrames, introIntervals, ARRAY_SIZE(introFrames)};
#pragma endregion CONFIGURATION
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
INCLUDED LIBRARIES
All the below, with the exception of NHY3274TH.h, are common external libraries.
NHY3274TH is a custom color sensor that uses a custom library.
*/
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#pragma region LIBRARIES

#include <Servo.h>                // Controls the card-feeding servo.
#include <Wire.h>                 // Enables I2C, which we use for communicating with the 14-segment LED display.
#include <Adafruit_GFX.h>         // Used for the 14-segment display.
#include <Adafruit_LEDBackpack.h> // Used for the 14-segment display.
#include <EEPROM.h>               // Helps us save information to EEPROM, which is like a tiny hard drive on the Nano. This lets us save values even when power-cycling.
#include <avr/pgmspace.h>         // Lets us store values to flash memory instead of SRAM. Filling SRAM completely causes issue with program operation.
#include <IRremote.hpp>
#include <NHY3274TH.h>

#pragma endregion LIBRARIES


#define DECODE_NEC // defines the type of IR transmission to decode based on the remote. See IRremote library for examples on how to decode other types of remote

#define left 0x8
#define right 0x5A
#define up 0x18
#define down 0x52
#define ok 0x1C
#define cmd1 0x45
#define cmd2 0x46
#define cmd3 0x47
#define cmd4 0x44
#define cmd5 0x40
#define cmd6 0x43
#define cmd7 0x7
#define cmd8 0x15
#define cmd9 0x9
#define cmd0 0x19
#define star 0x16
#define hashtag 0xD

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
PIN ASSIGNMENTS
*/
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#pragma region PIN ASSIGNMENTS

// MOTOR PINS
#define MOTOR_1_PIN_1 9 // Motor 1 pins control the card-throwing flywheel direction.
#define MOTOR_1_PIN_2 10
#define MOTOR_1_PWM 11  // Motor 1 PWM controls the speed of the flywheel.
#define MOTOR_2_PWM 5   // Motor 2 PWM controls the speed of rotation for the (yaw) motor.
#define MOTOR_2_PIN_2 6 // Motor 2 pins control the yaw motor direction.
#define MOTOR_2_PIN_1 7
#define FEED_SERVO_PIN 4 // This pin controls the card feeding servo motor.

// BUTTON PINS
#define BUTTON_PIN_4 14 // Button 4 ("Back").
#define BUTTON_PIN_3 15 // Button 3 ("Decrease").
#define BUTTON_PIN_2 16 // Button 2 ("Increase/Hit").
#define BUTTON_PIN_1 17 // Button 1 ("Advance/Pass").

// SENSOR PINS
#define CARD_SENS 2 // IR sensor that determines whether or not a card has passed through the mouth of DEALR.

// OTHER PINS
#define STNDBY 8 // Standby needs to be pulled HIGH. This can be done with a wire to 5V as well.

#pragma endregion PIN ASSIGNMENTS
#pragma region LIB OBJECTS

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
LIBRARY OBJECT ASSIGNMENTS:
Initialize objects for external libraries.
*/
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

Servo feedCard;                                    // Instantiate a Servo object called "feedCard" for controlling a servo motor.
NHY3274TH sensor;                                  // Instantiate an NHY3274TH object called "sensor" for interfacing with the NHY3274TH color sensor.
Adafruit_AlphaNum4 display = Adafruit_AlphaNum4(); // Instantiate an Adafruit_AlphaNum4 object called "display" for controlling the 14-segment display.

#pragma endregion LIB OBJECTS

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
ENUMERATIONS:
Convenient data types for managing DEALR's state machine and animations.
*/
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#pragma region ENUMERATIONS

// DEAL STATE: Tracks DEALR's operational states
enum dealState
{
  IDLE,                     // Enter this state when we are in menus or finished a game.
  INITIALIZING,             // Enter this state when we want to initialize our rotational direction to the red tag.
  DEALING,                  // Enter this state when we want to deal a single card.
  ADVANCING,                // Enter this state when we want to advance from one tag to the next.
  AWAITING_PLAYER_DECISION, // Enter this state when we're pausing for a player input.
  RESET_DEALR,              // Enter this state when we want to reset dealr completely, including all of its state machine flags.
};

// DISPLAY STATE: Tracks what is displayed on the 14-segment screen.
enum displayState
{
  INTRO_ANIM,    // Very first blinking animation that occurs on boot.
  SELECT_GAME,   // Displays the select game menu.
  SELECT_TOOL,   // Displays the tools menu.
  SELECT_CARDS,  // Displays the number of cards selection menu.
  DEAL_CARDS,    // Controls what is displayed when dealing a card.
  ERROR,         // Displays "EROR" when an error happens.
  STRUGGLE,      // Struggling face.
  LOOK_LEFT,     // Looking left face.
  LOOK_RIGHT,    // Looking right face.
  LOOK_STRAIGHT, // Open eyes face.
  FLIP,          // Display state for the word "FLIP".
};

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
ANIMATIONS:
The "DisplayAnimation" struct allows for new animations to be made more easily.
*/
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

const DisplayAnimation *currentAnimation = &initialBlinking;
uint8_t currentFrameIndex = 0;
unsigned long lastFrameTime = 0;

#pragma endregion ENUMERATIONS
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
CONSTANTS:
Fixed values such as motor speeds, timeouts, and default thresholds.
*/
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#pragma region CONSTANTS

// EEPROM VARIABLES
// These variables store the memory addresses of any information that needs to be saved through system reboots
#define EEPROM_VERSION_ADDR 0
#define EEPROM_VERSION 1
#define UV_THRESHOLD_ADDR (numColors * sizeof(RGBColor) + 2)

// GAMES INCLUDED
const uint8_t numGames = 6;            // Number of *index positions* for pre-programmed games (meaning "number of games" - 1). If you add a game, increment this number.
const char gamesMenu[][16] PROGMEM = { // "16" defines the max number of characters you can use in these game titles.
    "1-GO FISH",
    "2-21",
    "3-CRAZY EIGHTS",
    "4-WAR",
    "5-HEARTS",
    "6-RUMMY",
    "*7-TOOLS"};

// TOOL MENUS INCLUDED
const uint8_t numToolMenus = 4;        // Number of *index positions* for pre-programmed tuning routines (so "number of tool menus" - 1). If you add or subtract one, change this number.
const char toolsMenu[][16] PROGMEM = { // "26" defines the max number of characters you can use in these menu titles.
    "*1-DEAL CARD",                    // Deals a single card (useful for debugging card dealing)
    "*2-SHUFFLE DECK",                 // Deals cards alternately into two piles, which can be stacked or further shuffled.
    "*4-COLOR TUNER",                  // Place tags under sensor to "reset" color values for each tag
    "*6-RESET COLORS"};                // Resets color and UV values to factory defaults

// STARTING STATES AND STATE UPDATE TAGS:
dealState currentDealState = IDLE;             // Current state of the dealing interaction, starting with IDLE on boot.
dealState previousDealState;                   // Previous state of the dealing interaction. Necessary for detecting a change in deal states.
displayState currentDisplayState = INTRO_ANIM; // Current state of the display, starting with SCROLL_PLACE_TAGS_TEXT animation.
displayState previousDisplayState;             // Previous state of the display. Necessary for detecting a change in display state.

// MOTOR CONTROL CONSTANTS
// The Card Dealing Robot uses its own DC motor for its rotary axis. As a hack, you can also use either of the motors from the Line Following Domino Robot.
// To use one of those motors instead (which can be capable of higher speeds, at the risk of lower color reading accuracy), simply uncomment the following line:
// #define USE_DOMINO_MOTOR
#ifdef USE_DOMINO_MOTOR
const uint8_t highSpeed = 150;   // Default value for "high speed" movement (max is 255).
const uint8_t mediumSpeed = 140; // Default value for "medium speed" movement.
const uint8_t lowSpeed = 110;    // Low speed is a little above half speed. Any lower and torque is so low that the rotation stalls.
#else
const uint8_t highSpeed = 255;   // Default value for "high speed" movement (max is 255).
const uint8_t mediumSpeed = 220; // Default value for "medium speed" movement.
const uint8_t lowSpeed = 180;    // Low speed is a little above half speed. Any lower and torque is so low that the rotation stalls.
#endif
const uint8_t flywheelMaxSpeed = 255; // Default to top speed for flywheel motor

// TIMING CONSTANTS
const unsigned long errorTimeout = 6000;    // For rotations where we should have found a tag, but didn't, we throw an error after this amount of time.
const unsigned long throwExpiration = 4000; // If, when trying to deal a card, we take longer than this amount of time, throw an error.
const unsigned long reverseFeedTime = 400;  // Amount of time to reverse the feed servo after a deal (successful or unsuccessful).
#pragma endregion CONSTANTS
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
GLOBAL VARIABLES
*/
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#pragma region GLOBAL VARIABLES

// GAMEPLAY VARIABLES
int8_t currentGame = 0;            // Variable for holding the current game being selected.
int8_t previousGame = -1;          // Variable for holding the previous game that was selected (to detect change). We initialize to an impossible number.
int8_t currentToolsMenu = 0;       // Variable for holding the current tools routine being selected.
int8_t previousToolsMenu = -1;     // Variable for holding the previous tools menu selected. We initialize to an impossible number.
uint8_t numberOfPlayers = 4;       // Variable for holding the number of players that will play a game. Used in Tagless deals.
uint8_t numberOfCards = 1;         // Variable for holding the number of cards that should be dealt in the main deal of different games.
uint8_t initialRoundsToDeal = 0;   // Assigned when a game is selected and used as a multiplier to determine how many revolutions to make.
uint8_t remainingRoundsToDeal = 0; // Decrements as we deal out cards each round.
int8_t postCardsToDeal = 52;       // Stores number of cards to deal in the post-game, and decrements per hand.
int8_t cardsInHandToDeal = 0;      // This variable stores the current number of cards left to deal in tagless deals.
uint8_t numTags = 0;               // This variable holds how many tags we discover during checkForTags360().
uint8_t totalCardsToDeal = 0;

// FEED SERVO CONTROL
int8_t slideStep = 0;          // These are the steps the feed servo follows when dealing a card (advance, retract, stop).
int8_t previousSlideStep = -1; // Used to detect when slideStep changes. Initialize to an impossible number.

// VARIABLES RELATED TO TIMINGS (DEBOUNCES, TIMEOUTS, AND TAGS)
unsigned long overallTimeoutTag = 0;        // Tag for marking last human interaction. After a while, we can start the blinking screensaver.
unsigned long errorStartTime = 0;           // For logging when we start dealing a card, so we know if too long has elapsed and there's an error.
unsigned long throwStart = 0;               // Tag for when we start dealing a card.
unsigned long retractStartTime = 0;         // Variable for storing when we begin retracting a card during a throw error.
unsigned long lastDealtTime = 0;            // Variable for storing the last time a card was dealt.
unsigned long initializationStart = 0;      // Variable for storing when initialization began, so if we exceed that amount of time we can throw an error.
unsigned long lastRiggedCheck = 0;          // Tag for checking whether or not the rigged switch has been flipped.
unsigned long checkForTags360StartTime = 0; // Used during checkForTags360 to make sure we go the full way around looking for tags.
unsigned long expressionStarted = 0;        // Variable for storing the start time of any given expression
unsigned long adjustStart = 0;              // Tag for when a fine adjustment process starts after seeing a burst of unknown color
unsigned long lastSignalTime = 0;           // Stores last received signal time
unsigned long lastDealTime = 0;             // Stores the last time a card was dealt

// TEXT AND ANIMATION TIMINGS
uint16_t scrollDelayTime = 0;     // Variable for switching between scrolling and waiting intervals.
unsigned long lastScrollTime = 0; // Tag for when we last shifted text over in scrolling animations.
int scrollIndex = -1;             // Start at -1 to hold the first frame longer.
uint8_t messageRepetitions = 0;   // Variable for storing the number of times we have repeated scrolling text.
char message[36];                 // We use "char" to save RAM. This is the max scroll text length, but it can be increased if necessary.
int8_t messageLine = 0;           // For messages that scroll several lines, this variable holds which line we're scrolling.

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
COLOR MANAGEMENT STRUCT AND VARIABLES
The color sensor outputs red, green, blue, and "brightness" (c) values, which we package into a "struct" and use in different functions.
*/
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

struct RGBColor
{
  uint16_t r;
  uint16_t g;
  uint16_t b;
  uint16_t avgC;
};

// DEFAULT COLOR VALUES (can be updated with onboard color tuning function)
const int8_t numColors = 5; // The total number of available colors (red, green, blue, yellow, and black)
RGBColor colors[numColors]; // Declare an array of RGBColor objects to store color values for the color tuner.
const RGBColor defaultColors[numColors] = {
    {58, 149, 48, 118},  // Black
    {121, 105, 29, 255}, // Red
    {73, 157, 25, 255},  // Yellow
    {35, 132, 89, 255},  // Blue
    {41, 173, 41, 255}   // Green
};

// VARIABLES FOR FINDING AND DEBOUNCING COLOR READINGS
uint8_t activeColor = 0;                  // This is the color the sensor is currently seeing. There can be some "wobble" as we transition between colors, so this needs processing.
uint8_t previousActiveColor = -1;         // Initialize to a value that is not possible so that activeColor != previousActiveColor on boot
uint8_t stableColor = 0;                  // The stable color detected, which is what we get after processing "activeColor" a bit by averaging it over time.
float totalColorValue = 0;                // Variable for holding the value of all detected colors (R, G, and B) added together.
const int8_t numSamples = 10;             // Number of samples for averaging color value.
const uint8_t debounceCount = 3;          // Number of consecutive readings to confirm a color. More readings increases precision, but covers more radial distance. Too many readings can exceed tag width.
uint8_t colorBuffer[debounceCount] = {0}; // Buffer to store the last few colors.

// COLOR-MANAGING ARRAYS AND VARIABLES
int8_t colorsSeen[numColors] = {-1};                 // Array to record colors present, which holds colors seen in their default order.
int8_t colorsSeenIndexValue = 0;                     // Index for colorsSeen array. Ordered the same every time according to RGBColor array.
uint8_t activeColorIndexValue = 0;                   // Variable for holding the index value of the active color.
const int8_t maxTagColors = 4;                       // Variable for holding total number of tag colors.
int8_t colorStatus[maxTagColors] = {-1, -1, -1, -1}; // -1 means color not seen, 0 or more means "seen" and indicates number of cards dealt to the player of that tag.
int8_t colorLeftOfDealer = 0;                        // To store the color index number for the player left of the dealer, which can change each game.

#pragma endregion GLOBAL VARIABLES
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
STATE MACHINE FLAGS:
This code uses a state machine to establish what DEALR should be doing at any point (e.g., dealing, advancing, waiting for player decision). Some
of the state machine logic is controlled by the "enums" above, which iterate through these broader states. But there are lots of smaller, specific
states that can be handled by "boolean operators," or simple true/false variables. We toggle certain values "on" and "off" to enable and disable
these states, like whether the game is rigged or not. We also use these bools to set and check other conditions, like which direction DEALR is spinning,
whether or not a deal has been initialized, or whether or not we are checking for marked cards, to name just a few examples. There are *lots* of states
to check, but using bools is an easy way to both set and check different dealing conditions on the fly.
*/
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#pragma region STATE MACHINE FLAGS

bool rotatingCW = false;                   // Indicates clockwise rotation.
bool rotatingCCW = false;                  // Indicates counter-clockwise rotation.
bool stopped = true;                       // Indicates when rotation is stopped. We should start stopped.
bool correctingCCW = false;                // Indicates when a fine-adjust correction is being made CCW.
bool correctingCW = false;                 // Indicates when a fine-adjust correction is being made CW.
bool dealInitialized = false;              // Indicates we have initialized to the red tag and are ready to deal.
bool throwingCard = false;                 // Indicates we're currently throwing a card with flywheel.
bool cardDealt = false;                    // Indicates whether or not a card has been dealt.
bool numCardsLocked = false;               // Indicates confirmation of the number of cards in a selected game.
bool numPlayersLocked = false;             // Indicates confirmation of the number of players in game. Used in tagless deal.
bool blinkingAnimationActive = false;      // Indicates if the blinking animation is active.
bool newDealState = false;                 // Indicates if a change deal state has just taken place.
bool tagsPresent = false;                  // Indicates when a search for colored tags has yielded tags or not.
bool baselineExceeded = false;             // Indicates when a spike in color data has been seen.
bool fineAdjustCheckStarted = false;       // Indicates when a fine-adjust on a colored tag has started.
bool initialAnimationInProgress = false;   // Indicates when the start animation is in progress.
bool initialAnimationComplete = false;     // Indicates whether the initial pre-game animation has been completed.
bool scrollingStarted = false;             // Indicates the beginning of a text-scrolling operation.
bool scrollingComplete = false;            // Indicates the end of a text-scrolling operation.
bool cardLeftCraw = false;                 // Indicates when a card has exited the mouth of DEALR.
bool startCheckingForMarked = false;       // Indicates the beginning of a checking-for-card-mark proceedure.
bool notFirstRoundOfDeal = false;          // Indicates whether or not it's currently a round of deal *after* the first.
bool buttonInitialization = false;         // Indicates whether or not a button has been pressed yet.
bool advanceOnePlayer = false;             // Indicates whether we're currently advancing one player at a time (like in the post-deal portion of Go Fish).
bool gameOver = false;                     // Indicates that a game is over and we should fully reset.
bool postDeal = false;                     // Indicates a main deal is over and post-deal has begun.
bool scrollingMenu = false;                // Indicates whether or not we're currently scrolling menu text.
bool insideDealrTools = false;             // Indicates whether or not we're inside one of the DEALR "tools," like Deal a Single Card or Shuffle.
bool toolsMenuActive = false;              // Indicates whether we're in the "games" or "tools" menus. "False" represents the "games" menu, and "true" the tools menu.
bool shufflingCards = false;               // Indicates whether or not we're currently shuffling cards.
bool rotatingBackwards = false;            // Indicates whether or not we're dealing in reverse (useful in rigged games).
bool postDealRemainderHandled = false;     // Indicates whether or not we've successfully dealt the post-deal remaining cards, like in Rummy.
bool rotationTimeChecked = false;          // Indicates whether or not we have checked how long it takes to rotate one full turn from red tag to red tag.
bool errorInProgress = false;              // Indicates whether or not an error is detected to be in progress.
bool CW = true;                            // A convenience variable. Now we can say "rotate(CW)" instead of "rotate(true)"
bool CCW = false;                          // A convenience variable. Now we can say "rotate(CCW)" instead of "rotate(false)"
bool cardInCraw = true;                    // "Card-In-Craw" means there is a card in the mouth of the DEALR. The IR sensor reads "high" when not active and "low" when a card is in the beam.
bool previousCardInCraw = true;            // Flag for holding previous card-in-craw state
bool currentlyPlayerLeftOfDealer = false;  // Indicates when we're currently looking at the player left of dealer.
bool playerLeftOfDealerIdentified = false; // Indicates whether or not we've found the player left of dealer.
bool postDealStartOnRed = false;           // Indicates if post-deal starts on red or on player left of dealer. Left of dealer is most common, but starting on red is useful in some rigged games.
bool handlingFlipCard = false;             // Indicates whether or not we're currently handling the "flip card" in some games that flip a card after main deal.
bool currentlyCheckingTags = false;        // True when DEALR is rotating around looking to see what tags are present.
bool adjustInProgress = false;             // Indicates whether or not we're currently fine adjusting to confirm the color of the tag we're looking at.
bool irControlled = false;                 // Tracks if IR is controlling the robot

#pragma endregion STATE MACHINE FLAGS
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
FUNCTION PROTOTYPES
Function Prototypes let a program know what functions are going to be defined later on. This isn't always necessary in every IDE, but it's good practice, and
can serve as a kind of table of contents for what to expect later.
*/
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#pragma region FUNCTION PROTOTYPES

// State Machine Functions
void checkState();                      // Function for constantly checking the current dealing state of DEALR.
void handleIdleState();                 // Handles what happens in the "IDLE" dealing state.
void handleAdvancingState();            // Handles how to proceed when advancing from one player to another.
void handleInitializingStateInAdjust(); // Handles when a fine-adjustment is called during initialization to the red tag.
void handleAdvancingStateInAdjust();    // This is where advancing decisions happen. We have detected an "unknown color," then fine-adjusted to see what color it is.
void handleDealingState();              // Handles what happens when we enter the "dealing" state.
void handleAdvancingOnePlayer();        // This function is used during post-deals when we only want to advance a single player during post-deal.
void handleAwaitingPlayerDecision();    // Displays the correct scroll text during "Awaiting Player Decision" deal state.
void handleResetDealrState();           // Handles what happens when we enter the "reset" state.
inline void dealAndAdvance();           // A convenient wrapper function for dealing and then advancing

// Helper functions for handleDealingState
void handleStandardGameDecisionsAfterFineAdjust(); // This is where dealing decisions are made after DEALR has fine-adjusted and confirmed the color of a tag.
void handleStandardGameRemainderCards();           // This is where decisions are made regarding "remainder" cards, or cards that are dealt after a main deal completes.
void handlePostDealGameplay();                     // This is where decisions are made during a post-deal, or the cards that are dealt after a main deal completes.
void handleToolsDealing();                         // This is where we handle dealing decisions in DEALR's "tools" subroutines
void handleToolsAdvancingDecisions();              // This is where we decide which direction to advance in during DEALR's "tools" subroutines

// Functions related to dealing cards
void dealSingleCard();        // Safe wrapper function for cardDispensingActions. Includes some pre- and post-processing steps.
void cardDispensingActions(); // The series of actions that deal a single card.
void prepareForDeal();        // The steps that get us ready to deal a single card.

// Functions related to DEALR rotation and color sensing
void initializeToRed();                                      // Function for initializing to the red tag before a deal.
void fineAdjustCheck();                                      // fineAdjustCheck() starts after a "bump" in color is seen by the color sensor. It instructs Motor 2 to backtrack slowly until it can confirm the exact color.
void handleRotationAdjustments();                            // Handles switching directions when doing a "fine adjustment", no matter what direction we were going.
void moveOffActiveColor(bool rotateClockwise);               // Function that moves us to a part of the circle that doesn't have tags in it, like during a card flip operation. Takes a direction to rotate in as an input.
void returnToActiveColor(bool rotateClockwise);              // Function that moves us back onto the active color after having moved off.
void colorScan();                                            // Wrapper function for color scanning functions.
bool checkForColorSpike(uint16_t c, uint16_t blackBaseline); // Bool that checks to see if the color sensor detects a "spike" in color value with respect to the baseline

// Funcations related to gameplay mechanics
void handleFlipCard(); // Moves to an unused area, displays "FLIP", and then deals a card.
void handleGameOver(); // Handles when "game over" has been declared by initiating a reset.

// Buttons and Other Sensor Function Prototypes
void checkButton(int buttonPin, unsigned long &lastPress, int &lastButtonState,
                 unsigned long &pressTime, bool &longPressFlag, uint16_t longPressDuration,
                 void (*onRelease)(), void (*onLongPress)());
void checkButtons();                    // Wrapper function for checkButton. Checks whether or not buttons have been pressed.
void onButton1Release();                // Function for isolating when button one is released.
void onButton2Release();                // Function for isolating when button two is released.
void onButton3Release();                // Function for isolating when button three is released.
void onButton4Release();                // Function for isolating when button four is released.
void resetTagsOnButtonPress();          // Convenience function that resets some state machine tags on each button press.
void pollCraw();                        // Checks the IR sensor to see whether or not a card is in the mouth ("craw") of DEALR. Useful for determining whether or not cards have been successfully dealt.
void colorRead(uint16_t blackBaseline); // Function for using the color-reading sensor to detect color underneath it.
uint16_t calculateBlackBaseline();      // Retrieves the RGB value of "black" from EEPROM. We can compare readings against this to quickly detect spikes in brightness indicating tags.
void logBlackBaseline();                // Reads RGB values of "black" for storing to EEPROM.
int8_t numColorsSeen();                 // This function returns the number of color tags that have been seen in a rotation.

// Display-related function prototypes
void showGame();                                            // Function for writing the game being selected onto the display.
void showTool();                                            // Function for writing the tools menu being selected onto the display.
void showCards();                                           // Function for producing the card-number selection menu in games where a user must select number of cards per person.
void showPlayers();                                         // Function for producing the card-number selection menu in games where a user must select number of players playing.
void startScrollText(const char *text, uint16_t start,      // Starts scrolling text. Inputs are: text to scroll, length of time to hold while starting...
                     uint16_t delay, uint16_t end);         // ...scroll frame delay time, and length of time to hold while ending.
void updateScrollText();                                    // Updates scrolling text as we loop.
void stopScrollText();                                      // Interrupts and stops text from scrolling
void displayFace(const char *word);                         // Function for showing a single four-character word (or image) on the display.
void scrollMenuText(const char *text);                      // Helper function that receives text from "showGame()" and "showTool()."
void updateDisplay();                                       // Function that's called when we want to update the display.
void displayErrorMessage(const char *message);              // Displays an error message, then attempts to reset DEALR.
void getProgmemString(const char *progmemStr, char *buffer, // Helper function for progmem.
                      size_t bufferSize);
void runAnimation(const DisplayAnimation &animation);

// UI Manipulation Function Prototypes
void advanceMenu();     // Function for progressing when Button 1 (green) has been pressed.
void goBack();          // Function for regressing when Button 4 (red button) has been pressed.
void decreaseSetting(); // Function for what happens when Button 2 (blue) is pressed.
void increaseSetting(); // Function for what happens when Button 3 (yellow) is pressed.

// Motor Control Functions
void slideCard();                                   // Function for sliding a card into the flywheel for dealing.
void rotate(uint8_t rotationSpeed, bool direction); // Function for rotating. Takes speed and direction as inputs.
void rotateStop();                                  // Function for stopping rotation.
void flywheelOn(bool direction);                    // True = "forward"; False = "reverse".
void flywheelOff();                                 // Turns flywheel off.
void switchRotationDirection();                     // Reverses direction of yaw rotation.

// Tools and Their Helper Functions
// void colorTuner();                 // Controls the "color tuning" operation that locks down RGB values for specific color tags.
void recordColors(int startIndex); // Helper function for colorTuner().
void resetEEPROMToDefaults();      // Function for resetting EEPROM values to defaults.

// Error-and-Timeout-handling Functions
void handleThrowingTimeout(unsigned long currentTime); // Handles timeouts while dealing cards.
void handleFineAdjustTimeout();                        // Handles timeout for fine adjustment moves.
void resetFlags();                                     // Resets all state machine flags when called.
void resetColorsSeen();                                // Function used in the "reset" tool to reset colors seen.
void checkRotationTimeout();

// Bools for Evaluating Rigged Hands
void colorDetected(uint8_t colorIndex);             // Uses the index value of the color seen to increment the number of cards in that player's hand.
bool allHandsExceptActiveFull(uint8_t activeColor); // Returns "true" if all hands except currently active hand are full. Useful at the end of a deal during rigged games.
bool isHandFull(uint8_t activeColor);               // Returns "true" if hand of currently active color is full. Useful for not overdealing hands during rigged games.
bool allSeenColorsFull();                           // Returns "true" if all scanned colors are full. Allows us to end a main deal during rigged games.

// Reading and Writing to EEPROM Functions
void initializeEEPROM();                            // Function for checking whether EEPROM values were set by the user or are factory defaults.
void writeColorToEEPROM(int index, RGBColor color); // Used for writing RGB values to EEPROM.
void loadColorsFromEEPROM();                        // Whatever colors have been saved to EEPROM get loaded at startup using this function.
RGBColor readColorFromEEPROM(int index);            // Helper function used in loadColorsFromEEPROM.
RGBColor getBlackColorFromEEPROM();                 // Lets us check the value of our "baseline" luminance when no tag visible.

#pragma endregion FUNCTION PROTOTYPES
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
SETUP FUNCTION
In the setup for this build, we make sure our sensors are working and run a few startup routines.
*/
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#pragma region SETUP

void setup()
{
  if (useSerial)
  {
    Serial.begin(115200);
    Serial.println(F("Beginning HP_DEALR_2_2_4 02/2025"));
  }

  if (sensor.begin())
  {
    if (useSerial)
    {
      Serial.println(F("Found NHY3274TH sensor"));
    }
  }
  else
  {
    if (useSerial)
    {
      Serial.println(F("No NHY3274TH sensor found ... check your connections"));
    }
  }
  delay(300);

  // COLOR SENSOR ATTACHMENT
  /*
  A NOTE ON COLOR SENSING:

  The DEALR robot rotates over colored tags and reads the different colors as it goes in order to know what player it's facing. The more time it
  is able to "read" each tag--i.e. the longer the integration time--the more accurate the results. This creates an interesting dynamic, because we
  also want to rotate as fast as possible in order to deal faster. Accordingly, we must strike a balance between integration time, tag width, and
  rotation speed.

  Default integration time is set to 0x1, which equates to 8ms. It takes DEALR about 4300 ms to make a full revolution at max speed, and the sensor
  covers about 387mm in linear distance in this time, with each tag being about 10.7mm in arc length, or 1/36th of the circle. So we spend a little
  over 120ms rotating over each color, and so should in theory be able to read each tag 15 times. Realistically, as the sensor crosses from black to red,
  for example, the numerical values need to interpolate from full black to full red, so only the middle 10 or so readings will reflect the actual
  color. We take a running average of these readings to get even more accurate results.

  If we jump to the next sensor level, 33ms per reading, we get much greater color accuracy, but are only able to read each tag three times before we've
  zoomed past it. This leads to a much greater chance that we'll fully skip tags. And so we settle for lower color accuracy with faster read times.
  Because accuracy is lower, it benefits us to sometimes perform a check we've called "fineAdjustCheck." When we see a spike in color data ("c" value, or brightness),
  we stop the rotation motors, then *reverse* for a moment to check the color at a slower speed. This helps us bridge the gap between accuracy and speed.
  */

  sensor.begin();                 // Start color sensor
  sensor.setIntegrationTime(0x1); // Sets the integration time of the NHY3274TH sensor. 0x0 = 2ms; 0x1 = 8ms; 0x2 = 33ms; 0x3 = 132ms
  sensor.setGain(0x20);           // Sets gain of color sensor

  // PIN ASSIGNMENTS
  pinMode(MOTOR_1_PIN_1, OUTPUT);      // Assign "Motor_1_pin_1" as an output
  pinMode(MOTOR_1_PIN_2, OUTPUT);      // Assign "Motor_1_pin_2" as an output
  pinMode(MOTOR_2_PIN_1, OUTPUT);      // Assign "Motor_2_pin_1" as an output
  pinMode(MOTOR_2_PIN_2, OUTPUT);      // Assign "Motor_2_pin_2" as an output
  pinMode(MOTOR_1_PWM, OUTPUT);        // Assign "Motor_1_pwm" as the PWM pin for motor 1
  pinMode(MOTOR_2_PWM, OUTPUT);        // Assign "Motor_2_pwm" as the PWM pin for motor 2
  pinMode(STNDBY, OUTPUT);             // Assign the "standby" pin as an output so we can write it HIGH to prevent the module from sleeping
  pinMode(BUTTON_PIN_1, INPUT_PULLUP); // Set "Button_Pin_1" as a pull-up input
  pinMode(BUTTON_PIN_2, INPUT_PULLUP); // Set "Button_Pin_2" as a pull-up input
  pinMode(BUTTON_PIN_3, INPUT_PULLUP); // Set "Button_Pin_3" as a pull-up input
  pinMode(BUTTON_PIN_4, INPUT_PULLUP); // Set "Button_Pin_4" as a pull-up input
  pinMode(CARD_SENS, INPUT);           // Set the card_sens pin as an input
  pinMode(LED_BUILTIN, OUTPUT);        // Set the built-in LED on the Nano as an output

  feedCard.attach(FEED_SERVO_PIN); // Attach the FEED_SERVO_PIN servo as a servo object
  digitalWrite(STNDBY, HIGH);      // The standby pin for the motor driver must be HIGH or the board will sleep
  delay(50);

  for (uint8_t i = 0; i < maxTagColors; i++) // This for-loop resets each of the "colors seen" to -1, setting us up for card-counting next deal
  {
    colorStatus[i] = -1;
  }

  unsigned long seed = 0; // We do some pseudorandom number generation (PRNG) when chaotically dealing, and this "seed" determines the pseudorandom number starting point.

  seed += analogRead(A7); // Normally we would read an unused analog pin, grabbing its randomly fluctuating voltage in order to augment the randomness of our seed value.
                          // In this case, all our analog pins are in use, so we pick the one that fluctuates the most: the UV sensor pin.
  randomSeed(seed);

  display.begin(0x70);      // Initialize the display with its I2C address.
  display.setBrightness(5); // Brightness can be set between 0 and 7.

  resetColorsSeen();

  initializeEEPROM();       // This function checks to see whether EEPROM was set by the user, or is factory defaults, and loads those values.
  loadColorsFromEEPROM();   // This function loads the above EEPROM values.
  calculateBlackBaseline(); // Read the color values for black from EEPROM and sum them to create a baseline for the color black.

  while (digitalRead(CARD_SENS) == LOW) // This while-loop activates if the card-sensing IR circuit is triggered on boot.
  {
    errorInProgress = true;
    while (!scrollingComplete && digitalRead(CARD_SENS) == LOW)
    {
      displayErrorMessage("TUNE IR SENSOR"); // If this error activates, there is either a card in the DEALR's mouth, or we need to tune the IR sensor screw.
    }
    scrollingComplete = false;
    messageRepetitions = 0;
  }
  errorInProgress = false;

  displayFace(" HI ");
  delay(1000);

  currentDealState = IDLE;
  currentDisplayState = INTRO_ANIM;

  IrReceiver.begin(3, ENABLE_LED_FEEDBACK);
  Serial.print(F("Ready to receive IR signals of protocols: "));
  printActiveIRProtocols(&Serial);
  Serial.println(F("at pin 3"));
}
#pragma endregion SETUP
#pragma region LOOP

// MAIN LOOP
void loop()
{
  /*
   * Check if received data is available and if yes, try to decode it.
   */
  if (IrReceiver.decode())
  {

    /*
     * Print a short summary of received data
     */
    IrReceiver.printIRResultShort(&Serial);
    IrReceiver.printIRSendUsage(&Serial);
    if (IrReceiver.decodedIRData.protocol == UNKNOWN)
    { // command garbled or not recognized
      Serial.println(F("Received noise or an unknown protocol"));
      // We have an unknown protocol here, print more info
      IrReceiver.printIRResultRawFormatted(&Serial, true);
    }
    Serial.println();

    /*
     * !!!Important!!! Enable receiving of the next value,
     * since receiving has stopped after the end of the current received data packet.
     */
    IrReceiver.resume(); // Enable receiving of the next value

    /*
     * Finally, check the received data and perform actions according to the received command
     */

    switch (IrReceiver.decodedIRData.command)
    { // this is where the commands are handled

    case up: // program your own command for up!
      if (useSerial)
      {
        Serial.println(F("Up pressed."));
      }
      break;

    case down: // program your own command for down!
      if (useSerial)
      {
        Serial.println(F("Down pressed."));
      }
      break;

    case left: // fast counterclockwise rotation
    {
      rotatingCW = true;
      rotate(highSpeed, CW);
      irControlled = true;
      lastSignalTime = millis(); // Update last signal time
      scrollingMenu = false;
      currentDisplayState = LOOK_RIGHT;
      updateDisplay();
    }
    break;

    case right: // Fast clockwise rotation
    {
      rotatingCCW = true;
      rotate(highSpeed, CCW);
      irControlled = true;
      lastSignalTime = millis(); // Update last signal time
      scrollingMenu = false;
      currentDisplayState = LOOK_LEFT;
      updateDisplay();
    }
    break;

    case ok: // Firing routine (deals a card)
    {
      unsigned long dealTimeout = 200; // 200ms delay between deals

      if (millis() - lastDealTime > dealTimeout) // Only allow if timeout passed
      {
        Serial.println(F("OK pressed."));
        dealSingleCard();
        cardDealt = false;
        irControlled = true;
        lastDealTime = millis(); // Update last deal time
      }
      else
      {
        Serial.println(F("Ignored command: Too soon after last deal."));
      }

      currentDealState = IDLE;
      currentDisplayState = SELECT_GAME;
      updateDisplay();
    }
    break;
    }
  }
  delay(5);

  checkState();   // This function checks what state the DEALR is in (idle, dealing, awaiting player input, etc.) and lets the dealr behave accordingly.
  checkButtons(); // This function checks to see if buttons are being pressed
  checkRotationTimeout();
}

#pragma endregion LOOP
#pragma region FUNCTIONS
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
GAMEPLAY FLOW AND DEAL STATE HANDLING FUNCTIONS
"checkState" is actually a wrapper function for a series of "handle" functions that control different dealing states.
*/
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#pragma region State Handling

void checkRotationTimeout()
{
  unsigned long timeoutDuration = 250; // Stop after 250ms without input

  if (irControlled && (rotatingCCW || rotatingCW) && (millis() - lastSignalTime > timeoutDuration))
  {
    rotateStop();
    rotatingCCW = false; // Make sure we don't rotate backwards next time we move
    irControlled = false;
    Serial.println(F("Rotation stopped due to timeout."));
    currentDealState = IDLE;
    currentDisplayState = SELECT_GAME;
    updateDisplay();
  }
}

void checkState() // Always running to make sure DEALR does the right things in the right states.
{
  if (currentDealState != previousDealState) // If we change from one state to another, this block lets us do anything that should only happen once during that transition
  {
    newDealState = true;
    previousDealState = currentDealState;
  }

  if (gameOver) // At any point, we can set "gameOver" to "true" and the handleGameOver function will help us exit cleanly.
  {
    handleGameOver();
  }

  if (errorInProgress) // At any point, we can set "errorInProgress" to "true" and enter into the RESET_DEALR state.
  {
    errorInProgress = false;
    currentDealState = RESET_DEALR;
    updateDisplay();
  }

  switch (currentDealState)
  {
  case DEALING: // DEALING handles the process of dealing one or more cards.
    handleDealingState();
    break;

  case ADVANCING: // ADVANCING handles movement from one color tag to the next.
    handleAdvancingState();
    break;

  case INITIALIZING: // INITALIZING handles moving to "red" when starting deal.
    initializeToRed();
    break;

  case IDLE: // IDLE handles what happens when card dealer resets, is in a menu, or isn't in use.
    handleIdleState();
    break;

  case AWAITING_PLAYER_DECISION: // APD handles what happens when we're waiting for a player to make a decision.
    handleAwaitingPlayerDecision();
    break;

  case RESET_DEALR:
    handleResetDealrState(); // Handles resetting state flags when exiting a game or dealing with an error.
    break;
  }
}

void handleDealingState() // Handles what happens when we enter the "dealing" state.
{
  if (postDeal)
  {
    handlePostDealGameplay(); // In this version of the code we don't distinguish between rigged and unrigged post-deals.
    return;
  }

  if (!dealInitialized) // If we command a "deal," but we're not initialized, initialize to red first.
  {
    initializeToRed();
    return;
  }

  if (dealInitialized && !cardDealt) // If we command a "deal" and are initialized but haven't yet dealt a card, deal a card.
  {
    if (toolsMenuActive) // If we're using one of the tools menu tools, handle that separately.
    {
      handleToolsDealing();
    }
    else
    {
      dealSingleCard();
    }
  }

  if (dealInitialized && cardDealt)
  {
    cardDealt = false;

    if (toolsMenuActive) // If toolsMenu is active, we will never advance to either rigged or straight game dealing operations
    {
      handleToolsAdvancingDecisions();
      return;
    }

    {
      currentDealState = ADVANCING;
    }
  }
}

void dealSingleCard()
{
  static uint8_t consecutiveDeals = 0; // Static variable to track consecutive deals. In some circumstances we can deal several cards in a row, but may want to limit how many.

  if (currentDisplayState == LOOK_LEFT || currentDisplayState == LOOK_RIGHT)
  {
    currentDisplayState = LOOK_STRAIGHT;
    updateDisplay();
  }

  if (consecutiveDeals < 3) // When "chaotically dealing" in rigged games, we can deal several cards in a row, but want to avoid dealing more than three in a row (suspicious).
  {
    while (!cardDealt)
    {
      cardDispensingActions();
      if (errorInProgress)
      {
        return;
      }
    }
    flywheelOff();
  }
  else
  {
    // Serial.println(F("Consecutive deal limit reached. ADVANCING."));
    consecutiveDeals = 0;
    currentDealState = ADVANCING;
    return;
  }
}

void cardDispensingActions() // This is the series of things that actually have to happen to dispense a single cards.
{
  unsigned long currentTime = millis();

  if (errorInProgress) // If there's an error in progress, get us out of the dispensing function.
  {
    // Serial.println("Error in progress!");
    return;
  }

  if (!throwingCard) // If we're not yet throwing a card, fire up the motors that allow us to throw a card.
  {
    prepareForDeal();
  }
  else
  {
    pollCraw();  // Continuously poll the craw (card-sensing IR sensor) to see if card goes through.
    slideCard(); // If slieCard executes fully, cardDealt = true

    if (currentTime - expressionStarted > expressionDuration && !handlingFlipCard) // Display DEALR's struggling face for at least "expressionDuration" amount of time
    {
      currentDisplayState = STRUGGLE;
      updateDisplay();
    }
  }

  handleThrowingTimeout(currentTime); // If we're trying to throw a card but too much time elapses, throw an error and exit to a main menu.
}

void prepareForDeal() // Turns on the flywheel.
{
  unsigned long currentTime = millis();

  throwingCard = true; // Set throwingCard tag to "true".
  flywheelOn(true);    // Run flywheel forward.

  delay(100);               // Time for flywheel to speed up.
  cardLeftCraw = false;     // Flag for whether or not the card has exited the DEALR's mouth.
  throwStart = currentTime; // Tag the start of the throw to track deal time-out.
  slideStep = 0;            // If starting new deal, reset feed motor switch case step to 0.
  previousSlideStep = -1;   // Reset previousSlideStep to be different from slideStep.
}

void initializeToRed() // When we start dealing, we don't know where we are, so we rotate until we find "red" first.
{
  unsigned long currentTime = millis();

  if (!rotatingCCW && !adjustInProgress) // If we're not rotating CCW AND we're not fine-adjusting, start rotating top speed CCW.
  {
    rotate(highSpeed, CCW);
    initializationStart = currentTime; // Mark the start of our initialization state with a timestamp.
  }

  colorScan();

  if (baselineExceeded && !adjustInProgress) // If color spikes, run a check to see what color we saw. Schedule this by setting adjustInProgress to true.
  {
    adjustStart = millis();
    adjustInProgress = true;
  }

  if (adjustInProgress == true) // Here is where we check to see what color we actually saw.
  {
    fineAdjustCheck();
  }

  if (currentTime - initializationStart > errorTimeout) // If it takes too long for us to initialize, throw and error.
  {
    rotateStop();
    errorStartTime = currentTime;
    currentDisplayState = ERROR;
    currentDealState = IDLE;
    updateDisplay();
  }
}

void handleAdvancingState() // Executes when currentDealState = ADVANCING.
{
  if (newDealState) // In this block we can do anything we only want to do each time we enter the "advancing" state from a different state
  {
    adjustInProgress = false;
    newDealState = false;
    if (activeColor != 0)
    {
      previousActiveColor = activeColor; // As we start moving away from a color, log it as the "previous color" so we can track the difference as we advance.
    }

    if (!postDealRemainderHandled && !postDeal && !shufflingCards) // If it's not the post-deal (etc), check if there are rounds left to deal.
    {
      if (remainingRoundsToDeal == 0) // If there are no more rounds to deal, set "postDeal" to true and enter into that stae.
      {
        postDeal = true;
        rotatingBackwards = false; // In post-deal we always rotate forwards, so in any mode that has us rotating backwards, switch to forward rotation.
      }
    }

    if (rotatingBackwards)
    {
      moveOffActiveColor(CCW);
    }
    else
    {
      moveOffActiveColor(CW);
    }
  }

  if (activeColor == 0 && !adjustInProgress) // While we're seeing black and not adjusting, rotate at high speed until we hit the next color spike.
  {
    if (rotatingBackwards)
    {
      rotate(highSpeed, CCW);
    }
    else
    {
      rotate(highSpeed, CW);
    }
  }

  colorScan();

  if (baselineExceeded && !adjustInProgress) // If color spikes, run fineAdjustCheck. This "adjustment" helps us reverse and re-check the color in case we accidentally zoomed past the tag.
  {
    adjustStart = millis();
    adjustInProgress = true;
  }

  if (adjustInProgress == true)
  {
    fineAdjustCheck(); // This fineAdjustCheck makes sure we correctly read the colored tag.
    delay(10);         // Slight smoothing delay
  }
}

void fineAdjustCheck() // fineAdjustCheck() starts after a bump in color is seen. It instructs Motor 2 to backtrack until it confirms the color.
{
  unsigned long currentTime = millis(); // Update time

  if (!fineAdjustCheckStarted) // the first time we enter fineAdjust, set adjustStart = to millis()
  {
    adjustStart = currentTime;
    fineAdjustCheckStarted = true;
    if (rotatingCCW)
    {
      rotate(lowSpeed, CCW);
    }
    else
    {
      rotate(lowSpeed, CW);
    }
  }

  while (activeColor < 1) // While we're seeing no tags (black), rotate at low speed to detect what color we saw spike.
  {
    colorScan();
    handleRotationAdjustments(); // Handles which direction we correct towards.
    handleFineAdjustTimeout();
  }

  rotateStop(); // At this point we have a stable active color. We stop and then make decisions about whether or not we deal a card, depending on game states.

  if (!dealInitialized)
  {
    handleInitializingStateInAdjust(); // If we're still in the initializing phase before a deal, this handles decision-making after the fine-adjust.
  }

  else
  {
    handleAdvancingStateInAdjust(); // If we're in either a main deal or a tools function, this handles decision-making after the fine-adjust.
  }
}

void handleInitializingStateInAdjust() // Handles when a fine-adjustment is called during initialization to the red tag.
{
  if (activeColor > 1 && colorStatus[activeColor - 1] == -1) // If we're seeing a non-red color that has not yet been seen (a *new, non-red color*)...
  {
    adjustStart = millis();
    while (activeColor != 0) // Active color wasn't red, but we're looking for red to initialize! So we keep rotating, first until we hit black again, and then we carry on.
    {
      colorScan();
      rotate(highSpeed, CCW);
    }
    fineAdjustCheckStarted = false;
    adjustInProgress = false;
    correctingCW = false;
    correctingCCW = false;
  }
  else if (activeColor == 1) // At this point we have stabilized on the red tag. Flip the dealInitialized tag to "true" so we can carry on to the main deal.
  {
    // Serial.println(F("Deal Initialized!"));
    previousActiveColor = activeColor;
    rotateStop();
    dealInitialized = true;
    adjustStart = millis();
    fineAdjustCheckStarted = false;
    adjustInProgress = false;
    correctingCW = false;
    correctingCCW = false;
    resetColorsSeen();            // This is probably redundant since we should have reset colorsSeen before the deal.
    notFirstRoundOfDeal = false;  // We have reset and are initialized and about to commence the first round of a deal.
    currentDealState = ADVANCING; // In most cases, after initializing to red, this line switches us into the ADVANCING state so we move off the red tag to the player left of dealer.
  }
}

void handleAdvancingStateInAdjust() // This function handles the decision-making on how we advance after a color has been confirmed following the fineAdjustCheck process.
{
  adjustInProgress = false;       // Resets a tag used in the fine adjustment operation
  fineAdjustCheckStarted = false; // Resets a tag used in the fine adjustment operation
  correctingCW = false;           // Resets a tag used in the fine adjustment operation
  correctingCCW = false;          // Resets a tag used in the fine adjustment operation

  if (advanceOnePlayer)
  {
    handleAdvancingOnePlayer(); // If we were only supposed to advance one player during a post-deal, we run this function.
    return;
  }

  if (activeColor == 1 && previousActiveColor == 1 && !postDeal) // If we've done a full circle and hit red a second time in a row, we know we're missing tags! Throw an error.
  {
    errorInProgress = true;
    while (!scrollingComplete)
    {
      displayErrorMessage("EROR");
    }
    scrollingComplete = false;
    messageRepetitions = 0;
    currentDealState = RESET_DEALR;
    return;
  }
  handleStandardGameDecisionsAfterFineAdjust();
}

void handleRotationAdjustments() // This function handles switching directions when doing a "fine adjustment", no matter what direction we were going.
{
  if (rotatingCW && !correctingCW && !correctingCCW) // If we were spinning CW when fineAdjustCheck was called, and we were not already making a fine adjust correction...
  {
    rotateStop(); // Stop the rotation.
    delay(100);
    correctingCW = false;
    correctingCCW = true;
    rotate(lowSpeed, CCW); // Travel counter-clockwise at low speed. Elsewhere we also read the color sensor to get a more accurate reading.
  }
  else if (rotatingCCW && !correctingCW && !correctingCCW) // If we were spinning CCW when fineAdjustCheck was called, and we were not already correcting...
  {
    rotateStop(); // Stop the rotation.
    delay(100);
    correctingCCW = false;
    correctingCW = true;
    rotate(lowSpeed, CW); // Travel clockwise at low speed. Elsewhere we also read the color sensor to get a more accurate reading.
  }
}

void handleStandardGameDecisionsAfterFineAdjust() // This function handles decision-making in non-rigged games after a color has been confirmed.
{
  if (activeColor == 1) // We don't note the color "red" during initialization, so if the color we just hit is red, then we must have made our first full rotation.
  {
    if (colorStatus[activeColor - 1] == -1) // If this is the first time we're seeing red (again)...
    {
      colorStatus[activeColor - 1] = 0; // Note in the colorStatus array that this color has been seen by changing its value from -1 to 0.
      notFirstRoundOfDeal = true;       // We also note that we are no longer in the first round of dealing, which matters in some games.
    }
  }
  else if (activeColor > 1) // If we're seeing a non-red color in a non-rigged game...
  {
    if (!playerLeftOfDealerIdentified) // If we haven't already noted the color of the player left of the dealer, this is our chance.
    {
      playerLeftOfDealerIdentified = true;
      colorLeftOfDealer = activeColor; // We mark the color of the player left-of-dealer so later in the post-game we can allow them to play first.
    }

    if (currentGame == 3 && previousActiveColor != 1) // If we're playing War, and we hit two non-reds in a row, we know there are too many tags. (Traditionally war only has 2 players.)
    {
      errorInProgress = true;
      while (!scrollingComplete)
      {
        displayErrorMessage("EROR");
      }
      scrollingComplete = false;
      messageRepetitions = 0;
      currentDealState = RESET_DEALR;
      updateDisplay();
      return;
    }

    if (activeColor == colorLeftOfDealer && notFirstRoundOfDeal) // If we're looking at player-left-of-dealer after first card has been dealt
    {
      // Serial.println(F("Currently player left of dealer."));

      remainingRoundsToDeal--;

      // Serial.print("Rounds to deal = ");
      // Serial.println(remainingRoundsToDeal);
      if (remainingRoundsToDeal == 0 && !postDeal)
      {
        postDeal = true;
      }
    }
    else // In almost all non-rigged circumstances...
    {
      if (postDealRemainderHandled)
      {
        currentDealState = AWAITING_PLAYER_DECISION;
        return;
      }
    }
  }
  currentDealState = DEALING;
}

void handleStandardGameRemainderCards() // This function handles how community cards are distributed after a main deal.
{
  if (currentGame == 2 || currentGame == 5) // Crazy Eights and Rummy have a card that is flipped after the main deal.
  {
    handleFlipCard(); // This function handles the dealing of an extra card and the populating of the word "FLIP" on the display.
  }
  else
  {
    postDealRemainderHandled = true; // The post-deal remainder refers specifically to "community cards" that are dealt after the main deal.
  }
}

void handlePostDealGameplay() // This function handles the post-deal portion of games that do not end after the main deal.
{
  if ((currentGame == 3 || currentGame == 4) && !toolsMenuActive) // War and Hearts have no post-deal, so we toggle "gameOver" immediately after the main deal to exit cleanly.
  {
    gameOver = true;
    return;
  }

  postDealStartOnRed = false;
  while (activeColor < 2 && previousActiveColor != 1) // Return to non-red color to the left of red:
  {
    colorScan();
    rotate(mediumSpeed, CW);
  }
  rotateStop();

  if (!postDealRemainderHandled) // If we haven't flipped a card for Crazy Eights or Rummy, handle this remainder. Otherwise, mark remainder as handled.
  {
    handleStandardGameRemainderCards();
  }
  else
  {
    currentDealState = AWAITING_PLAYER_DECISION;
  }
}

inline void dealAndAdvance() // This is a convenient wrapper function for dealing a card and then advancing.
{
  dealSingleCard();
  currentDealState = ADVANCING;
}

void handleToolsDealing() // This function is specifically *only* dealing, not advancing, which is why it's comparatively simple.
{
  if (currentToolsMenu == 0) // If we're using the "deal single card tool" and are in the "dealing" state, simply deal a card.
  {
    dealSingleCard();
  }
  else if (currentToolsMenu == 1 || currentToolsMenu == 2) // These are the "separate marked cards" and "shuffle cards" tools. The other tools are more programmatic and have their own sections.
  {
    dealSingleCard();
  }
}

void handleToolsAdvancingDecisions() // This function deals with "advancing" decisions when using the first three tools.
{
  if (currentToolsMenu == 0)
  {
    displayFace("DELT");
    delay(1000);
    currentDealState = IDLE;
    currentDisplayState = SELECT_TOOL;
    insideDealrTools = false;
    updateDisplay();
  }
  else if (currentToolsMenu == 1)
  {
    switchRotationDirection();
    currentDealState = ADVANCING;
    adjustStart = millis();
  }
  else if (currentToolsMenu == 2)
  {
    currentDealState = ADVANCING;
  }
}

void handleResetDealrState() // Handles what happens when we enter the "reset" state.
{
  resetFlags();
  currentGame = 0;
  currentToolsMenu = 0;
  displayFace("EXIT");
  delay(1000);
  buttonInitialization = false;
  toolsMenuActive = false;

  initialAnimationComplete = false;
  currentDisplayState = SELECT_GAME;
}

void handleIdleState() // Handles what happens in the "IDLE" dealing state.
{
  if (newDealState == true)
  {
    newDealState = false;
    flywheelOff();
    if (!stopped)
    {
      rotateStop();
    }
    if (dealInitialized == true)
    {
      dealInitialized = false; // Reset initialization (re-initialize each game).
    }
    resetColorsSeen();
    postDeal = false;
    notFirstRoundOfDeal = false;
    postCardsToDeal = 52;
    cardDealt = false;
    numCardsLocked = false;
    correctingCW = false;
    correctingCCW = false;
  }

  updateDisplay();

  if (slideStep != 0) // Ensures that the feed servo is primed to deal a card.
  {
    previousSlideStep = -1;
    slideStep = 0;
  }
}

void handleGameOver() // Handles when "game over" has been declared by initiating a reset.
{
  moveOffActiveColor(CW); // Rotate clockwise
  gameOver = false;
  toolsMenuActive = false; // Switching to select game menu. Deactivating tool menu.
  currentDealState = RESET_DEALR;
  currentDisplayState = SELECT_GAME;
  updateDisplay();
}

void moveOffActiveColor(bool rotateClockwise) // Function that moves DEALR off the active tag, ideally into an empty portion of the circle.
{
  while (activeColor != 0)
  {
    if (errorInProgress)
    {
      return;
    }
    colorScan();
    if (rotateClockwise)
    {
      rotate(lowSpeed, CW);
    }
    else
    {
      rotate(lowSpeed, CCW);
    }
  }
  rotateStop();
}

void returnToActiveColor(bool rotateClockwise) // Function that returns us to last active color.
{
  while (activeColor == 0)
  {
    if (errorInProgress)
    {
      return;
    }
    colorScan();
    if (rotateClockwise)
    {
      rotate(lowSpeed, CW);
    }
    else
    {
      rotate(lowSpeed, CCW);
    }
  }
  rotateStop();
}

void handleFlipCard() // Moves to an unused area, displays "FLIP", and then deals a card.
{
  handlingFlipCard = true;
  moveOffActiveColor(CCW);
  delay(20);
  currentDisplayState = FLIP;
  updateDisplay();
  dealSingleCard();
  cardDealt = false;
  postDealRemainderHandled = true;
  returnToActiveColor(CW);
  handlingFlipCard = false;
}

void handleAdvancingOnePlayer() // This function is used during post-deals when we only want to advance a single player at a time.
{
  while (activeColor < 1) // While we're looking at black, rotate and scan.
  {
    rotate(mediumSpeed, CW);
    colorScan();
  }
  rotateStop();

  if (postDealStartOnRed) // If our post-deal dealt its first card to red, decrement whenever we reach red again.
  {
    if (activeColor == 1)
    {
      cardsInHandToDeal--;
    }
  }
  else
  {
    if (activeColor > 1 && previousActiveColor == 1) // If our post-deal dealt its first card to the player left-of-dealer, decrement whenever we pass red.
    {
      cardsInHandToDeal--;
    }
  }

  advanceOnePlayer = false;
  adjustStart = millis();
  fineAdjustCheckStarted = false;
  adjustInProgress = false;
  correctingCW = false;
  correctingCCW = false;
  currentDealState = AWAITING_PLAYER_DECISION;
}
#pragma endregion State Handling

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
BUTTONS AND SENSOR-HANDLING FUNCTIONS
*/
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#pragma region Buttons and Sensors

void checkButton(int buttonPin, unsigned long &lastPress, int &lastButtonState,
                 unsigned long &pressTime, bool &longPressFlag, uint16_t longPressDuration,
                 void (*onRelease)(), void (*onLongPress)()) // This demanding function handles everything related to button-pushing in DEALR.
{

  int currentButtonState = digitalRead(buttonPin); // Read the current button state.

  if (currentButtonState == LOW) // If the button has been pressed...

  {
    if (lastButtonState == HIGH) // ... and if the button wasn't already being pressed...
    {
      pressTime = millis(); // Record press time.
    }

    if (!longPressFlag && millis() - pressTime >= longPressDuration) // Check for long press.
    {
      longPressFlag = true;
      if (onLongPress != nullptr)
      {
        resetTagsOnButtonPress();
        onLongPress(); // Trigger the long press action.
      }
    }
  }

  if (lastButtonState == LOW && currentButtonState == HIGH) // Handle button release.
  {
    lastPress = millis();

    if (!longPressFlag && onRelease != nullptr) // If not a long press, trigger the normal release action.
    {
      resetTagsOnButtonPress();
      onRelease(); // Trigger the release action.
    }
    longPressFlag = false; // Reset the long-press flag after release
  }

  lastButtonState = currentButtonState; // Update the last button state
}

void onButton1Release() // This function handles what happens when we release Button 1 (green). It matters that the action happens on "release" so we can use long-click in some cases.
{
  if (currentDealState == AWAITING_PLAYER_DECISION) // If we're playing a game and awaiting a player decision, do one of these things based on the game:
  {
    advanceOnePlayer = true;
    currentDealState = ADVANCING;
    if (currentGame == 0) // If we're playing Go Fish:
    {
      currentDisplayState = LOOK_LEFT;
    }
    else if (currentGame == 1) // If we're playing 21:
    {
      currentDisplayState = LOOK_LEFT;
    }
    else if (currentGame == 2) // If we're playing Crazy Eights:
    {
      currentDisplayState = LOOK_LEFT;
    }
    else if (currentGame == 5) // If we're playing Rummy:
    {
      currentDisplayState = LOOK_LEFT;
    }
    else if (currentGame == 6) // If we're playing Custom Game:
    {
      lastDealtTime = millis();
      cardsInHandToDeal--;
      colorScan();
    }
    updateDisplay();
  }
  else
  {
    advanceMenu(); // If we're just in one of the menus, Button 1 is the "confirm and advance" button.
  }
}

void onButton2Release() // This function handles what happens when we release Button 2 (blue).
{
  if (currentDealState == AWAITING_PLAYER_DECISION)
  {
    if (currentGame == 0) // If we're playing Go Fish:
    {
      dealSingleCard();
      cardDealt = false;
    }
    else if (currentGame == 1) // If we're playing 21:
    {
      dealSingleCard();
      cardDealt = false;
    }
    else if (currentGame == 2) // If we're playing Crazy Eights:
    {
      dealSingleCard();
      cardDealt = false;
    }
    else if (currentGame == 5) // If we're playing Rummy:
    {
      dealSingleCard();
      cardDealt = false;
    }
    else if (currentGame == 6) // If we're playing Custom Game:
    {
      lastDealtTime = millis();
      dealSingleCard();
      cardDealt = false;
    }
    updateDisplay();
  }

  if (currentDisplayState == SELECT_CARDS || currentDisplayState == SELECT_TOOL || currentDisplayState == SELECT_GAME)
  {
    increaseSetting();
  }
}

void onButton3Release() // This function handles what happens when we release Button 3 (yellow).
{
  if (currentDisplayState == SELECT_CARDS || currentDisplayState == SELECT_GAME || currentDealState == IDLE)
  {
    decreaseSetting();
  }
}

void onButton4Release() // This function handles what happens when we release Button 4 (red).
{
  if (currentDealState != IDLE)
  {
    displayFace("EXIT");
    rotateStop();
    flywheelOff();
    delay(1000);
    currentDealState = RESET_DEALR;
  }
  else
  {
    goBack();
  }
}

void onButton1LongPress()
{
}

void onButton2LongPress()
{
}

void onButton3LongPress()
{
}

void onButton4LongPress()
{
}

void resetTagsOnButtonPress()
{
  overallTimeoutTag = millis();    // Reset tag for overall timeout every time button is pressed.
  scrollDelayTime = 0;             // Force any scrolling text to start scrolling immediately.
  scrollingStarted = false;        // Reset scrollingStarted tag.
  scrollingMenu = false;           // Reset scrolling menu tag each time button is pressed.
  messageLine = 0;                 // Reset line that's scrolling in cases where several lines scroll.
  messageRepetitions = 0;          // Every time a button is pressed, reset messageRepetitions tag.
  scrollIndex = -1;                // Reset the scroll index.
  blinkingAnimationActive = false; // Reset the screensaver animation, since a button press indicates a user is present
}

void checkButtons()
{
  static unsigned long lastPress1 = millis(), lastPress2 = millis(),
                       lastPress3 = millis(), lastPress4 = millis();
  static int lastButtonState1 = HIGH, lastButtonState2 = HIGH,
             lastButtonState3 = HIGH, lastButtonState4 = HIGH;
  static unsigned long pressTime1 = 0, pressTime2 = 0,
                       pressTime3 = 0, pressTime4 = 0;
  static bool longPress1 = false, longPress2 = false,
              longPress3 = false, longPress4 = false;

  // Call the checkButton function for each button
  checkButton(BUTTON_PIN_1, lastPress1, lastButtonState1, pressTime1, longPress1, 3000, onButton1Release, onButton1LongPress);
  checkButton(BUTTON_PIN_2, lastPress2, lastButtonState2, pressTime2, longPress2, 3000, onButton2Release, onButton2LongPress);
  checkButton(BUTTON_PIN_3, lastPress3, lastButtonState3, pressTime3, longPress3, 3000, onButton3Release, onButton3LongPress);
  checkButton(BUTTON_PIN_4, lastPress4, lastButtonState4, pressTime4, longPress4, 3000, onButton4Release, onButton4LongPress);
}

void pollCraw() // Checks the IR sensor to see whether or not a card is in the mouth of DEALR. Useful for determining whether or not cards have been dealt.
{
  static unsigned long lastDebounceTime = 0; // Time when the sensor was last debounced
  const unsigned long debounceInterval = 20; // Debounce interval in milliseconds

  unsigned long currentTime = millis(); // Update time

  if (currentTime - lastDebounceTime >= debounceInterval)
  {
    cardInCraw = digitalRead(CARD_SENS);
    if (cardInCraw != previousCardInCraw) // detect a change in craw sensor
    {
      if (cardInCraw == LOW)
      {
        // Serial.println(F("New card in craw!"));
        cardLeftCraw = false;
        cardDealt = false;
      }
      else // If cardInCraw is newly pulled high (i.e. seeing no cards)
      {
        cardLeftCraw = true;
        // Serial.println(F("Card just left craw."));
      }
      previousCardInCraw = cardInCraw;
    }
    lastDebounceTime = currentTime;
  }
}

void colorScan() // Wrapper function for grabbing the black value from EEPROM and comparing it to color reading data from colorRead.
{
  uint16_t blackBaseline = calculateBlackBaseline(); // Retrieve the stored brightness of "no tag" (i.e. black) in order to compare color spikes.
  colorRead(blackBaseline);                          // Read color every loop (with respect to the brightness of "black") as we advance towards the next color.
}

void colorRead(uint16_t blackBaseline) // Checks the color sensing board to see what color we're looking at, and assigns that to be "activeColor".
{
  static uint8_t stableColorCount = 0; // Counter to track stability of color readings
  static uint8_t bufferIndex = 0;      // Index for the circular buffer

  uint16_t r, g, b, c;
  sensor.getRawData(&r, &g, &b, &c);
  totalColorValue = r + g + b;

  // Serial.print("Red: ");
  // Serial.print(r);
  // Serial.print(" Green: ");
  // Serial.print(g);
  // Serial.print(" Blue: ");
  // Serial.print(b);
  // Serial.print(" White: ");
  // Serial.println(c);

  baselineExceeded = checkForColorSpike(c, blackBaseline);

  float normalizedR = r / totalColorValue * 255.0;
  float normalizedG = g / totalColorValue * 255.0;
  float normalizedB = b / totalColorValue * 255.0;

  float distances[numColors];
  for (uint8_t i = 0; i < numColors; i++)
  {
    distances[i] = sqrt(pow(normalizedR - colors[i].r, 2) +
                        pow(normalizedG - colors[i].g, 2) +
                        pow(normalizedB - colors[i].b, 2));
  }

  uint8_t closestColor = 0;
  float minDistance = distances[0];
  for (uint8_t i = 1; i < numColors; i++)
  {
    if (distances[i] < minDistance)
    {
      minDistance = distances[i];
      closestColor = i;
    }
  }

  colorBuffer[bufferIndex] = closestColor;
  bufferIndex = (bufferIndex + 1) % debounceCount;

  bool isStable = true;
  for (uint8_t i = 0; i < debounceCount; i++)
  {
    if (colorBuffer[i] != closestColor)
    {
      isStable = false;
      break;
    }
  }

  if (isStable)
  {
    stableColor = closestColor;
    stableColorCount++;
  }
  else
  {
    stableColorCount = 0;
  }

  if (stableColorCount >= debounceCount) // If we're about to update activeColor
  {
    activeColor = stableColor;
    // Serial.print(F("Active color is "));
    // Serial.println(activeColor);
    delay(10);
  }
}

bool checkForColorSpike(uint16_t c, uint16_t blackBaseline)
{
  if (!baselineExceeded && !adjustInProgress && float(c) >= float(blackBaseline) * 1.6)
  {
    // Serial.println(F("Baseline exceeded!"));
    baselineExceeded = true;
  }
  else if (baselineExceeded && float(c) < float(blackBaseline) * 1.6)
  {
    // Serial.println(F("Back below baseline."));
    baselineExceeded = false;
  }
  return baselineExceeded;
}

uint16_t calculateBlackBaseline() // Retrieves the RGB value of "black" from EEPROM
{
  static uint16_t blackBaseline = 0;
  static bool isCalculated = false;

  if (!isCalculated) // If we've never used the Color Tag Tuning Tool, set the default C value of "black" from EEPROM
  {
    RGBColor blackColor = getBlackColorFromEEPROM();
    blackBaseline = blackColor.avgC;
    isCalculated = true;
  }
  return blackBaseline;
}

void logBlackBaseline() // Reads RGB values of "black" for storing to EEPROM
{
  uint32_t totalR = 0, totalG = 0, totalB = 0, totalC = 0;
  for (int j = 0; j < numSamples; j++)
  {
    uint16_t r, g, b, c;
    sensor.getRawData(&r, &g, &b, &c);
    totalR += r;
    totalG += g;
    totalB += b;
    totalC += c;
    delay(10); // Small delay between samples
  }

  uint16_t avgR = totalR / numSamples;
  uint16_t avgG = totalG / numSamples;
  uint16_t avgB = totalB / numSamples;
  uint16_t avgC = totalC / numSamples;

  if (avgC > 255) // Total brightness can be much higher than the max value for a byte (255). To prevent rolling over, clip it at 255.
  {
    avgC = 255;
  }

  float totalColor = avgR + avgG + avgB;
  float percentageRed = avgR / totalColor;
  float percentageGreen = avgG / totalColor;
  float percentageBlue = avgB / totalColor;

  uint8_t proportionRed = round(percentageRed * 255);
  uint8_t proportionGreen = round(percentageGreen * 255);
  uint8_t proportionBlue = round(percentageBlue * 255);
  uint8_t totalLuminance = avgC;

  RGBColor newColor = {proportionRed, proportionGreen, proportionBlue, totalLuminance};
  colors[0] = newColor; // Store the black color at index 0
  writeColorToEEPROM(0, newColor);
}
#pragma endregion Buttons and Sensors

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
DISPLAY-RELATED FUNCTIONS
*/
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#pragma region 14-Segment Display

void showGame() // Displays the currently selected game.
{
  char buffer[30];
  getProgmemString(gamesMenu[currentGame], buffer, sizeof(buffer));
  display.clear();
  if (strlen(buffer) > 4)
  {
    scrollMenuText(buffer);
  }
  else
  {
    scrollingMenu = false;
    for (uint8_t i = 0; i < 4; i++)
    {
      display.writeDigitAscii(i, buffer[i]);
    }
    display.writeDisplay();
  }
  if (currentGame != previousGame)
  {
    previousGame = currentGame;
  }
}

void showTool() // Displays the currently selected tool.
{
  char buffer[28];
  getProgmemString(toolsMenu[currentToolsMenu], buffer, sizeof(buffer));
  display.clear();
  if (strlen(buffer) > 4)
  {
    scrollMenuText(buffer);
  }
  else
  {
    scrollingMenu = false;
    for (uint8_t i = 0; i < 4; i++)
    {
      display.writeDigitAscii(i, buffer[i]);
    }
    display.writeDisplay();
  }
  if (currentToolsMenu != previousToolsMenu)
  {
    previousToolsMenu = currentToolsMenu;
  }
}

void showCards() // Displays the currently selected number of cards.
{
  display.clear();
  String cards = "C=" + String(numberOfCards);
  for (uint8_t i = 0; i < cards.length() && i < 4; i++)
  {
    display.writeDigitAscii(i, cards[i]);
  }
  display.writeDisplay();
}

void showPlayers() // Displays the currently selected number of players.
{
  display.clear();
  String players = "P=" + String(numberOfPlayers);
  for (uint8_t i = 0; i < players.length() && i < 4; i++)
  {
    display.writeDigitAscii(i, players[i]);
  }
  display.writeDisplay();
}

void startScrollText(const char *text, uint16_t start, uint16_t delay, uint16_t end) // Starts scrolling text. Inputs are: text to scroll, length of time to hold while starting, scroll frame delay time, and length of time to hold while ending.
{
  strncpy(message, text, sizeof(message) - 1);
  message[sizeof(message) - 1] = '\0';
  textStartHoldTime = start;
  textSpeedInterval = delay;
  textEndHoldTime = end;
  lastScrollTime = millis();
  scrollingComplete = false;
  scrollIndex = -1; // Reset the scroll index
}

void updateScrollText() // Updates scrolling text as we loop.
{
  unsigned long currentTime = millis(); // Update time

  // Check if it's time to update the display
  if (currentTime - lastScrollTime >= scrollDelayTime)
  {
    lastScrollTime = currentTime; // Update the last scroll time
    if (scrollIndex == -1)
    {
      display.clear();
      display.writeDigitAscii(0, message[0]);
      display.writeDigitAscii(1, message[1]);
      display.writeDigitAscii(2, message[2]);
      display.writeDigitAscii(3, message[3]);
      display.writeDisplay();
      scrollDelayTime = textStartHoldTime; // Set delayTime to hold interval
      scrollIndex++;
    }
    else if (scrollIndex < static_cast<int>(strlen(message)) - 3)
    {
      // Scroll the text
      display.clear();
      display.writeDigitAscii(0, message[scrollIndex]);
      display.writeDigitAscii(1, message[scrollIndex + 1]);
      display.writeDigitAscii(2, message[scrollIndex + 2]);
      display.writeDigitAscii(3, message[scrollIndex + 3]);
      display.writeDisplay();
      scrollDelayTime = textSpeedInterval; // Set delayTime to scroll interval
      scrollIndex++;
    }
    else
    {
      // Hold the last frame for one second
      display.clear();
      display.writeDigitAscii(0, message[strlen(message) - 4]);
      display.writeDigitAscii(1, message[strlen(message) - 3]);
      display.writeDigitAscii(2, message[strlen(message) - 2]);
      display.writeDigitAscii(3, message[strlen(message) - 1]);
      display.writeDisplay();
      scrollDelayTime = textEndHoldTime; // Set delayTime to hold interval
      scrollIndex = -1;                  // Reset the scroll index to start again
      messageRepetitions++;              // How many times has the full message repeated, in messages that only repeat x times before advancing
      messageLine++;                     // When one message has several lines, used to increment through
      scrollingStarted = false;
      if (messageLine > 3)
      {
        messageLine = 0;
      }
      delay(10); // Helps smooth out function?
    }
  }
}

void stopScrollText()
{
  display.clear();
  messageRepetitions = 0;
  messageLine = 0;
  lastScrollTime = millis();
  scrollingStarted = false;
  scrollingComplete = true;
  scrollIndex = -1; // Reset the scroll index
}

void displayFace(const char *word) // Displays a single 4-letter word.
{
  display.clear();
  for (uint8_t i = 0; i < 4; i++)
  {
    if (word[i] != '\0') // Check if the character is not the string terminator
    {
      display.writeDigitAscii(i, word[i]);
    }
  }
  display.writeDisplay();
}

void scrollMenuText(const char *text) // Helper function that receives text from "showGame()" and "showTool()"
{
  if (!scrollingMenu)
  {
    startScrollText(text, 1000, textSpeedInterval, 1000);
    scrollingMenu = true;
  }
  updateScrollText();
}

void updateDisplay() // A catch-all display-updating switch-case that controls what should be displayed on the 14-segment timer when called.
{
  if (currentDisplayState != previousDisplayState)
  {
    scrollingStarted = false;
    scrollingComplete = false;
    messageRepetitions = 0;
    scrollIndex = -1;
    currentFrameIndex = 0;
    lastFrameTime = millis();
    previousDisplayState = currentDisplayState;
    // if (useSerial)
    // {
    //   Serial.print(F("Display state changed to: "));
    //   Serial.println(currentDisplayState);
    // }
  }

  switch (currentDisplayState)
  {
  case INTRO_ANIM:
    if (!initialAnimationComplete)
    {
      runAnimation(initialBlinking);
    }
    else
    {
      advanceMenu();
    }
    break;

  case SELECT_GAME:
    if (stopped)
    {
      showGame();
    }
    break;

  case SELECT_TOOL:
    if (!insideDealrTools)
    {
      showTool();
    }
    break;

  case SELECT_CARDS:
    showCards();
    break;

  case DEAL_CARDS:
    displayFace("DEAL");
    break;

  case ERROR:
    displayFace("EROR");
    delay(1000);
    currentDealState = RESET_DEALR;
    break;

  case STRUGGLE:
    expressionStarted = millis();
    displayFace(EFFORT);
    break;

  case LOOK_LEFT:
    expressionStarted = millis();
    displayFace(LEFT);
    break;

  case LOOK_RIGHT:
    expressionStarted = millis();
    displayFace(RIGHT);
    break;

  case LOOK_STRAIGHT:
    displayFace(LOOK_BIG);
    break;

  case FLIP:
    displayFace("FLIP");
    break;
  }

  if (scrollingMenu)
  {
    updateScrollText();
  }
}

void displayErrorMessage(const char *message) // Displays an error message, then resets DEALR.
{
  if (!scrollingStarted)
  {
    scrollingStarted = true;
    startScrollText(message, 1000, textSpeedInterval, 1000);
  }
  while (!scrollingComplete)
  {
    updateScrollText();
    if (messageRepetitions > 0)
    {
      scrollingComplete = true;
    }
  }
  currentDealState = RESET_DEALR;
  updateDisplay();
}

void getProgmemString(const char *progmemStr, char *buffer, size_t bufferSize) // Helper function for getting strings to store in progmem.
{
  strncpy_P(buffer, progmemStr, bufferSize - 1);
  buffer[bufferSize - 1] = '\0'; // Ensure null-terminated
}

void runAnimation(const DisplayAnimation &animation)
{
  unsigned long currentTime = millis();
  static unsigned long lastAnimationTime = 0;
  static uint8_t currentFrame = 0;
  static const DisplayAnimation *lastAnimation = nullptr;
  static uint8_t lastDisplayedFrame = 255; // Track last displayed frame

  if (lastAnimation != &animation) // If switching animations, reset frame tracking

  {
    // if (useSerial)
    // {
    //   Serial.println(F("New animation detected! Resetting frames."));
    // }
    currentFrame = 0;
    lastAnimation = &animation;
    lastAnimationTime = currentTime; // Ensure immediate update
    lastDisplayedFrame = 255;        // Force the first frame to print

    displayFace(animation.frames[currentFrame]); // Display the first frame immediately
    // if (useSerial)
    // {
    //   Serial.print(F("Frame: "));
    //   Serial.print(animation.frames[currentFrame]);
    //   Serial.print(F(" | Interval: "));
    //   Serial.println(animation.intervals[currentFrame]);
    // }

    lastDisplayedFrame = currentFrame; // Update last displayed frame
    return;                            // Exit to avoid waiting
  }

  // Use the current frame's interval before changing the frame
  if (currentTime - lastAnimationTime >= animation.intervals[currentFrame])
  {
    lastAnimationTime = currentTime; // Update timing BEFORE incrementing frame

    if (&animation == &initialBlinking && currentFrame == animation.numFrames - 1) // If this is the last frame of `initialBlinking`, mark the animation as complete and stop updating
    {
      // if (useSerial)
      // {
      //   Serial.println(F("Initial blinking animation complete!"));
      // }
      currentFrame = 0;
      lastAnimation = nullptr;
      initialAnimationComplete = true;
      initialAnimationInProgress = false;
      updateDisplay();
      return; // Prevent displaying frame 0 again
    }

    currentFrame = (currentFrame + 1) % animation.numFrames; // Advance frame AFTER using the interval

    displayFace(animation.frames[currentFrame]); // Display new frame

    if (currentFrame != lastDisplayedFrame) // Print debug before changing frames
    {
      // if (useSerial)
      // {
      //   Serial.print(F("Frame: "));
      //   Serial.print(animation.frames[currentFrame]);
      //   Serial.print(F(" | Interval: "));
      //   Serial.println(animation.intervals[currentFrame]);
      // }
      lastDisplayedFrame = currentFrame; // Update last displayed frame
    }
  }
}

void handleAwaitingPlayerDecision() // Displays the correct scroll text during "Awaiting Player Decision" deal state.
{
  if (postDeal && postCardsToDeal <= 0) // If there are no more cards pending in a post-deal portion of a game.
  {
    gameOver = true;
    return;
  }

  if (currentGame == 0) // Go Fish
  {
    if (!scrollingStarted)
    {
      if (messageLine % 2 == 0)
      {
        startScrollText("BLUE = FISH", 1000, textSpeedInterval, 1000);
      }
      else
      {
        startScrollText("GREEN = PASS", 1000, textSpeedInterval, 1000);
      }
      scrollingStarted = true;
    }
    updateScrollText();
  }
  else if (currentGame == 1) // 21
  {
    if (!scrollingStarted)
    {
      if (messageLine % 2 == 0)
      {
        startScrollText("BLUE = HIT  ^ ", 1000, textSpeedInterval, 1000);
      }
      else
      {
        startScrollText("GREEN = STAND   ^", 1000, textSpeedInterval, 1000);
      }
      scrollingStarted = true;
    }
    updateScrollText();
  }
  else if (currentGame == 2) // Crazy Eights
  {
    if (!scrollingStarted)
    {
      if (messageLine % 2 == 0)
      {
        startScrollText("BLUE = DRAW  ^ ", 1000, textSpeedInterval, 1000);
      }
      else
      {
        startScrollText("GREEN = PASS   ^", 1000, textSpeedInterval, 1000);
      }
      scrollingStarted = true;
    }
    updateScrollText();
  }
  else if (currentGame == 5) // Rummy
  {
    if (!scrollingStarted)
    {
      if (messageLine % 2 == 0)
      {
        startScrollText("BLUE = DRAW  ^ ", 1000, textSpeedInterval, 1000);
      }
      else
      {
        startScrollText("GREEN = NEXT   ^", 1000, textSpeedInterval, 1000);
      }
      scrollingStarted = true;
    }
    updateScrollText();
  }
  else if (currentGame == 6) // Custom Game
  {
    if (!scrollingStarted)
    {
      if (messageLine % 2 == 0)
      {
        startScrollText("BLUE = DRAW  ^ ", 1000, textSpeedInterval, 1000);
      }
      else
      {
        startScrollText("GREEN = PASS   ^", 1000, textSpeedInterval, 1000);
      }
      scrollingStarted = true;
    }
    updateScrollText();
  }
}
#pragma endregion 14-Segment Display

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
UI MANIPULATION FUNCTIONS
*/
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#pragma region UI Manipulation

void advanceMenu() // Advances menus in UI according to current selection.
{
  // if (useSerial)
  // {
  //   Serial.println(F("Advancing menu."));
  // }
  scrollingStarted = false;
  scrollingComplete = false;
  scrollingMenu = false;
  switch (currentDisplayState)
  {
  case INTRO_ANIM:
    initialAnimationComplete = false;
    buttonInitialization = true;
    toolsMenuActive = false; // Switching to select game menu. Deactivating Tools menu.
    stopped = true;
    currentDisplayState = SELECT_GAME;
    updateDisplay();
    break;

  case SELECT_GAME:
    if (currentGame == 0) // Current game is Go Fish
    {
      currentDisplayState = DEAL_CARDS;
      currentDealState = DEALING;
      remainingRoundsToDeal = 5;
      postCardsToDeal = 126; // Cards to deal after main deal (deal until deck is exhausted). 127 is the max value for a uint8_t.
      initialRoundsToDeal = remainingRoundsToDeal;
    }
    else if (currentGame == 1) // Current game is 21
    {
      currentDisplayState = DEAL_CARDS;
      currentDealState = DEALING;

      remainingRoundsToDeal = 2; // each player starts with 2 cards in 21
      postCardsToDeal = 1;       // Cards to deal after main deal (only one round in 21)
      initialRoundsToDeal = remainingRoundsToDeal;
    }
    else if (currentGame == 2) // Current game is Crazy Eights
    {
      currentDisplayState = DEAL_CARDS;
      currentDealState = DEALING;
      remainingRoundsToDeal = 5; // Crazy 8s deals 5 rounds initially
      postCardsToDeal = 126;     // Cards to deal after main deal (deal until deck is exhausted)
      initialRoundsToDeal = remainingRoundsToDeal;
    }
    else if (currentGame == 3) // WAR
    {
      currentDisplayState = DEAL_CARDS;
      currentDealState = DEALING;
      remainingRoundsToDeal = 52; // War deals the full deck to two people.
      postCardsToDeal = 0;        // There is no post-game in War.
      initialRoundsToDeal = remainingRoundsToDeal;
    }
    else if (currentGame == 4) // Hearts
    {
      currentDisplayState = DEAL_CARDS;
      currentDealState = DEALING;
      remainingRoundsToDeal = 13; // Hearts deals 13/player.
      postCardsToDeal = 0;        // Cards to deal after main deal. Hearts has no post-deal.
      initialRoundsToDeal = remainingRoundsToDeal;
    }
    else if (currentGame == 5) // Rummy
    {
      currentDisplayState = DEAL_CARDS;
      currentDealState = DEALING;
      remainingRoundsToDeal = 7; // For two-player Rummy, this can be increased to 10. Better yet, it could be procedural based on how many tags are seen!
      postCardsToDeal = 126;     // Cards to deal after main deal (deal until deck is exhausted).
      initialRoundsToDeal = remainingRoundsToDeal;
    }
    else if (currentGame == 6) // currentGame is "tools". This takes us into the tools menu.
    {
      toolsMenuActive = true; // Switching to Tools menu; deactivating games menu.
      currentDisplayState = SELECT_TOOL;
    }
    else // Unknown game
    {
      currentDealState = RESET_DEALR;
      currentDisplayState = ERROR;
      updateDisplay();
    }
    break;

  case SELECT_TOOL:
    if (toolsMenuActive && currentToolsMenu == 0) // DEAL SINGLE CARD
    {
      dealInitialized = true; // Initialize wherever we are so we don't have to go looking for the red tag for this tool.
      currentDealState = DEALING;
      remainingRoundsToDeal = 0;
      currentDisplayState = STRUGGLE;
      updateDisplay();
    }
    else if (toolsMenuActive && currentToolsMenu == 1) // SHUFFLE CARDS
    {
      shufflingCards = true;
      remainingRoundsToDeal = 52;
      currentDealState = DEALING;
      currentDisplayState = DEAL_CARDS;
      updateDisplay();
    }
    else if (toolsMenuActive && currentToolsMenu == 2) // SEPARATE MARKED/UNMARKED CARDS
    {
      remainingRoundsToDeal = 52;
      currentDealState = DEALING;
      currentDisplayState = DEAL_CARDS;
      updateDisplay();
    }
    else if (toolsMenuActive && currentToolsMenu == 3) // COLOR TUNER
    {
      // colorTuner();
    }
    else if (toolsMenuActive && currentToolsMenu == 4) // UV TUNER
    {
      // Formerly UV Tuner
    }
    else if (toolsMenuActive && currentToolsMenu == 5) // RESET COLOR VALUES TO DEFAULT
    {
      resetEEPROMToDefaults();
      displayFace("DONE");
      // printStoredColors();
      delay(1500);
      updateDisplay();
      currentDealState = RESET_DEALR;
      updateDisplay();
    }
    insideDealrTools = true;
    break;

  case SELECT_CARDS:
    if (numCardsLocked == false)
    {
      numCardsLocked = true;
      // Serial.print(F("Locked cards number = "));
      // Serial.println(numberOfCards);
      remainingRoundsToDeal = numberOfCards;
      initialRoundsToDeal = remainingRoundsToDeal;
    }

    currentDisplayState = DEAL_CARDS;
    currentDealState = DEALING;
    updateDisplay();
    break;

  case DEAL_CARDS:
    currentDealState = DEALING;
    break;

  case LOOK_STRAIGHT:
    break;

  case LOOK_RIGHT:
    break;

  case LOOK_LEFT:
    break;

  case STRUGGLE:
    break;

  case FLIP:
    break;

  case ERROR:
    break;
  }
}

void goBack() // Returns to prior menu, or exits program.
{
  // if (useSerial)
  // {
  //   Serial.println(F("Going back one menu."));
  // }
  scrollingMenu = false;
  switch (currentDisplayState)
  {

  case DEAL_CARDS:
    currentDisplayState = SELECT_CARDS;
    break;

  case SELECT_CARDS:
    toolsMenuActive = false; // Switching to select game menu. Deactivating Tools menu.
    currentDisplayState = SELECT_GAME;
    break;

  case SELECT_TOOL:
    toolsMenuActive = false; // Switching to select game menu. Deactivating Tools menu.
    currentDisplayState = SELECT_GAME;
    break;

  case SELECT_GAME:
    currentGame = 0;
    currentToolsMenu = 0;
    toolsMenuActive = false;
    buttonInitialization = false;
    scrollingStarted = false;

    initialAnimationComplete = false;
    currentDisplayState = INTRO_ANIM;

    currentDealState = IDLE;
    break;

  case INTRO_ANIM:
    // Nothing happens when we go back, but we can make sure buttonInitialization = false so the screensaver starts.
    buttonInitialization = false;
    break;

  case LOOK_STRAIGHT:
    break;

  case LOOK_RIGHT:
    break;

  case LOOK_LEFT:
    break;

  case STRUGGLE:
    break;

  case FLIP:
    break;

  case ERROR:
    break;
  }
}

void decreaseSetting() // In menus, decrements selected menu option.
{

  if (currentDisplayState == SELECT_CARDS && numberOfCards > 1)
  {
    numberOfCards--;
    // Serial.print(F("Number of cards = "));
    // Serial.println(numberOfCards);
  }
  else if (currentDisplayState == SELECT_GAME)
  {
    currentGame--;
    if (currentGame < 0)
    {
      currentGame = numGames;
    }
  }
  else if (currentDisplayState == SELECT_TOOL)
  {
    currentToolsMenu--;
    if (currentToolsMenu < 0)
    {
      currentToolsMenu = numToolMenus;
    }
  }
}

void increaseSetting() // In menus, increments selected menu option.
{
  if (currentDisplayState == SELECT_CARDS)
  {
    numberOfCards++;
    // Serial.print(F("Number of cards = "));
    // Serial.println(numberOfCards);
  }
  else if (currentDisplayState == SELECT_GAME && currentGame >= 0)
  {
    currentGame++;
    if (currentGame > numGames)
    {
      currentGame = 0;
    }
  }
  else if (currentDisplayState == SELECT_TOOL && currentToolsMenu >= 0)
  {
    currentToolsMenu++;
    if (currentToolsMenu > numToolMenus)
    {
      currentToolsMenu = 0;
    }
  }
}
#pragma endregion UI Manipulation

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
MOTOR CONTROL FUNCTIONS
*/
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#pragma region Motor Control

void slideCard() // This function proceeds through steps to eject a card from DEALR.
{
  unsigned long currentTime = millis(); // Update time
  static unsigned long lastStepTime = 0;

  switch (slideStep)
  {
  case 0: // Start feeding card with feed servo:
    if (slideStep != previousSlideStep)
    {
      previousSlideStep = slideStep;
      feedCard.write(150); // Advances feed motor to feed card towards mouth
      lastStepTime = currentTime;
    }
    if (cardLeftCraw == true) // If the IR sensor sees a card has passed through the mouth of DEALR...
    {
      slideStep = 1;
      cardLeftCraw = false; // Reset this state for the next card to deal.
    }
    break;

  case 1: // Reverses motion to retract the next card into a deal-ready state.
    feedCard.write(30);
    lastStepTime = currentTime;
    slideStep = 2;
    break;

  case 2: // Wait for reverseFeedTime to complete.
    if (slideStep != previousSlideStep && currentTime - lastStepTime >= reverseFeedTime)
    {
      previousSlideStep = slideStep;
      feedCard.write(90); // Neutral position to stop the servo
      lastStepTime = currentTime;
      slideStep = 3;
    }
    break;

  case 3:                                                                    // Stop the servo and complete the slide
    if (slideStep != previousSlideStep && currentTime - lastStepTime >= 100) // Small delay to ensure the servo has stopped.
    {
      previousSlideStep = -1;
      feedCard.write(90);
      slideStep = 0;
      throwingCard = false;
      if (!errorInProgress)
      {
        cardDealt = true;
      }
      overallTimeoutTag = currentTime;
      delay(10);
    }
    break;
  }
}

void rotate(uint8_t rotationSpeed, bool direction) // Rotates CW or CCW at a specified speed.
{
  analogWrite(MOTOR_2_PWM, rotationSpeed);

  if (direction == CW) // CW = "True"
  {
    rotatingCW = true;
    rotatingCCW = false;
    stopped = false;
    digitalWrite(MOTOR_2_PIN_1, HIGH);
    digitalWrite(MOTOR_2_PIN_2, LOW);
    if (millis() - expressionStarted > expressionDuration)
    {
      currentDisplayState = LOOK_LEFT;
    }
  }
  else if (direction == CCW) // CCW = "False"
  {
    rotatingCW = false;
    rotatingCCW = true;
    stopped = false;
    analogWrite(MOTOR_2_PWM, rotationSpeed);
    digitalWrite(MOTOR_2_PIN_1, LOW);
    digitalWrite(MOTOR_2_PIN_2, HIGH);
    if (millis() - expressionStarted > expressionDuration)
    {
      currentDisplayState = LOOK_LEFT;
    }
  }
  updateDisplay();
}

void rotateStop() // Stops yaw rotation and toggles a few rotating states to false.
{
  if (!stopped)
  {
    stopped = true;
    rotatingCW = false;
    rotatingCCW = false;
    digitalWrite(MOTOR_2_PIN_1, LOW);
    digitalWrite(MOTOR_2_PIN_2, LOW);
    delay(20); // Slight delay while motors stop.
  }
}

void flywheelOn(bool direction) // Turns flywheel on. Accepts "true" for forward, "false" for reverse.
{
  if (direction == false)
  {
    analogWrite(MOTOR_1_PWM, flywheelMaxSpeed);
    digitalWrite(MOTOR_1_PIN_1, HIGH);
    digitalWrite(MOTOR_1_PIN_2, LOW);
  }
  else
  {
    analogWrite(MOTOR_1_PWM, flywheelMaxSpeed);
    digitalWrite(MOTOR_1_PIN_1, LOW);
    digitalWrite(MOTOR_1_PIN_2, HIGH);
  }
}

void flywheelOff() // Turns flywheel off.
{
  digitalWrite(MOTOR_1_PIN_1, LOW);
  digitalWrite(MOTOR_1_PIN_2, LOW);
  delay(20);
}

void switchRotationDirection() // Changes direction of yaw rotation.
{
  rotatingBackwards = !rotatingBackwards;
}
#pragma endregion Motor Control

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
DEALR TOOLS FUNCTIONS AND THEIR HELPERS
DEALR has a few "tools" that are used for debugging and tuning. This section includes functions for saving new threshold values to EEPROM for
the specific color values of the specific tags in each box (colorTuner), the specific luminance values of the UV light reflected off any given deck
of cards (uvSensorTuner), as well as a reset to factory defaults function, a "shuffle" function, and "separate marked from unmarked cards" function,
and a "deal a single card" function.
*/
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#pragma region DEALR Tools

// void colorTuner() // Controls the "color tuning" operation that locks down RGB values for color tags, which can experience variation for a bunch of reasons, like room lighting and UV exposure fading.
// {
//   if (!scrollingStarted)
//   {
//     messageRepetitions = 1;
//     scrollingStarted = true;
//     scrollingComplete = false;
//   }

//   int messageCounter = 0;
//   const char *messages[] = {
//       "REMOVE TAGS.",
//       "REPLACE WHEN PROMPTED.",
//       "TO CONFIRM, PRESS GREEN"};
//   const int numMessages = sizeof(messages) / sizeof(messages[0]);

//   while (!scrollingComplete)
//   {
//     if (messageRepetitions >= 1)
//     {
//       messageRepetitions = 0;
//       startScrollText(messages[messageCounter], textStartHoldTime, textSpeedInterval, textEndHoldTime);
//       messageCounter = (messageCounter + 1) % numMessages; // Cycle through messages
//     }
//     updateScrollText();

//     if (digitalRead(BUTTON_PIN_1) == LOW || digitalRead(BUTTON_PIN_2) == LOW || digitalRead(BUTTON_PIN_3) == LOW) // Pressing a button cancels the text animation.
//     {
//       messageCounter = 0;
//       scrollingComplete = true;
//       scrollingStarted = false;
//       while (digitalRead(BUTTON_PIN_1) == LOW || digitalRead(BUTTON_PIN_2) == LOW || digitalRead(BUTTON_PIN_3) == LOW)
//       {
//         // Wait for "confirm" button to be released before proceeding.
//       };
//       break;
//     }
//     else if (digitalRead(BUTTON_PIN_4) == LOW)
//     {
//       messageCounter = 0;
//       currentDealState = RESET_DEALR;
//       updateDisplay();
//       break;
//     }
//   }

//   if (scrollingComplete)
//   {
//     logBlackBaseline();
//     recordColors(1); // Start from index 1 (Red) instead of black.
//     loadColorsFromEEPROM();
//     displayFace("DONE");
//     scrollingStarted = false; // reset in case we want to tune again
//     scrollingComplete = false;
//     insideDealrTools = false;
//     delay(1500);

//     currentDealState = IDLE;
//     toolsMenuActive = false; // Switching to select game menu. Deactivating Tools menu.
//     currentDisplayState = SELECT_GAME;
//     updateDisplay();
//   }
// }

void recordColors(int startIndex) // This function scans, records, and saves color values to EEPROM for colorTuner().
{
  const char *colorNames[numColors] = {"BLAK", "RED ", "YELO", "BLUE", "GREE"};
  unsigned long lastDebounceTime = 0;
  const unsigned long debounceDelay = 50; // 50ms debounce delay
  toolsMenuActive = false;

  for (int i = startIndex; i < numColors; i++)
  {
    unsigned long currentTime = millis();

    displayFace(colorNames[i]);

    bool buttonPressed = false;
    bool buttonReleased = true;

    while (!buttonPressed)
    {
      if (digitalRead(BUTTON_PIN_4) == LOW)
      { // Allows the "back" button to cancel this operation
        currentDealState = RESET_DEALR;
        updateDisplay();
        return;
      }
      if (digitalRead(BUTTON_PIN_1) == LOW && buttonReleased)
      {
        currentTime = millis();
        if ((currentTime - lastDebounceTime) > debounceDelay)
        {
          while (digitalRead(BUTTON_PIN_1) == LOW)
          {
            buttonReleased = false;
            buttonPressed = true;
            lastDebounceTime = currentTime;
            displayFace("SAVD");
            delay(1000);
          }
        }
      }
      else if (digitalRead(BUTTON_PIN_1) == HIGH)
      {
        buttonPressed = false;
        buttonReleased = true;
        lastDebounceTime = currentTime;
      }
    }

    uint32_t totalR = 0, totalG = 0, totalB = 0, totalC = 0;
    for (int j = 0; j < numSamples; j++)
    {
      uint16_t r, g, b, c;
      sensor.getRawData(&r, &g, &b, &c);
      totalR += r;
      totalG += g;
      totalB += b;
      totalC += c;
      delay(10); // Small delay between samples
    }

    uint16_t avgR = totalR / numSamples;
    uint16_t avgG = totalG / numSamples;
    uint16_t avgB = totalB / numSamples;
    uint16_t avgC = totalC / numSamples;

    if (avgC > 255) // Total brightness can be much higher than the max value for a byte (255). To prevent rolling over, clip it at 255.
    {
      avgC = 255;
    }

    float totalColor = avgR + avgG + avgB;
    float percentageRed = avgR / totalColor;
    float percentageGreen = avgG / totalColor;
    float percentageBlue = avgB / totalColor;

    uint8_t proportionRed = round(percentageRed * 255);
    uint8_t proportionGreen = round(percentageGreen * 255);
    uint8_t proportionBlue = round(percentageBlue * 255);
    uint8_t totalLuminance = avgC;

    RGBColor newColor = {proportionRed, proportionGreen, proportionBlue, totalLuminance};
    colors[i] = newColor;
    writeColorToEEPROM(i, newColor);
  }
}

void resetEEPROMToDefaults() // Function for resetting EEPROM values to defaults
{
  for (int i = 0; i < numColors; i++) // Write default values to EEPROM

  {
    writeColorToEEPROM(i, defaultColors[i]);
  }

  EEPROM.write(EEPROM_VERSION_ADDR, EEPROM_VERSION); // Write the version byte to indicate EEPROM has been initialized
}
#pragma endregion DEALR Tools
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
ERROR AND TIMEOUT HANDLING FUNCTIONS
*/
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#pragma region Error_and_Timeout

void handleThrowingTimeout(unsigned long currentTime) // Handles timeouts while dealing cards. If something is taking too long, something's probably going wrong.
{
  static unsigned long retractDuration = 1000;
  static bool retractStarted = false;
  static bool retractCompleted = false;

  if (currentTime - throwStart >= throwExpiration && !retractStarted && !retractCompleted) // For handling dealing timeout.
  {
    retractStarted = true;
    retractStartTime = currentTime;
    feedCard.write(30);
    delay(20); // Smoothing delay
    flywheelOn(false);
    delay(20); // Smoothing delay
  }

  if (retractStarted && currentTime - retractStartTime >= retractDuration)
  {
    retractStarted = false;
    feedCard.write(90);
    delay(20); // Smoothing delay
    flywheelOff();
    delay(20); // Smoothing delay
    retractCompleted = true;
  }

  if (retractCompleted)
  {
    retractCompleted = false;
    if (currentToolsMenu == 1)
    {
      errorInProgress = true;
      shufflingCards = false;
      toolsMenuActive = false;
      currentDisplayState = SELECT_TOOL;
      currentDealState = RESET_DEALR;
    }
    else if (currentToolsMenu == 2)
    {
      errorInProgress = true;
      toolsMenuActive = true;
      currentDisplayState = SELECT_TOOL;
      currentDealState = RESET_DEALR;
    }
    else
    {
      errorInProgress = true;
      currentDisplayState = ERROR;
      currentDealState = RESET_DEALR;
    }
    updateDisplay();
  }
}

void handleFineAdjustTimeout() // Handles timeout for fine adjustment moves.
{
  unsigned long currentTime = millis();

  if (currentTime - adjustStart > errorTimeout && currentDealState != AWAITING_PLAYER_DECISION)
  {
    errorStartTime = currentTime;
    currentDisplayState = ERROR;
    currentDealState = IDLE;
    updateDisplay();
  }
}

void resetFlags() // Resets all state machine flags when called.
{
  adjustStart = millis();
  advanceOnePlayer = false;
  baselineExceeded = false;
  blinkingAnimationActive = false;
  dealInitialized = false;
  cardDealt = false;
  cardLeftCraw = false;
  correctingCCW = false;
  correctingCW = false;
  colorLeftOfDealer = -1;
  currentlyPlayerLeftOfDealer = false;
  currentGame = 0;
  currentToolsMenu = 0;
  errorInProgress = false;
  fineAdjustCheckStarted = false;
  gameOver = false;
  insideDealrTools = false;
  initialAnimationComplete = false;
  numCardsLocked = false;
  numPlayersLocked = false;
  newDealState = false;
  notFirstRoundOfDeal = false;
  numTags = 0;
  overallTimeoutTag = millis();
  postDeal = false;
  initialAnimationInProgress = false;
  postDealRemainderHandled = false;
  postDealStartOnRed = false;
  previousSlideStep = -1;
  previousCardInCraw = 1;
  playerLeftOfDealerIdentified = false;
  rotatingBackwards = false;
  rotationTimeChecked = false;
  resetColorsSeen();
  rotatingCW = false;
  rotatingCCW = false;
  remainingRoundsToDeal = 0;
  stopped = true;
  scrollingMenu = false;
  scrollingStarted = false;
  scrollingComplete = false;
  shufflingCards = false;
  slideStep = 0;
  startCheckingForMarked = false;
  tagsPresent = false;
  throwingCard = false;
  totalCardsToDeal = 0;
  for (uint8_t i = 0; i < maxTagColors; i++)
  {
    colorStatus[i] = -1;
  }
}

void resetColorsSeen() // Function used in the "reset" tool to reset colors seen
{
  memset(colorsSeen, -1, sizeof(colorsSeen));
  colorsSeenIndexValue = 0;
}

#pragma endregion Error_and_Timeout

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
BOOLS FOR EVALUATING THE FULLNESS OF EACH PLAYER'S HAND
*/
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#pragma region Hand Evaluation

void colorDetected(uint8_t activeColor) //
{
  uint8_t activeColorIndex = activeColor - 1; // Adjust the active color index to match the colorStatus array indexing (which excludes black).

  if (colorStatus[activeColorIndex] == -1)
  {
    // Serial.print(F("New color detected! Color is: "));
    // Serial.println(activeColor);
    colorStatus[activeColorIndex] = 0;
  }
}

int8_t numColorsSeen() // When called, returns the total number of colors that have been scanned.
{
  int8_t numColorsSeen = 0; // Initialize to 0
  for (uint8_t i = 0; i < maxTagColors; i++)
  {
    if (colorStatus[i] != -1)
    {
      numColorsSeen++;
    }
  }
  return numColorsSeen; // Return the count after the loop.
}

bool allHandsExceptActiveFull(uint8_t activeColor) // Returns "true" if all hands except currently active hand are full.
{
  uint8_t activeColorIndex = activeColor - 1; // Adjust the active color index to match the colorStatus array indexing (which excludes black).

  for (uint8_t i = 0; i < maxTagColors; i++) // Iterate through all colors except the active color.

  {
    if (i == activeColorIndex) // Skip the active color's index.

      continue;

    if (colorStatus[i] != -1 && colorStatus[i] < initialRoundsToDeal) // Check if the color has been seen and if its hand is not full.

    {
      return false; // Found a color that is seen but not full.
    }
  }
  return true; // All hands except the active color's hand are full.
}

bool isHandFull(uint8_t activeColor) // Returns "true" if the hand of the active color is full.
{
  uint8_t activeColorIndex = activeColor - 1; // Adjust the active color index to match the colorStatus array indexing (which excludes black).

  if (activeColor > 0) // Ensure activeColor is greater than 0 (i.e., not black).

  {
    if (colorStatus[activeColorIndex] != -1 && colorStatus[activeColorIndex] >= initialRoundsToDeal) // Check if the active color has been seen and if its hand is full.

    {
      return true; // The hand is full.
    }
  }
  return false; // The hand is not full or activeColor is black.
}

bool allSeenColorsFull() // Returns "true" if all scanned colors are full.
{
  for (uint8_t i = 0; i < maxTagColors; i++) // Iterate through all colors in the colorStatus[] array.

  {
    if (colorStatus[i] != -1 && colorStatus[i] < initialRoundsToDeal) // Check if the color has been seen and if its hand is not full.

    {
      return false; // Found a color that is seen but not full.
    }
  }
  return true; // All seen colors have full hands.
}
#pragma endregion Hand Evaluation

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
FUNCTIONS FOR READING AND WRITING TO EEPROM
*/
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#pragma region EEPROM

void initializeEEPROM() // Function for checking whether EEPROM values were set by the user or are factory defaults.
{
  if (EEPROM.read(EEPROM_VERSION_ADDR) != EEPROM_VERSION)
  {
    for (int i = 0; i < numColors; i++) // Write default values to EEPROM.

    {
      writeColorToEEPROM(i, defaultColors[i]);
    }
    EEPROM.write(EEPROM_VERSION_ADDR, EEPROM_VERSION); // Write the version byte to indicate EEPROM has been initialized.
  }
}

void writeColorToEEPROM(int index, RGBColor color) // Used for writing RGB values to EEPROM.
{
  int addr = index * sizeof(RGBColor) + 1;
  EEPROM.put(addr, color);
}

void loadColorsFromEEPROM() // Whatever colors have been saved to EEPROM get loaded at startup using this function.
{
  for (int i = 0; i < numColors; i++)
  {
    colors[i] = readColorFromEEPROM(i);
  }
}

RGBColor readColorFromEEPROM(int index) // Helper function used in loadColorsFromEEPROM.
{
  RGBColor color;
  int addr = index * sizeof(RGBColor) + 1;
  EEPROM.get(addr, color);
  return color;
}

RGBColor getBlackColorFromEEPROM()
{
  return readColorFromEEPROM(0); // If we have black stored at index 0, this will retrieve it
}

#pragma endregion EEPROM
#pragma endregion FUNCTIONS
//////////////////////////////////////////////////////////////////////////////////////////////////////////
//END CODE
//////////////////////////////////////////////////////////////////////////////////////////////////////////