#include <Arduino.h>
#pragma region README

/*
 ************************************************************************************
 * Card Dealing Robot Stock Code
 * 10.27.2025
 * Version: 3.0.0
 * Author: Crunchlabs LLC
 *
 *   This firmware powers the DEALR card-dealing robot. It includes six built-in games, an open
 *   mode for dealing any number of cards to any number of players, and a variety of advanced
 *   features, including color tag detection, marked card dealing, and expressive animations.
 *   For more info and to access the hacks, visit the Crunchlabs IDE. For community updates,
 *   check out our Discord server. For help making your own hacks, ask our very own Mark Robot
 *   on the IDE!
 ************************************************************************************
 */

#pragma endregion README

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
#include "NHY3274TH.h"            // Support for the NHY3274TH board
#include "LTR381RGB.h"            // Support for LTR381RGB board

#include "PinDefinitions.h"
#include "Config.h"
#include "Enums.h"

#pragma endregion LIBRARIES
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
LIBRARY OBJECT ASSIGNMENTS:
Initialize objects for external libraries.
*/
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#pragma region LIB OBJECTS

Servo feedCard;                                    // Instantiate a Servo object called "feedCard" for controlling a servo motor.
ColorSensor *activeSensor = nullptr;               // Pointer to the active color sensor (either NHY3274TH or LTR381RGB)
NHY3274TH nhySensor;                               // Instantiate an NHY3274TH object called "nhySensor" for interfacing with the NHY3274TH color sensor.
LTR381RGB ltrSensor;                               // Instantiate an LTR381RGB object called "ltrSensor" for interfacing with LTR381RGB color sensor
Adafruit_AlphaNum4 display = Adafruit_AlphaNum4(); // Instantiate an Adafruit_AlphaNum4 object called "display" for controlling the 14-segment display.

#pragma endregion LIB OBJECTS

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
ANIMATIONS:
The "DisplayAnimation" struct allows for new animations to be made more easily.
*/
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#pragma region ANIMATIONS

const DisplayAnimation *currentAnimation = &initialBlinking;
uint8_t currentFrameIndex = 0;
unsigned long lastFrameTime = 0;

#pragma endregion ANIMATIONS

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
CONSTANTS:
Fixed values such as motor speeds, timeouts, and default thresholds.
*/
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#pragma region CONSTANTS

// EEPROM VARIABLES - These variables store the memory addresses of any information that needs to be saved through system reboots
#define EEPROM_VERSION_ADDR 0
#define EEPROM_VERSION 1
#define UV_THRESHOLD_ADDR (numColors * sizeof(RGBColor) + 2)

// GAMES INCLUDED
const uint8_t numGames = 7;            // Number of *index positions* for pre-programmed games (meaning "number of games" - 1). If you add a game, increment this number.
const char gamesMenu[][16] PROGMEM = { // "16" defines the max number of characters you can use in these game titles.
    "1-GO FISH",
    "2-21",
    "3-CRAZY EIGHTS",
    "4-WAR",
    "5-HEARTS",
    "6-RUMMY",
    "7-CUSTOM GAME", // Custom game is also known as "tagless deal," where all tags except red are removed. User inputs number of cards/players to deal in.
    "*8-TOOLS"};

// TOOL MENUS INCLUDED
const uint8_t numToolMenus = 5;        // Number of *index positions* for pre-programmed tuning routines (so "number of tool menus" - 1). If you add or subtract one, change this number.
const char toolsMenu[][26] PROGMEM = { // "26" defines the max number of characters you can use in these menu titles.
    "*1-DEAL SINGLE CARD",             // Deals a single card (useful for debugging card dealing)
    "*2-SHUFFLE DECK",                 // Deals cards alternately into two piles, which can be stacked or further shuffled.
    "*3-SEPARATE MARKED CARDS",        // Looks for cards that are marked and deals them into a different pile from unmarked cards
    "*4-COLOR TUNER",                  // Place tags under sensor to "reset" color values for each tag
    "*5-UV SENSOR TUNER",              // Deals 5 cards, and takes the highest reflectance value, adds a buffer, and calls that the "marked card threshold"
    "*6-RESET DEFAULT COLORS"};        // Resets color and UV values to factory defaults

// STARTING STATES AND STATE UPDATE TAGS:
dealState currentDealState = IDLE;                         // Current state of the dealing interaction, starting with IDLE on boot.
dealState previousDealState;                               // Previous state of the dealing interaction. Necessary for detecting a change in deal states.
displayState currentDisplayState = SCROLL_PLACE_TAGS_TEXT; // Current state of the display, starting with SCROLL_PLACE_TAGS_TEXT animation.
displayState previousDisplayState;                         // Previous state of the display. Necessary for detecting a change in display state.

// MOTOR CONTROL CONSTANTS
const uint8_t highSpeed = 255;   // Default value for "high speed" movement (max is 255).
const uint8_t mediumSpeed = 220; // Default value for "medium speed" movement.
const uint8_t lowSpeed = 180;    // Low speed is a little above half speed. Any lower and torque is so low that the rotation stalls.

const uint8_t flywheelMaxSpeed = 255; // Default to top speed for flywheel motor

// UV SENSOR CONSTANTS
const uint16_t defaultUVThreshold = 8; // Initial UV value indicating a marked card.
const uint8_t numberOfReadings = 6;    // Number of readings of each marked card to establish an average value.

// TIMING CONSTANTS
const unsigned long riggedCheckInterval = 2000; // How often we poll the rig switch to see if it has been flipped.
const unsigned long min360Interval = 1000;      // This variable ensures there's no chance of double-reading the red tag while initializing a tagless deal.
unsigned long tag2Time = 0;                     // This variable holds the time it takes to get from the red tag to the second tag, which we can use to calculate where the other tags are in a circle for tagless deals.

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
uint8_t consecutiveDeals = 0;      // This is the number of cards that have been dealt to one player consecutively

// FEED SERVO CONTROL
int8_t slideStep = 0;          // These are the steps the feed servo follows when dealing a card (advance, retract, stop).
int8_t previousSlideStep = -1; // Used to detect when slideStep changes. Initialize to an impossible number.

// UV SENSOR VALUES
uint16_t uvReaderValue = 0;                      // Variable for storing reading from UV sensor.
uint16_t storedUVThreshold = defaultUVThreshold; // Value starts as the default value, but can be updated using a built-in tool.

// VARIABLES RELATED TO TIMINGS (DEBOUNCES, TIMEOUTS, AND TAGS)
unsigned long overallTimeoutTag = 0;        // Tag for marking last human interaction. After a while, we can start the blinking screensaver.
unsigned long errorStartTime = 0;           // For logging when we start dealing a card, so we know if too long has elapsed and there's an error.
unsigned long throwStart = 0;               // Tag for when we start dealing a card.
unsigned long retractStartTime = 0;         // Variable for storing when we begin retracting a card during a throw error.
unsigned long rotationTime = 0;             // For logging the amount of time it takes to rotate a full 360 in advance of a tagless deal.
unsigned long taglessRotationInterval = 0;  // For a tagless deal, this variable will be the amount of time it takes to make a full rotation / number of players.
unsigned long lastDealtTime = 0;            // Variable for storing the last time a card was dealt.
unsigned long initializationStart = 0;      // Variable for storing when initialization began, so if we exceed that amount of time we can throw an error.
unsigned long lastRiggedCheck = 0;          // Tag for checking whether or not the rigged switch has been flipped.
unsigned long checkForTags360StartTime = 0; // Used during checkForTags360 to make sure we go the full way around looking for tags.
unsigned long markedLEDStart = 0;           // Tag for the start of the blinking timing for onboard LED (pin 13)
unsigned long expressionStarted = 0;        // Variable for storing the start time of any given expression
unsigned long adjustStart = 0;              // Tag for when a fine adjustment process starts after seeing a burst of unknown color
unsigned long flipDisplayStart = 0;         // Tag for when we started displaying "FLIP" on the screen

// TEXT AND ANIMATION TIMINGS
uint16_t scrollDelayTime = 0;     // Variable for switching between scrolling and waiting intervals.
unsigned long lastScrollTime = 0; // Tag for when we last shifted text over in scrolling animations.
int scrollIndex = -1;             // Start at -1 to hold the first frame longer.
uint8_t messageRepetitions = 0;   // Variable for storing the number of times we have repeated scrolling text.
char message[36];                 // We use "char" to save RAM. This is the max scroll text length, but it can be increased if necessary.
int8_t messageLine = 0;           // For messages that scroll several lines, this variable holds which line we're scrolling.
bool flipDisplayActive = false;   // Indicates whether or not the "FLIP" display is currently active.

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
const RGBColor defaultColors_NHY3274TH[numColors] = {
    // {61, 145, 50, 66},   // Black (matte wood)
    // {106, 108, 41, 209}, // Red (matte wood)
    // {72, 150, 33, 255},  // Yellow (matte wood)
    // {37, 126, 93, 233},  // Blue (matte wood)
    // {44, 159, 52, 186}   // Green (matte wood)

    {65, 82, 108, 56},   // Black (glossy wood)
    {158, 50, 47, 194},  // Red (glossy wood)
    {111, 100, 44, 505}, // Yellow (glossy wood)
    {28, 58, 169, 275},  // Blue (glossy wood)
    {46, 109, 100, 164}  // Green (glossy wood)
};

const RGBColor defaultColors_LTR381RGB[numColors] = {
    {100, 118, 37, 129}, // Black
    {144, 90, 21, 329},  // Red
    {113, 123, 20, 619}, // Yellow
    {60, 107, 88, 281},  // Blue
    {77, 135, 43, 222}   // Green
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
int8_t totalCardsToDeal = 0;                         // Variable for holding number of cards that should be dealt in tagless game.
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
bool stopped = true;                       // Indicates when rotation is stopped.
bool correctingCCW = false;                // Indicates when a fine-adjust correction is being made CCW.
bool correctingCW = false;                 // Indicates when a fine-adjust correction is being made CW.
bool dealInitialized = false;              // Indicates we have initialized to the red tag and are ready to deal.
bool throwingCard = false;                 // Indicates we're currently throwing a card with flywheel.
bool cardDealt = false;                    // Indicates whether or not a card has been dealt.
bool numCardsLocked = false;               // Indicates confirmation of the number of cards in a selected game.
bool numPlayersLocked = false;             // Indicates confirmation of the number of players in game. Used in tagless deal.
bool blinkingAnimationActive = false;      // Indicates if the blinking animation is active.
bool riggedGame = false;                   // Indicates whether a game is rigged or not. False for 'No', true for 'Yes'.
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
bool markedCard = false;                   // Indicates if a card is marked.
bool notFirstRoundOfDeal = false;          // Indicates whether or not it's currently a round of deal *after* the first.
bool buttonInitialization = false;         // Indicates whether or not a button has been pressed yet.
bool advanceOnePlayer = false;             // Indicates whether we're currently advancing one player at a time (like in the post-deal portion of Go Fish).
bool gameOver = false;                     // Indicates that a game is over and we should fully reset.
bool postDeal = false;                     // Indicates a main deal is over and post-deal has begun.
bool scrollingMenu = false;                // Indicates whether or not we're currently scrolling menu text.
bool insideDealrTools = false;             // Indicates whether or not we're inside one of the DEALR "tools," like Deal a Single Card or Shuffle.
bool toolsMenuActive = false;              // Indicates whether we're in the "games" or "tools" menus. "False" represents the "games" menu, and "true" the tools menu.
bool separatingCards = false;              // Indicates whether or not we're running the "separating cards" tuning function.
bool shufflingCards = false;               // Indicates whether or not we're currently shuffling cards.
bool rotatingBackwards = false;            // Indicates whether or not we're dealing in reverse (useful in rigged games).
bool postDealRemainderHandled = false;     // Indicates whether or not we've successfully dealt the post-deal remaining cards, like in Rummy.
bool fullExit = false;                     // Indicates, during a reset, that we should return to the pre-game animation.
bool gamesExit = false;                    // Indicates, during a reset, that we should return to the games menu.
bool toolsExit = false;                    // Indicates, during a reset, that we should return to the tools menu.
bool rotationTimeChecked = false;          // Indicates whether or not we have checked how long it takes to rotate one full turn from red tag to red tag.
bool errorInProgress = false;              // Indicates whether or not an error is detected to be in progress.
bool tagsChecked = false;                  // Indicates whether or not we have checked for tags (used in tagless deals and rigged games).
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
bool taglessGame = false;                  // Flag for indicating whether or not it will be a tagless game
bool communityCards = false;               // Indicates whether or not the game has "community cards"
bool timeoutLatched = false;               // Indicates whether or not the overall timeout has been latched.
bool retracting = false;                   // Indicates whether or not we're currently retracting a card during a throw error.
bool retractDone = false;                  // Indicates whether or not the retraction of a card during a throw error is complete.
bool flippingCard = false;                 // Indicates whether or not we're currently flipping a card.
bool toolDealingActive = false;            // Indicates when we're dealing a card within a tool
bool redToTag2IsCW = true;                 // Indicates whether the direction from the red tag to the second tag is clockwise or counterclockwise, which we can use to determine which way to rotate during tagless deals and rigged games.

#pragma endregion STATE MACHINE FLAGS
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
FUNCTION PROTOTYPES
Function Prototypes let a program know what functions are going to be defined later on. This isn't always necessary in every IDE, but it's good practice, and
can serve as a kind of table of contents for what to expect later.
*/
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#pragma region FUNCTION PROTOTYPES

// Setup Functions
void startRoutine(); // Routine that runs in "setup". Includes motor test and initial blink animation.

// State Machine Functions
void checkState();                         // Function for constantly checking the current dealing state of DEALR.
void handleIdleState();                    // Handles what happens in the "IDLE" dealing state.
void handleAdvancingState();               // Handles how to proceed when advancing from one player to another.
void handleInitializingStateInAdjust();    // Handles when a fine-adjustment is called during initialization to the red tag.
void handleAdvancingStateInAdjust();       // This is where advancing decisions happen. We have detected an "unknown color," then fine-adjusted to see what color it is.
void handleDealingState();                 // Handles what happens when we enter the "dealing" state.
void handleTaglessDealState();             // Handles what happens when we enter a "tagless deal" state.
void handleTaglessPlayerPostDealAdvance(); // Handless advance moves in tagless post-game
void handleAdvancingOnePlayer();           // This function is used during post-deals when we only want to advance a single player during post-deal.
void handleAwaitingPlayerDecision();       // Displays the correct scroll text during "Awaiting Player Decision" deal state.
void handleResetDealrState();              // Handles what happens when we enter the "reset" state.
inline void dealAndAdvance();              // A convenient wrapper function for dealing and then advancing

// Helper functions for handleDealingState
void handleStandardGameDecisionsAfterFineAdjust(); // This is where dealing decisions are made after DEALR has fine-adjusted and confirmed the color of a tag.
void handleStandardGameRemainderCards();           // This is where decisions are made regarding "remainder" cards, or cards that are dealt after a main deal completes.
void handlePostDealGameplay();                     // This is where decisions are made during a post-deal, or the cards that are dealt after a main deal completes.
void handleRiggedGameDealing();                    // This is where decisions are made about when to deal and to whom when the game is rigged.
void handleRiggedGameAdvancingDecisions();         // This is where decisions are made about which direction to advance in during a rigged game.
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
void checkRotationTime();                                    // Used during "tagless" deal. We see how long it takes to rotate around in a circle and then divide that up into intervals based on the number of players.
void colorScan();                                            // Wrapper function for color scanning functions.
bool checkForColorSpike(uint16_t c, uint16_t blackBaseline); // Bool that checks to see if the color sensor detects a "spike" in color value with respect to the baseline.

// Funcations related to gameplay mechanics
void handleFlipCard(); // Moves to an unused area, displays "FLIP", and then deals a card.
void handleGameOver(); // Handles when "game over" has been declared by initiating a reset.
void dealTagless();    // Handles "tagless deals", where we use only the red tag for homing, but can deal to any number of players

// Buttons and Other Sensor Function Prototypes
void checkButton(int buttonPin, unsigned long &lastPress, int &lastButtonState,
                 unsigned long &pressTime, bool &longPressFlag, uint16_t longPressDuration,
                 void (*onRelease)(), void (*onLongPress)());
void checkButtons();                                                   // Wrapper function for checkButton. Checks whether or not buttons have been pressed.
void onButton1Release();                                               // Function for isolating when button one is released.
void onButton1LongPress();                                             // Function for isolating when button one is long-pressed.
void onButton2Release();                                               // Function for isolating when button two is released.
void onButton2LongPress();                                             // Function for isolating when button two is long-pressed.
void onButton3Release();                                               // Function for isolating when button three is released.
void onButton3LongPress();                                             // Function for isolating when button three is long-pressed.
void onButton4Release();                                               // Function for isolating when button four is released.
void onButton4LongPress();                                             // Function for isolating when button four is long-pressed.
void resetTagsOnButtonPress();                                         // Convenience function that resets some state machine tags on each button press.
void checkIfRigged();                                                  // Reads the switch on the back of the dealer's head that determines whether games are "rigged" or "unrigged."
bool checkIfMarked();                                                  // Checks if a card is marked and returns "true" or "false" depending on the result.
void pollCraw();                                                       // Checks the IR sensor to see whether or not a card is in the mouth ("craw") of DEALR. Useful for determining whether or not cards have been successfully dealt.
void colorRead(uint16_t blackBaseline);                                // Function for using the color-reading sensor to detect color underneath it.
void getColorData(uint16_t *r, uint16_t *g, uint16_t *b, uint16_t *c); // For selecting which color sensor is in use
void normalizeRaw(uint16_t *r, uint16_t *g, uint16_t *b);              // Helper for normalizing color values over different sensors.
uint16_t calculateBlackBaseline();                                     // Retrieves the RGB value of "black" from EEPROM. We can compare readings against this to quickly detect spikes in brightness indicating tags.
void logBlackBaseline();                                               // Reads RGB values of "black" for storing to EEPROM.
void checkForTags360();                                                // This function spins from red tag all the way around to the red tag again and logs what other tags are seen in what order.
int8_t numColorsSeen();                                                // This function returns the number of color tags that have been seen in a rotation.

// Display-related function prototypes
void showGame();                                            // Function for writing the game being selected onto the display.
void showTool();                                            // Function for writing the tools menu being selected onto the display.
void showCards();                                           // Function for producing the card-number selection menu in games where a user must select number of cards per person.
void showPlayers();                                         // Function for producing the card-number selection menu in games where a user must select number of players playing.
void startPreGameAnimation();                               // Starts the pre-game animation, which transitions into the blinking animation.
void startScrollText(const char *text, uint16_t start,      // Starts scrolling text. Inputs are: text to scroll, length of time to hold while starting...
                     uint16_t delay, uint16_t end);         // ...scroll frame delay time, and length of time to hold while ending.
void updateScrollText();                                    // Updates scrolling text as we loop.
void stopScrollText();                                      // Interrupts and stops text from scrolling
void displayFace(const char *word);                         // Function for showing a single four-character word (or image) on the display.
void displayFace_P(const char *wordP);                      // Function for showing a single four-character word (or image) stored in PROGMEM on the display.
void scrollMenuText(const char *text);                      // Helper function that receives text from "showGame()" and "showTool()."
void updateDisplay();                                       // Function that's called when we want to update the display.
void displayErrorMessage(const char *message);              // Displays an error message, then attempts to reset DEALR.
void getProgmemString(const char *progmemStr, char *buffer, // Helper function for progmem.
                      size_t bufferSize);
static void getAnimFrame_P(const char (*frames)[5], uint8_t index, char out[5]); // Helper function for getting animation frames from PROGMEM.
void runAnimation(const DisplayAnimation &animation);

// UI Manipulation Function Prototypes
void advanceMenu();     // Function for progressing when Button 1 (green) has been pressed.
void goBack();          // Function for regressing when Button 4 (red button) has been pressed.
void decreaseSetting(); // Function for what happens when Button 2 (blue) is pressed.
void increaseSetting(); // Function for what happens when Button 3 (yellow) is pressed.
void checkTimeouts();   // Function for checking to see whether device has been idle a long time, and starting the idle animation.

// Motor Control Functions
void slideCard();                                   // Function for sliding a card into the flywheel for dealing.
void rotate(uint8_t rotationSpeed, bool direction); // Function for rotating. Takes speed and direction as inputs.
void rotateStop();                                  // Function for stopping rotation.
void flywheelOn(bool direction);                    // True = "forward"; False = "reverse".
void flywheelOff();                                 // Turns flywheel off.
void switchRotationDirection();                     // Reverses direction of yaw rotation.

// Tools and Their Helper Functions
void colorTuner();                 // Controls the "color tuning" operation that locks down RGB values for specific color tags.
void recordColors(int startIndex); // Helper function for colorTuner().
void uvSensorTuner();              // Controls the "UV tuning" operation that locks down the threshold visible light value for a card to be determined "marked".
void recordUVThreshold();          // Helper function for uvSensorTuner().
void resetEEPROMToDefaults();      // Function for resetting EEPROM values to defaults.

// Error-and-Timeout-handling Functions
void handleThrowingTimeout(unsigned long currentTime); // Handles timeouts while dealing cards.
void handlePostDealOutcome();                          // Handles errors that occur during post-deal.
void resetThrowCardState();                            // Resets variables related to dealing a card when an error occurs.
void handleFineAdjustTimeout();                        // Handles timeout for fine adjustment moves.
void resetFlags();                                     // Resets all state machine flags when called.
void resetColorsSeen();                                // Function used in the "reset" tool to reset colors seen.

// Bools for Evaluating Rigged Hands
void colorDetected(uint8_t colorIndex);                  // Uses the index value of the color seen to increment the number of cards in that player's hand.
bool allHandsExceptActiveFull(uint8_t activeColor);      // Returns "true" if all hands except currently active hand are full. Useful at the end of a deal during rigged games.
bool allNonRiggedColorsFull();                           // Returns "true" if all colors that are not red are full. Useful at the end of a deal during rigged games.
bool isRiggedHandFull();                                 // Returns "true" if red's hand is full. Useful at the end of a deal during rigged games.
bool isHandFull(uint8_t activeColor);                    // Returns "true" if hand of currently active color is full. Useful for not overdealing hands during rigged games.
bool allSeenColorsFull();                                // Returns "true" if all scanned colors are full. Allows us to end a main deal during rigged games.
bool isTooFarAhead(uint8_t activeColor);                 // Returns "true" if one player has too many more cards than another (default 8)
bool allOtherHandsTooFarAheadOrFull(uint8_t checkColor); // Returns "true" if all other hands are too far ahead or full

// Reading and Writing to EEPROM Functions
void initializeEEPROM();                                 // Function for checking whether EEPROM values were set by the user or are factory defaults.
void writeColorToEEPROM(int index, RGBColor color);      // Used for writing RGB values to EEPROM.
void loadColorsFromEEPROM();                             // Whatever colors have been saved to EEPROM get loaded at startup using this function.
void loadStoredUVValueFromEEPROM(uint16_t &uvThreshold); // Loads stored UV threshold values from EEPROM on boot.
RGBColor readColorFromEEPROM(int index);                 // Helper function used in loadColorsFromEEPROM.
RGBColor getBlackColorFromEEPROM();                      // Lets us check the value of our "baseline" luminance when no tag visible.

// Helper Functions for Printing to Serial Monitor
void printStoredColors();      // Prints RGB values for the colors that are stored (Default are Black, Red, Yellow, Green, and Blue).
void printDealtCardsInfo();    // Helper function for printing information on what colors have been seen to the serial monitor.
void printStoredUVThreshold(); // Helper function for printing the stored UV threshold for determining when a card has been marked.

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

  // Try NHY3274TH first
  if (nhySensor.begin())
  {
    activeSensor = &nhySensor;
    if (useSerial)
    {
      Serial.println("NHY");
    }
    nhySensor.setIntegrationTime(0x02);
    nhySensor.setGain(0x08);
  }
  // Try LTR381RGB if NHY not found
  else if (ltrSensor.begin())
  {
    activeSensor = &ltrSensor;
    if (useSerial)
    {
      Serial.println("LTR");
    }
    // Example: 17-bit (50 ms) integration with 6× gain, 50 ms meas. rate
    ltrSensor.setResolutionAndRate(/*resolutionBits=*/4, /*rateCode=*/1);
    ltrSensor.setGain(/*0..4*/ 4); // 6×
  }
  else
  {
    if (useSerial)
    {
      Serial.println("No sensor!");
    }
    while (true)
      ; // Halt here if neither is present
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
  pinMode(UV_READER, INPUT);           // Set the UV_reader pin as an input
  pinMode(RIG_SWITCH, INPUT);          // Set the rig_switch as an input
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

  checkIfRigged(); // When starting up, poll rig-switch to see if game is rigged on boot.

  initializeEEPROM();                                // This function checks to see whether EEPROM was set by the user, or is factory defaults, and loads those values.
  loadColorsFromEEPROM();                            // This function loads the above EEPROM values.
  uint16_t blackBaseline = calculateBlackBaseline(); // Read the color values for black from EEPROM and sum them to create a baseline for the color black.

  loadStoredUVValueFromEEPROM(storedUVThreshold); // If we have never run the UV Tuning tool, the storedUVThreshold will be the default value.

  if (useSerial)
  {
    Serial.println(F("Colors from EEPROM are "));
    printStoredColors();

    Serial.println(F("UV Values from EEPROM are "));
    printStoredUVThreshold();
    printDealtCardsInfo();

    Serial.print(F("Black baseline is: "));
    Serial.print(blackBaseline);
    Serial.println(F("**************"));
  }

  if (motorStartRoutine)
  {
    startRoutine(); // The start routine moves each of the motors a little to ensure they're working.
  }

  while (digitalRead(CARD_SENS) == LOW) // This while-loop activates if the card-sensing IR circuit is triggered on boot.
  {
    errorInProgress = true;
    while (!scrollingComplete && digitalRead(CARD_SENS) == LOW)
    {
      displayErrorMessage("EROR - TUNE IR SENSOR"); // If this error activates, there is either a card in the DEALR's mouth, or we need to tune the IR sensor screw.
    }
    scrollingComplete = false;
    messageRepetitions = 0;
  }
  errorInProgress = false;

  displayFace(" HI ");
  delay(1000);

  currentDealState = IDLE;
  currentDisplayState = INTRO_ANIM;
}

#pragma endregion SETUP
#pragma region LOOP

// MAIN LOOP
void loop()
{
  checkState();    // This function checks what state the DEALR is in (idle, dealing, awaiting player input, etc.) and lets the dealr behave accordingly.
  checkButtons();  // This function checks to see if buttons are being pressed
  checkTimeouts(); // This function tracks a few overall time-out circumstances (like, when the DEALR should go to sleep because it's bored!)
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

void checkState() // Always running to make sure DEALR does the right things in the right states.
{
  unsigned long currentTime = millis(); // Update time in ms every loop.

  if (currentDealState == ERROR_RECOVERY)
  {
    checkButtons();  // allow Retry / Exit input
    updateDisplay(); // keep scroll moving
    return;          // prevent any other state handling
  }

  if (currentDealState != previousDealState) // If we change from one state to another, this block lets us do anything that should only happen once during that transition
  {
    newDealState = true;
    overallTimeoutTag = currentTime; // Every time the dealState changes, update overall timeout tag.
    previousDealState = currentDealState;
    // if (useSerial)
    // {
    //   Serial.print("STATE:");
    //   Serial.println(currentDealState); // shows numeric state ID
    // }
  }

  if (gameOver) // At any point, we can set "gameOver" to "true" and the handleGameOver function will help us exit cleanly.
  {
    handleGameOver();
  }

  if (errorInProgress && currentDealState != ERROR_RECOVERY)
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

  case TAGLESS_DEAL:
    handleTaglessDealState(); // Handles deals that only use one tag (red by default) to home.
    break;

  case ERROR_RECOVERY:                  // Wait for player to choose retry or exit
    handleThrowingTimeout(currentTime); // keeps scroll animation alive
    checkButtons();                     // let buttons trigger their onRelease()
    updateDisplay();
    break;
  }
}

void startRoutine() // This routine both tests the motors and also retracts any card that might be in the craw (the mouth of the dealer)
{
  analogWrite(MOTOR_1_PWM, 255);      // Set flywheel PWM to high speed.
  analogWrite(MOTOR_2_PWM, lowSpeed); // Set yaw motor to low speed (give a small wiggle to show the motor works).
  digitalWrite(MOTOR_1_PIN_1, HIGH);  // Retract with flywheel.
  feedCard.write(30);                 // Retract with feed wheel.
  delay(500);                         // Wait a half second.
  feedCard.write(90);                 // Stop feed motor.
  digitalWrite(MOTOR_1_PIN_1, LOW);   // Stop flywheel.
  delay(100);                         // Wait a tenth of a second.
  digitalWrite(MOTOR_2_PIN_1, HIGH);  // Rotate a little clockwise (CW).
  delay(300);                         // Wait three tenths of a second.
  digitalWrite(MOTOR_2_PIN_1, LOW);   // Rotate a little counter-clockwise (CCW).
  digitalWrite(MOTOR_2_PIN_2, HIGH);
  delay(300);
  digitalWrite(MOTOR_2_PIN_2, LOW); // Turn yaw motor off.
  delay(100);
}

void handleDealingState()
{
  if (currentDealState == ERROR_RECOVERY)
  {
    return;
  }

  if (postDeal && !toolsMenuActive)
  {
    handlePostDealGameplay();
    return;
  }

  if (!dealInitialized)
  {
    initializeToRed();
    return;
  }

  if (dealInitialized && !cardDealt)
  {
    if (toolsMenuActive)
    {
      handleToolsDealing();
    }
    else if (riggedGame)
    {
      handleRiggedGameDealing();
    }
    else
    {
      dealSingleCard();
    }
  }

  if (dealInitialized && cardDealt)
  {
    cardDealt = false;

    if (currentDealState == ERROR_RECOVERY)
    {
      return;
    }

    if (toolsMenuActive)
    {
      handleToolsAdvancingDecisions();
      return;
    }

    if (riggedGame)
    {
      handleRiggedGameAdvancingDecisions();
    }
    else
    {
      if (!toolDealingActive)
      {
        handlePostDealOutcome();
      }
    }
  }
}

void dealSingleCard()
{
  currentDisplayState = LOOK_STRAIGHT;
  updateDisplay();

  // Immediately exit if recovering from an error
  if (currentDealState == ERROR_RECOVERY || errorInProgress)
  {
    return;
  }

  if (toolsMenuActive)
  {
    consecutiveDeals = 0; // When dealing within the tools menu, we don't have to worry about being suspicious for dealing too many cards in a row, so we can reset the consecutiveDeals count to 0 to allow unlimited dealing in a row.
  }

  if (consecutiveDeals < 3) // When "chaotically dealing" in rigged games, we can deal several cards in a row, but want to avoid dealing more than three in a row (suspicious).
  {
    while (!cardDealt && !errorInProgress)
    {
      cardDispensingActions();
      if (currentDealState == ERROR_RECOVERY)
      {
        return;
      }
    }
    flywheelOff();
    consecutiveDeals++;

    if (currentDealState == ERROR_RECOVERY)
    {
      return;
    }

    if (riggedGame && !taglessGame && !toolsMenuActive)
    {
      colorStatus[activeColor - 1]++;
      totalCardsToDeal--;
      // printDealtCardsInfo();
      //  Serial.print(F("totalCardsToDeal = "));
      //  Serial.println(totalCardsToDeal);
      if (totalCardsToDeal <= 0) // If we have no more cards to deal in the main deal, move on to post-deal (in games that have post-deal sections).
      {
        postDeal = true;
      }
    }
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
  if (taglessGame && postDeal) // If we're advancing, but we're in the tagless game mode and the main deal has completed successfully:
  {
    handleTaglessPlayerPostDealAdvance();
    return;
  }

  if (newDealState) // In this block we can do anything we only want to do each time we enter the "advancing" state from a different state
  {
    consecutiveDeals = 0;
    adjustInProgress = false;
    newDealState = false;
    if (activeColor != 0)
    {
      previousActiveColor = activeColor; // As we start moving away from a color, log it as the "previous color" so we can track the difference as we advance.
    }

    if (!postDealRemainderHandled && !postDeal && !separatingCards && !shufflingCards) // If it's not the post-deal (etc), check if there are rounds left to deal.
    {
      if (remainingRoundsToDeal == 0) // If there are no more rounds to deal, set "postDeal" to true and enter into that stae.
      {
        postDeal = true;
        rotatingBackwards = false; // In post-deal we always rotate forwards, so in any mode that has us rotating backwards, switch to forward rotation.
      }
    }

    // Ping pong shortest path for War
    if (currentGame == 3)
    {
      if (previousActiveColor == 1)
      { // Leaving Red, going to Tag 2
        rotatingBackwards = !redToTag2IsCW;
      }
      else
      { // Leaving Tag 2, going back to Red
        rotatingBackwards = redToTag2IsCW;
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
    if (errorInProgress)
      return;

    if (!baselineExceeded)
    {
      if (useSerial)
        Serial.println(F("[FALSE ALARM] Spike vanished. Aborting adjust."));
      adjustInProgress = false;
      fineAdjustCheckStarted = false;
      correctingCW = false;
      correctingCCW = false;
      return; // Break out of the adjustment completely!
    }
  }

  rotateStop(); // At this point we have a stable active color. We stop and then make decisions about whether or not we deal a card, depending on game states.

  if (useSerial)
  {
    Serial.print(F("[ADJUST DONE] Backed into tag index: "));
    Serial.println(activeColor);
  }

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
    while (activeColor != 0 || baselineExceeded) // Active color wasn't red, but we're looking for red to initialize! So we keep rotating, first until we hit black again, and then we carry on.
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
    resetColorsSeen();           // This is probably redundant since we should have reset colorsSeen before the deal.
    totalCardsToDeal = 0;        // Reset cards-to-deal to zero.
    notFirstRoundOfDeal = false; // We have reset and are initialized and about to commence the first round of a deal.
    if (taglessGame)             // Tagless games cannot currently be rigged (the state is turned off when tagless is selected), and are handled a little differently.
    {
      moveOffActiveColor(CW);          // Move off the red tag in a clockwise fashion.
      currentDealState = TAGLESS_DEAL; // Set the deal state to the Tagless Deal state.
      return;
    }
    if (riggedGame || currentGame == 3) // If we're initializing to red, and the game is rigged...
    {
      while (!tagsChecked) // If we're dealing a rigged game, then we need to do a full 360 to check what tags are present, and log the order of those tags.
      {
        if (errorInProgress)
        {
          return;
        }
        checkForTags360();
      }
      markedCard = checkIfMarked();
      if (markedCard && riggedColor == 1) // In most card games in general, deal proceeds to the player left of dealer first. But if DEALR detects a marked card is first up, we want to give it to red immediately.
      {
        currentDealState = DEALING;
        return;
      }
    }
    currentDealState = ADVANCING; // In most cases, after initializing to red, this line switches us into the ADVANCING state so we move off the red tag to the player left of dealer.
  }
}

void handleAdvancingStateInAdjust() // This function handles the decision-making on how we advance after a color has been confirmed following the fineAdjustCheck process.
{
  adjustInProgress = false;       // Resets a tag used in the fine adjustment operation
  fineAdjustCheckStarted = false; // Resets a tag used in the fine adjustment operation
  correctingCW = false;           // Resets a tag used in the fine adjustment operation
  correctingCCW = false;          // Resets a tag used in the fine adjustment operation

  if (activeColor == previousActiveColor && activeColor != 0)
  {
    if (useSerial)
      Serial.println(F("[FALSE ALARM] Backed into previous tag. Resuming advance."));
    if (rotatingBackwards)
    {
      moveOffActiveColor(CCW);
    }
    else
    {
      moveOffActiveColor(CW);
    }
    currentDealState = ADVANCING;
    return;
  }

  if (advanceOnePlayer)
  {
    handleAdvancingOnePlayer(); // If we were only supposed to advance one player during a post-deal, we run this function.
    return;
  }

  if (activeColor == 1 && previousActiveColor == 1 && !taglessGame && !postDeal) // If we've done a full circle and hit red a second time in a row, we know we're missing tags! Throw an error.
  {
    errorInProgress = true;
    while (!scrollingComplete)
    {
      displayErrorMessage("EROR - ADD MORE TAGS");
    }
    scrollingComplete = false;
    messageRepetitions = 0;
    fullExit = true;
    currentDealState = RESET_DEALR;
    return;
  }

  if (separatingCards) // This is the only tool that requires decision making after every card is dealt. We need to evaluate on the fly whether to advance or deal again.
  {
    markedCard = checkIfMarked(); // Check if the next card is marked, and set that bool value equal to markedCard
    if (activeColor > 1)          // If tag is not red...
    {
      switchRotationDirection(); // Reverse direction back towards the red tag.
      if (!markedCard)           // If card is not marked...
      {
        if (!cardDealt)
        {
          currentDealState = DEALING; // If the card isn't marked, deal it immediately to the non-red tag.
          return;
        }
      }
      else // If card is marked...
      {
        newDealState = true;
        currentDealState = ADVANCING; // If the card is marked, advance back to the red tag before dealing.
        return;
      }
    }
    else if (activeColor == 1) // If tag is red...
    {
      switchRotationDirection();
      if (markedCard) // If card is marked...
      {
        if (!cardDealt)
        {
          currentDealState = DEALING;
          return;
        }
      }
      else
      {
        newDealState = true;
        currentDealState = ADVANCING;
        return;
      }
    }
  }

  if (riggedGame) // If game is rigged and we're on a known color, jump into "dealing" to decide what to do.
  {
    currentDealState = DEALING; // This sends us back into handleDealingState() where we decide whether or not to deal to this new color.
  }
  else // If the game is not rigged...
  {
    handleStandardGameDecisionsAfterFineAdjust();
  }
}

void handleTaglessPlayerPostDealAdvance() // This function helps the "tagless deal" game mode time its advancing moves after the main deal.
{
  unsigned long currentTime = millis();
  colorScan();

  rotate(mediumSpeed, CW);
  if ((cardsInHandToDeal > 0 && (currentTime - lastDealtTime > taglessRotationInterval)) || ((cardsInHandToDeal == 0) && (activeColor == 1)))
  {
    rotateStop();
    if (cardsInHandToDeal <= 0)
    {
      cardsInHandToDeal = numberOfPlayers;
    }
    currentDealState = AWAITING_PLAYER_DECISION;
  }
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
    if (!notFirstRoundOfDeal)
    {
      colorStatus[activeColor - 1] = 0;
      notFirstRoundOfDeal = true;
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
        displayErrorMessage("EROR - TOO MANY TAGS");
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
  if (currentDealState == ERROR_RECOVERY)
  {
    return;
  }

  if ((currentGame == 3 || currentGame == 4) && !toolsMenuActive) // War and Hearts have no post-deal, so we toggle "gameOver" immediately after the main deal to exit cleanly.
  {
    gameOver = true;
    return;
  }

  if (riggedGame)
  {
    if (currentGame == 1 || currentGame == 3 || currentGame == 4 || currentGame == 5) // In these games, after a rigged deal, you want to end up looking at red. In some, you then deal a flip card.
    {
      postDealStartOnRed = true;
      while (activeColor != 1) // Return to red
      {
        colorScan();
        rotate(mediumSpeed, CW);
      }
    }
    else if (currentGame == 0 || currentGame == 2 || currentGame == 6) // If we're playing Go Fish, Crazy Eights, or Custom Game, and game is rigged, we want to end up looking at player 1 (not red) after main deal.
    {
      while (activeColor != colorLeftOfDealer) // After red, advance to the next color.
      {
        colorScan();
        rotate(mediumSpeed, CW);
      }
    }
    rotateStop(); // After we arrive at whichever player DEALR should be facing for the post-deal, stop. Post-game deals are not currently rigged.
    rotatingBackwards = false;
    resetColorsSeen();
  }
  else
  {
    postDealStartOnRed = false;
    while (activeColor < 2 && previousActiveColor != 1) // Return to non-red color to the left of red:
    {
      colorScan();
      rotate(mediumSpeed, CW);
    }
    rotateStop();
  }

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
  if (currentDealState != ERROR_RECOVERY) // advance only if no error
    currentDealState = ADVANCING;
}

void handleRiggedGameDealing() // At this point we have advanced, seen a color, stopped, but have not yet dealt. The job here is to decide, for each rigged hand, whether to deal a card or skip the player and advance.
{
  if (currentDealState == ERROR_RECOVERY)
    return;

  markedCard = checkIfMarked();               // This function checks to see if the current next card is marked.
  uint8_t activeColorIndex = activeColor - 1; // Adjust the active color index value to match the colorStatus array indexing (which excludes black).

  if (colorStatus[activeColorIndex] >= initialRoundsToDeal || isTooFarAhead(activeColor)) // If the number of cards in the current player's hand is equal to the number of total cards each hand should have, skip this player.
  {
    currentDealState = ADVANCING;
    return;
  }

  // If everyone else's hand is full, we are just finishing the deck for this player.
  if (allHandsExceptActiveFull(activeColor))
  {
    consecutiveDeals = 0;       // Bypass the 8-card anti-cheat limit
    dealSingleCard();           // Rapid-fire the card
    currentDealState = DEALING; // Stay exactly where we are for the next card!
    return;
  }

  if (riggedGame && (isRiggedHandFull() || (activeColor == riggedColor && isTooFarAhead(riggedColor))) && markedCard)
  {
    markedCard = false;
  }

  if (riggedGame && !markedCard && activeColor == riggedColor && allOtherHandsTooFarAheadOrFull(riggedColor))
  {
    markedCard = true; // Pretend it's marked so Red takes it!
  }

  if (markedCard)
  {
    if (activeColor == riggedColor) // If we're seeing a marked card, and the active color is the rigged color...
    {
      if (!isRiggedHandFull()) // And the rigged hand is not full...
      {
        dealAndAdvance(); // Deal the marked card to the rigged color, then advance.
      }
      else
      {
        currentDealState = ADVANCING; // If the rigged hand is full, skip this player.
      }
      return;
    }

    if (previousActiveColor == riggedColor) //...but the prior color seen was the rigged color
    {
      if (!isRiggedHandFull()) //... and the rigged hand is not full...
      {
        switchRotationDirection(); // Switch direction back to the rigged color.
        currentDealState = ADVANCING;
        return;
      }
      else // If the rigged hand is full...
      {
        dealSingleCard(); // Deal the marked card to the present non-rigged color.
        return;
      }
    }
    currentDealState = ADVANCING;
    return;
  }
  else // If we're seeing an unmarked card...
  {
    if (activeColor == riggedColor) // And if the active color is the rigged color...
    {
      currentDealState = ADVANCING; // Skip the rigged color when seeing unmarked cards.
    }
    else // If the active color is NOT the rigged color...
    {
      dealAndAdvance();
    }
  }
}

void handleRiggedGameAdvancingDecisions() // After we deal a card during a rigged game, we need to decide whether to deal or advance.
{
  if (currentDealState == ERROR_RECOVERY)
    return;

  if (totalCardsToDeal <= 0 || allSeenColorsFull() && !postDeal) // If total cards to deal are exhausted, or all known hands are full...
  {
    postDeal = true;
    currentDealState = DEALING; // We enter into post-deal through the DEALING state.
    return;
  }

  cardDealt = false; // Reset cardDealt flag to prep for the next card that should be dealt.
  markedCard = checkIfMarked();

  if (riggedGame && (isRiggedHandFull() || (activeColor == riggedColor && isTooFarAhead(riggedColor))) && markedCard)
  {
    markedCard = false;
  }

  if (markedCard)
  {
    if (activeColor == riggedColor && !isRiggedHandFull()) // If we're looking at a marked card and we're already pointed towards the rigged game tag, deal it before advancing.
    {
      currentDealState = DEALING;
      return;
    }

    if (activeColor != 0 && activeColor != riggedColor)
    {
      if (previousActiveColor == riggedColor && !isRiggedHandFull()) // If previous color was the rigged color and that color is not full:
      {
        switchRotationDirection(); // Every time we find a marked card and the rigged color was the last color seen, double back and deal to the rigged color.
      }
    }
    return;
  }
  if (activeColor > 0 && activeColor != riggedColor) // And if the player is not the rigged color...
  {
    if (previousActiveColor != riggedColor) // When we're dealing unmarked cards to non-rigged players, sometimes switch directions to be more "chaotic"
    {
      rotatingBackwards = random(3) == 0; // Randomly set rotatingBackwards to true or false, but ONLY after dealing a card.
    }

    bool triggerDoubleDeal = false;

    // --- NEW: Split the probability based on the game ---
    if (currentGame == 3) // If the game is War
    {
      // Aggressively deal chunks of cards to speed up the massive 52-card deal
      if (random(100) < 85)
      {
        triggerDoubleDeal = true;
      }
    }
    else // For all other games (Go Fish, 21, etc.)
    {
      // The original logic: Chance of dealing multiple cards in a row increases as the deal proceeds.
      float initialCardsToDeal = float(initialRoundsToDeal) * float(numTags);
      float chance = (initialCardsToDeal - float(totalCardsToDeal)) / initialCardsToDeal;
      if (random(100) < chance * 100)
      {
        triggerDoubleDeal = true;
      }
    }

    // Execute the multi-deal if the chance triggered, the hand isn't full, AND they aren't 8 cards ahead!
    if (triggerDoubleDeal && !isHandFull(activeColor) && !isTooFarAhead(activeColor))
    {
      currentDealState = DEALING;
      return;
    }
  }
}

void handleToolsDealing() // This function is specifically *only* dealing, not advancing, which is why it's comparatively simple.
{
  toolDealingActive = true;

  postDeal = false;
  gameOver = false;

  if (currentToolsMenu == 0) // If we're using the "deal single card tool" and are in the "dealing" state, simply deal a card.
  {
    if (riggedGame == true)
    {
      checkIfMarked(); // This just lets us illuminate the little green light on the Nano when we see a marked card during the execution of this tool.
    }
    dealSingleCard();
  }
  else if (currentToolsMenu == 1 || currentToolsMenu == 2) // These are the "separate marked cards" and "shuffle cards" tools. The other tools are more programmatic and have their own sections.
  {
    dealSingleCard();
  }

  toolDealingActive = false;
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
    toolDealingActive = false;
    return;
  }
  else if (currentToolsMenu == 1)
  {
    switchRotationDirection();
    currentDealState = ADVANCING;
    adjustStart = millis();
  }
  else if (currentToolsMenu == 2)
  {
    markedCard = checkIfMarked();
    if ((activeColor == 1 && markedCard) || (activeColor != 1 && !markedCard)) // For this tool, deal all marked cards to red, and all unmarked cards to any color that isn't red.
    {
      currentDealState = DEALING;
    }
    else
    {
      currentDealState = ADVANCING;
    }
  }
}

void handleResetDealrState() // Handles what happens when we enter the "reset" state.
{
  resetFlags();
  if (fullExit) // In full exit, we exit all the way to the intro screen.
  {
    currentGame = 0;
    currentToolsMenu = 0;
    displayFace("EXIT");
    delay(1000);
    buttonInitialization = false;
    toolsMenuActive = false;
    if (scrollInstructions)
    {
      currentDisplayState = SCROLL_PLACE_TAGS_TEXT;
    }
    else
    {
      initialAnimationComplete = false;
      currentDisplayState = INTRO_ANIM;
    }
    fullExit = false;
  }
  else if (gamesExit) // In a "games exit," we just exit to the select game menu.
  {
    currentToolsMenu = 0;
    displayFace("GAME");
    delay(1000);
    buttonInitialization = true;
    toolsMenuActive = false;
    currentDisplayState = SELECT_GAME;
    gamesExit = false;
  }
  else if (toolsExit) // In a "tools exit," we just exit to the select tool menu.
  {
    currentGame = 0;
    displayFace("TOOL");
    delay(1000);
    buttonInitialization = true;
    toolsMenuActive = true;
    currentDisplayState = SELECT_TOOL;
    toolsExit = false;
  }
  currentDealState = IDLE;
  updateDisplay();
}

void handleTaglessDealState() // Handles what happens when we enter a "tagless deal" state.
{
  if (newDealState)
  {
    newDealState = false;
    taglessGame = true;
  }

  if (!dealInitialized) // If we command a "tagless deal," but we're not initialized, initialize to red first.
  {
    initializeToRed();
  }

  if (dealInitialized && !rotationTimeChecked) // If we're doing a tagless deal and have initialized but haven't checked how long a 360 takes to perform, do that first.
  {
    checkRotationTime();
  }

  if (dealInitialized && rotationTimeChecked)
  {
    dealTagless();
  }
}

void handleIdleState() // Handles what happens in the "IDLE" dealing state.
{
  if (newDealState == true)
  {
    // if (useSerial)
    // {
    //   Serial.println(F("Entered IDLE state from other state."));
    // }
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
    // if (currentDisplayState == SCROLL_PLACE_TAGS_TEXT && buttonInitialization == true)
    // {
    //   buttonInitialization = false; // De-initialize when idle and in pre-game state so the animation can come up.
    //   startPreGameAnimation();
    // }
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

  if (currentGame == 3 || currentGame == 4) // If we're playing "War" or "Hearts" and full deck is dealt, display "PLAY" and then exit to the select game menu. These games traditionally end after the main deal, so we don't have a post-deal to handle.
  {
    displayFace("PLAY");
    delay(3000); // Hold for 3 seconds
  }

  gameOver = false;
  toolsMenuActive = false; // Switching to select game menu. Deactivating tool menu.
  gamesExit = true;
  currentDealState = RESET_DEALR;
  currentDisplayState = SELECT_GAME;
  updateDisplay();
}

void moveOffActiveColor(bool rotateClockwise) // Function that moves DEALR off the active tag, ideally into an empty portion of the circle.
{
  while (activeColor != 0 || baselineExceeded)
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
  flippingCard = true;
  handlingFlipCard = true;

  moveOffActiveColor(CCW);
  delay(20);
  currentDisplayState = FLIP;
  updateDisplay();
  delay(350);

  dealSingleCard();
  cardDealt = false;
  postDealRemainderHandled = true;

  returnToActiveColor(CW);
  handlingFlipCard = false;

  if (currentDealState != ERROR_RECOVERY)
  {
    flippingCard = false;
    currentDisplayState = DEAL_CARDS;
    updateDisplay();
  }
}

void dealTagless() // Handles "tagless deals", where we use only the red tag for homing, but can deal to any number of players.
{
  unsigned long currentTime = millis();

  if (stopped)
  {
    rotate(mediumSpeed, CW);
  }

  colorScan();

  if (((cardsInHandToDeal > 1) && (currentTime - lastDealtTime > taglessRotationInterval)) || (cardsInHandToDeal <= 1 && activeColor == 1)) // If there are still cards to deal and we've exceeded the taglessRotationInterval...
  {
    rotateStop();
    consecutiveDeals = 0;
    dealSingleCard();
    cardDealt = false;
    cardsInHandToDeal--;
    if (cardsInHandToDeal <= 0)
    {
      cardsInHandToDeal = numberOfPlayers;
      remainingRoundsToDeal--;
      if (remainingRoundsToDeal == 0)
      {
        // Serial.println(F("Post-deal about to start. Spin a bit more."));
        lastScrollTime = millis();
        rotateStop();
        rotate(mediumSpeed, CW);
        while (millis() - lastScrollTime <= taglessRotationInterval)
        {
          colorScan();
        }
        postDeal = true;
        rotateStop();
        cardsInHandToDeal--;
        currentDealState = AWAITING_PLAYER_DECISION;
      }
      else if (postCardsToDeal <= 0)
      {
        gameOver = true;
        return;
      }
    }

    lastDealtTime = millis();
  }
}

void checkRotationTime() // Used during "tagless" deal. We see how long it takes to rotate around in a circle and then divide that up into intervals.
{
  unsigned long currentTime = millis();
  static bool rotationStarted = false;
  static unsigned long rotationStartTime = 0;

  if (activeColor == 0 && !rotationStarted)
  {
    rotationStarted = true;
    rotationStartTime = currentTime;
    rotateStop();
    rotate(mediumSpeed, CW);
  }

  colorScan();

  if (activeColor == 1 && !rotationTimeChecked)
  {
    static unsigned long rotationEndTime = 0;
    rotateStop();
    rotationEndTime = currentTime;
    rotationTime = rotationEndTime - rotationStartTime;
    taglessRotationInterval = (rotationTime / numberOfPlayers) * .88; // This value lowers taglessRotationInterval to account for some rotational inertia.
    lastDealtTime = millis();
    rotationTimeChecked = true;
  }
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
      if (!taglessGame)
      {
        postCardsToDeal--;
      }
      else
      {
        cardsInHandToDeal--;
      }
    }
  }
  else
  {
    if (activeColor > 1 && previousActiveColor == 1) // If our post-deal dealt its first card to the player left-of-dealer, decrement whenever we pass red.
    {
      if (!taglessGame)
      {
        postCardsToDeal--;
      }
      else
      {
        cardsInHandToDeal--;
      }
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
      if (currentDisplayState != SCROLL_PLACE_TAGS_TEXT)
      {
        buttonInitialization = true; // A button has been pressed, so we know not to start the screensaver blinking animation, except when we've hit "back" all the way to the beginning.
      }
    }
    longPressFlag = false; // Reset the long-press flag after release
  }

  lastButtonState = currentButtonState; // Update the last button state
}

void onButton1Release() // This function handles what happens when we release Button 1 (green). It matters that the action happens on "release" so we can use long-click in some cases.
{
  if (currentDealState == ERROR_RECOVERY)
  {
    // --- fully reset dealing flags ---
    errorInProgress = false;
    gameOver = false;
    toolsMenuActive = false;

    cardDealt = false;
    throwingCard = false;
    slideStep = 0;
    previousSlideStep = -1;

    timeoutLatched = false;
    retracting = false;
    retractDone = false;
    retractStartTime = 0;

    // reset servo/timing
    resetThrowCardState();

    // resume normal operation
    dealInitialized = true;
    currentDisplayState = DEAL_CARDS;
    currentDealState = DEALING;
    scrollingStarted = false;
    messageLine = 0;

    if (toolsMenuActive)
      toolDealingActive = true;
    currentDealState = DEALING;

    if (flippingCard)
    {
      handleFlipCard(); // If we were flipping a card when the error occurred, retry that operation.
    }
    else
    {
      currentDisplayState = DEAL_CARDS;
      currentDealState = DEALING;
      updateDisplay();
    }
    return;
  }

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

  if (currentDisplayState == SCREENSAVER || currentDisplayState == SCROLL_PLACE_TAGS_TEXT || currentDisplayState == SCROLL_PICK_GAME_TEXT)
  {
    advanceMenu();
  }
  else if (currentDisplayState == SELECT_CARDS || currentDisplayState == SELECT_PLAYERS || currentDisplayState == SELECT_TOOL || currentDisplayState == SELECT_GAME)
  {
    increaseSetting();
  }
}

void onButton3Release() // This function handles what happens when we release Button 3 (yellow).
{
  if (currentDisplayState == SCREENSAVER || currentDisplayState == SCROLL_PLACE_TAGS_TEXT || currentDisplayState == SCROLL_PICK_GAME_TEXT)
  {
    advanceMenu();
  }
  else if (currentDisplayState == SELECT_PLAYERS || currentDisplayState == SELECT_CARDS || currentDisplayState == SELECT_GAME || currentDealState == IDLE)
  {
    decreaseSetting();
  }
}

void onButton4Release()
{
  if (currentDealState == ERROR_RECOVERY)
  {
    // --- stop any ongoing scroll ---
    scrollingStarted = false;
    scrollingMenu = false;
    messageRepetitions = 0;
    scrollIndex = -1;
    currentFrameIndex = 0;
    blinkingAnimationActive = false;

    // --- stop motion and lights ---
    feedCard.write(90);
    flywheelOff();
    rotateStop();

    // --- clear flags to fully exit ---
    errorInProgress = false;
    timeoutLatched = false;
    retracting = false;
    retractDone = false;
    retractStartTime = 0;
    flippingCard = false;

    // --- return to safe idle/menu state ---
    currentDisplayState = SCROLL_PICK_GAME_TEXT; // or SELECT_GAME, whichever starts a new session
    currentDealState = RESET_DEALR;
    postDeal = false;
    toolsMenuActive = false;
    gameOver = false;

    updateDisplay();
    return;
  }

  // existing non-error EXIT logic
  if (currentDealState != IDLE)
  {
    if (toolsMenuActive || insideDealrTools || currentDisplayState == SELECT_TOOL)
    {
      toolsExit = true;
      gamesExit = false; // force-clear the other one
    }
    else
    {
      gamesExit = true;
      toolsExit = false;
    }

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
  // Serial.println("Button 1 long-press"); //Currently there is no application of Button 1 long-press.
}

void onButton2LongPress()
{
  overallTimeoutTag = millis();
  scrollDelayTime = 0;
  scrollingStarted = false;
  scrollingMenu = false;
  messageRepetitions = 0;
  scrollIndex = -1;

  toolsMenuActive = true;
  currentToolsMenu = 3;
  currentDisplayState = SELECT_TOOL;
  updateDisplay();
  buttonInitialization = true;
  blinkingAnimationActive = false;
  initialAnimationInProgress = false;
  scrollingComplete = true;
}

void onButton3LongPress()
{
  overallTimeoutTag = millis();
  scrollDelayTime = 0;
  scrollingStarted = false;
  scrollingMenu = false;
  messageRepetitions = 0;
  scrollIndex = -1;
  buttonInitialization = true;

  // Serial.println("Shortcut to *5. UV SENSOR TUNER");
  toolsMenuActive = true;
  currentToolsMenu = 4;
  currentDisplayState = SELECT_TOOL;
  updateDisplay();
  buttonInitialization = true;
  blinkingAnimationActive = false;
  initialAnimationInProgress = false;
  scrollingComplete = true;
}

void onButton4LongPress()
{
  overallTimeoutTag = millis();
  scrollDelayTime = 0;
  scrollingStarted = false;
  scrollingMenu = false;
  messageRepetitions = 0;
  scrollIndex = -1;

  // Serial.println("Shortcut to *1. DEAL SINGLE CARD");
  toolsMenuActive = true;
  currentToolsMenu = 0;
  currentDisplayState = SELECT_TOOL;
  updateDisplay();
  buttonInitialization = true;
  blinkingAnimationActive = false;
  initialAnimationInProgress = false;
  scrollingComplete = true;
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

void checkIfRigged() // Reads the switch on the back of the dealer's head that determines whether games are "rigged" or "unrigged."
{
  uint8_t rigSwitchValue = analogRead(RIG_SWITCH); // A6 cannot use digitalRead(), so we do an analogRead() and assign that value as "on" or "off" with this variable.

  if (rigSwitchValue >= 125)
  {
    riggedGame = true;
  }
  else
  {
    riggedGame = false;
  }
}

bool checkIfMarked() // Reads a card and determines whether or not that card has been marked.
{
  uint16_t average = 0; // Average of the readings
  uint16_t total = 0;   // Running total of the readings

  for (int i = 0; i < numberOfReadings; i++)
  {
    uvReaderValue = analogRead(UV_READER); // Read from the UV sensor
    total += uvReaderValue;                // Add the reading to the total
    delay(10);
  }
  average = total / numberOfReadings; // Calculate the average:

  if (average >= storedUVThreshold) // Determine if the card is marked
  {
    digitalWrite(LED_BUILTIN, HIGH); // turn the LED on
    markedLEDStart = millis();
    markedCard = true;
  }
  else
  {
    digitalWrite(LED_BUILTIN, LOW); // turn the LED off by making the voltage LOW
    markedCard = false;
  }

  return markedCard; // Returns "true" if card is marked, "false" otherwise
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
  getColorData(&r, &g, &b, &c);

  // Serial.print("RRed: ");
  // Serial.print(r);
  // Serial.print(" RGreen: ");
  // Serial.print(g);
  // Serial.print(" RBlue: ");
  // Serial.print(b);
  // Serial.print(" RWhite: ");
  // Serial.println(c);

  normalizeRaw(&r, &g, &b);
  totalColorValue = r + g + b;

  // Serial.print("NRed: ");
  // Serial.print(r);
  // Serial.print(" NGreen: ");
  // Serial.print(g);
  // Serial.print(" NBlue: ");
  // Serial.print(b);
  // Serial.print(" NWhite: ");
  // Serial.println(c);

  baselineExceeded = checkForColorSpike(c, blackBaseline);

  float normalizedR = r / totalColorValue * 255.0;
  float normalizedG = g / totalColorValue * 255.0;
  float normalizedB = b / totalColorValue * 255.0;

  float distances[numColors];
  for (uint8_t i = 0; i < numColors; i++)
  {
    float dR = normalizedR - colors[i].r;
    float dG = normalizedG - colors[i].g;
    float dB = normalizedB - colors[i].b;

    // Simple multiplication replaces pow(), and we drop sqrt() entirely
    distances[i] = (dR * dR) + (dG * dG) + (dB * dB);
  }

  uint8_t closestColor = 0;

  if (baselineExceeded)
  {
    float minDistance = distances[0];
    for (uint8_t i = 1; i < numColors; i++)
    {
      if (distances[i] < minDistance)
      {
        minDistance = distances[i];
        closestColor = i;
      }
    }
    // --- LUMINANCE REJECTION FILTER ---
    // If the color matches, but is less than 50% as bright as the saved EEPROM profile,
    // it is track glare pretending to be a tag. Reject it!
    if (closestColor != 0 && c < (colors[closestColor].avgC * 0.5))
    {
      closestColor = 0;
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

    // --- DEBUG SNIPPET ---
    static uint8_t lastPrintedColor = 0;

    if (activeColor == 0)
    {
      lastPrintedColor = 0; // Reset when we hit the black track
    }
    else if (useSerial && activeColor != lastPrintedColor)
    {
      const char *colorNames[] = {"Black", "Red", "Yellow", "Blue", "Green"};
      Serial.print(F("\n*** TAG CONFIRMED: "));
      Serial.print(colorNames[activeColor]);
      Serial.print(F(" *** | R: "));
      Serial.print(r);
      Serial.print(F(" G: "));
      Serial.print(g);
      Serial.print(F(" B: "));
      Serial.println(b);

      lastPrintedColor = activeColor; // Lock it so it only prints once per tag!
    }
    // ------------------------------

    delay(10);
  }
}

void getColorData(uint16_t *r, uint16_t *g, uint16_t *b, uint16_t *c)
{
  if (activeSensor)
  {
    activeSensor->getRawData(r, g, b, c); // Use interface method
  }
  else
  {
    *r = *g = *b = *c = 0; // fallback if no sensor was detected
  }
}

void normalizeRaw(uint16_t *r, uint16_t *g, uint16_t *b)
{
  float rGain = 1.0;
  float gGain = 1.0;
  float bGain = 1.0;

  // Apply gains based on which sensor successfully initialized
  if (activeSensor == &nhySensor)
  {
    // These values were obtained by dividing the averaging of r+g+b read over black
    rGain = 1.22;
    gGain = 0.60;
    bGain = 1.92;
  }
  else if (activeSensor == &ltrSensor)
  {
    float ltr_rGain = 1.00;
    float ltr_gGain = 0.57; // Drastically reduces the oversensitive green
    float ltr_bGain = 1.05; // Gives blue a tiny boost to help separation
  }

  uint32_t rVal = (uint32_t)(*r * rGain);
  uint32_t gVal = (uint32_t)(*g * gGain);
  uint32_t bVal = (uint32_t)(*b * bGain);

  *r = min(rVal, 65535UL);
  *g = min(gVal, 65535UL);
  *b = min(bVal, 65535UL);
}

bool checkForColorSpike(uint16_t c, uint16_t blackBaseline)
{
  // Set the default multiplier for the NHY sensor
  float spikeMultiplier = 1.6;

  // Adjust it if the LTR sensor is active
  if (activeSensor == &ltrSensor)
  {
    spikeMultiplier = 1.3; // Trigger threshold remains slightly elevated to ignore track noise
  }

  // Trigger the spike when it goes ABOVE the multiplier
  if (!baselineExceeded && !adjustInProgress && float(c) >= float(blackBaseline) * spikeMultiplier)
  {
    baselineExceeded = true;
    if (useSerial)
    {
      Serial.print(F("\n[SPIKE DETECTED] Brightness: "));
      Serial.print(c);
      Serial.print(F(" (Threshold: "));
      Serial.print(float(blackBaseline) * spikeMultiplier);
      Serial.println(F(")"));
    }
  }
  // --- NEW: HYSTERESIS BUFFER ---
  // Only cancel the spike if it drops significantly BELOW a smaller multiplier (1.1).
  // This prevents tiny noisy dips in brightness from prematurely triggering the "False Alarm" abort!
  else if (baselineExceeded && float(c) < float(blackBaseline) * 1.1)
  {
    Serial.println(F("** Hysteresis protection activated **"));
    baselineExceeded = false;
  }
  return baselineExceeded;
}

uint16_t calculateBlackBaseline() // Retrieves the RGB value of "black" from EEPROM
{
  return colors[0].avgC;
}

void logBlackBaseline() // Reads RGB values of "black" for storing to EEPROM
{
  uint32_t totalR = 0, totalG = 0, totalB = 0, totalC = 0;
  for (int j = 0; j < numSamples; j++)
  {
    uint16_t r, g, b, c;
    getColorData(&r, &g, &b, &c);
    normalizeRaw(&r, &g, &b);
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

  float totalColor = avgR + avgG + avgB;
  float percentageRed = avgR / totalColor;
  float percentageGreen = avgG / totalColor;
  float percentageBlue = avgB / totalColor;

  uint8_t proportionRed = round(percentageRed * 255);
  uint8_t proportionGreen = round(percentageGreen * 255);
  uint8_t proportionBlue = round(percentageBlue * 255);
  uint16_t totalLuminance = avgC;

  RGBColor newColor = {proportionRed, proportionGreen, proportionBlue, totalLuminance};
  colors[0] = newColor; // Store the black color at index 0
  writeColorToEEPROM(0, newColor);
}

void checkForTags360() // This function spins from red to red and logs what tags are seen.
{
  unsigned long currentTime = millis();
  // Serial.print(F("currentTime = "));
  // Serial.println(currentTime);
  if (currentlyCheckingTags == false)
  {
    // Serial.println(F("Checking tags!"));
    currentlyCheckingTags = true;
    checkForTags360StartTime = currentTime;
    // Serial.print(F("Initialization start time = "));
    // Serial.println(checkForTags360StartTime);
  }

  if (activeColor > 0 && colorStatus[activeColor - 1] == -1) // Set red to first index position, 0
  {
    colorDetected(activeColor);
  }

  while (activeColor == 1)
  {
    // Serial.println(F("Move off red."));
    moveOffActiveColor(true);
  }

  while (!tagsChecked)
  {
    currentTime = millis();
    rotate(mediumSpeed, CW); // Rotate clockwise
    colorScan();
    if (activeColor > 0 && colorStatus[activeColor - 1] == -1)
    {
      if (!playerLeftOfDealerIdentified) // Use this opportunity to find player left of dealer in rigged games
      {
        playerLeftOfDealerIdentified = true;
        colorLeftOfDealer = activeColor;
        tag2Time = currentTime;
      }
      colorDetected(activeColor);
      totalCardsToDeal += initialRoundsToDeal;
      // Serial.print(F("Total cards to deal now = "));
      // Serial.println(totalCardsToDeal);
    }
    if (activeColor == 1 && (currentTime - checkForTags360StartTime > min360Interval))
    {
      // Serial.println("Found red again.");
      currentlyCheckingTags = false;
      tagsChecked = true;
      rotateStop();

      // Calculate the shortest path for War:
      unsigned long arc1Time = tag2Time - checkForTags360StartTime;
      unsigned long arc2Time = currentTime - tag2Time;

      if (arc1Time <= arc2Time)
      {
        redToTag2IsCW = true; // The CW path is shorter
      }
      else
      {
        redToTag2IsCW = false; // The CCW path is shorter
      }
    }
  }
  // Serial.println(F("Scan complete. Colors seen: "));
  // printDealtCardsInfo();
  // Count the number of tags present
  numTags = numColorsSeen();

  totalCardsToDeal = numTags * initialRoundsToDeal;
  // Serial.print(F("numTags | totalCardsToDeal = "));
  // Serial.print(numTags);
  // Serial.print(F(" | "));
  // Serial.println(totalCardsToDeal);

  // Make sure number of cards is appropriate for current game:
  if (currentGame == 3 && numTags > 2) // If we're playing war and there are more than 2 tags:
  {
    rotateStop();
    errorInProgress = true;
    displayErrorMessage("EROR - TOO MANY TAGS");
    currentDisplayState = ERROR;
    currentDealState = RESET_DEALR;
  }
  else if ((currentGame == 3 && numTags < 2) || (currentGame == 4 && numTags < 4)) // If we're playing war or hearts and there are too few tags:
  {
    rotateStop();
    errorInProgress = true;
    displayErrorMessage("EROR - NOT ENOUGH TAGS");
    currentDisplayState = ERROR;
    currentDealState = RESET_DEALR;
  }
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
  char buffer[40];
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
  display.writeDigitAscii(0, 'C');
  display.writeDigitAscii(1, '=');

  if (numberOfCards >= 10)
  {
    display.writeDigitAscii(2, (numberOfCards / 10) + '0');
    display.writeDigitAscii(3, (numberOfCards % 10) + '0');
  }
  else
  {
    display.writeDigitAscii(2, numberOfCards + '0');
    display.writeDigitAscii(3, ' '); // Blank space if single digit
  }
  display.writeDisplay();
}

void showPlayers() // Displays the currently selected number of players.
{
  display.clear();
  display.writeDigitAscii(0, 'P');
  display.writeDigitAscii(1, '=');

  if (numberOfPlayers >= 10)
  {
    display.writeDigitAscii(2, (numberOfPlayers / 10) + '0');
    display.writeDigitAscii(3, (numberOfPlayers % 10) + '0');
  }
  else
  {
    display.writeDigitAscii(2, numberOfPlayers + '0');
    display.writeDigitAscii(3, ' '); // Blank space if single digit
  }
  display.writeDisplay();
}

void startPreGameAnimation() // Starts the pre-game animation, which transitions into the blinking animation
{
  if (!initialAnimationInProgress && !initialAnimationComplete)
  {
    currentDisplayState = INTRO_ANIM;
    initialAnimationInProgress = true;
    // if (useSerial)
    // {
    //   Serial.println(F("Starting pre-game animation..."));
    // }
  }
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

void updateScrollText()
{
  if (currentDisplayState == SCREENSAVER)
    return;

  unsigned long currentTime = millis();

  if (currentTime - lastScrollTime >= scrollDelayTime)
  {
    lastScrollTime = currentTime;

    if (scrollIndex == -1)
    {
      display.clear();
      display.writeDigitAscii(0, message[0]);
      display.writeDigitAscii(1, message[1]);
      display.writeDigitAscii(2, message[2]);
      display.writeDigitAscii(3, message[3]);
      display.writeDisplay();

      scrollDelayTime = textStartHoldTime;
      // --- FIX: Start at offset 1 to prevent double-drawing the first frame! ---
      scrollIndex = 1;
    }
    // --- FIX: Change to '- 4' to prevent double-drawing the last frame! ---
    else if (scrollIndex < static_cast<int>(strlen(message)) - 4)
    {
      display.clear();
      display.writeDigitAscii(0, message[scrollIndex]);
      display.writeDigitAscii(1, message[scrollIndex + 1]);
      display.writeDigitAscii(2, message[scrollIndex + 2]);
      display.writeDigitAscii(3, message[scrollIndex + 3]);
      display.writeDisplay();

      scrollDelayTime = textSpeedInterval;
      scrollIndex++;
    }
    else
    {
      display.clear();
      int len = strlen(message);
      display.writeDigitAscii(0, message[len - 4]);
      display.writeDigitAscii(1, message[len - 3]);
      display.writeDigitAscii(2, message[len - 2]);
      display.writeDigitAscii(3, message[len - 1]);
      display.writeDisplay();

      scrollDelayTime = textEndHoldTime;
      scrollIndex = -1;
      messageRepetitions++;
      messageLine++;
      scrollingStarted = false;
      if (messageLine > 3)
      {
        messageLine = 0;
      }
      delay(10);
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

void displayFace_P(const char *wordP) // wordP points to PROGMEM
{
  char buf[5];             // 4 chars + null
  memcpy_P(buf, wordP, 4); // copy exactly 4 characters from PROGMEM
  buf[4] = '\0';
  displayFace(buf); // reuse existing RAM-based function
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

  if (handlingFlipCard)
  {
    displayFace("FLIP");
    return;
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

  case SCROLL_PLACE_TAGS_TEXT:
    if (!scrollingStarted && !scrollingComplete)
    {
      startScrollText("    PLACE TAGS, THEN TAP GREEN   ^", 0, textSpeedInterval, 1200);
      scrollingStarted = true;
    }
    else if (scrollingStarted && !scrollingComplete)
    {
      updateScrollText();
      if (messageRepetitions >= 2)
      {
        messageRepetitions = 0;
        scrollingComplete = true;
      }
    }
    else
    {
      advanceMenu();
    }
    break;

  case SCROLL_PICK_GAME_TEXT:
    if (!scrollingStarted && !scrollingComplete)
    {
      startScrollText("    PICK GAME WITH BLUE/YELLOW ^^ ", 0, textSpeedInterval, textEndHoldTime);
      scrollingStarted = true;
    }

    if (!scrollingComplete)
    {
      updateScrollText();
      if (messageRepetitions >= 2)
      {
        messageRepetitions = 0;
        scrollingComplete = true;
        scrollingStarted = false;
        blinkingAnimationActive = true;
      }
    }
    else
    {
      advanceMenu();
    }
    break;

  case SCREENSAVER:
    checkIfRigged();
    if (riggedGame)
    {
      runAnimation(evilScreensaverBlinking);
    }
    else
    {
      runAnimation(screensaverBlinking);
    }
    break;

  case SELECT_GAME:
    showGame();
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

  case SELECT_PLAYERS:
    showPlayers();
    break;

  case DEAL_CARDS:
    displayFace("DEAL");
    break;

  case ERROR:
    displayFace("EROR");
    delay(1000);
    fullExit = true;
    currentDealState = RESET_DEALR;
    break;

  case STRUGGLE:
    expressionStarted = millis();
    checkIfRigged();
    if (riggedGame)
    {
      if (markedCard)
      {
        displayFace_P(MONEY);
      }
      else
      {
        displayFace_P(SNEAKY);
      }
    }
    else
    {
      displayFace_P(EFFORT);
    }
    break;

  case LOOK_LEFT:
    displayFace_P(LEFT);
    break;

  case LOOK_RIGHT:
    displayFace_P(RIGHT);
    break;

  case LOOK_STRAIGHT:
    checkIfRigged();
    if (riggedGame)
    {
      displayFace_P(WILD);
    }
    else
    {
      displayFace_P(LOOK_BIG);
    }
    break;

  case FLIP:
    displayFace("FLIP");
    break;

  case TAGLESS_DISPLAY:
    if (!scrollingStarted)
    {
      delay(100);
      if (messageLine % 2 == 0)
      {
        startScrollText("REMOVE ALL TAGS EXCEPT RED.", 1000, textSpeedInterval, 1000);
      }
      else
      {
        startScrollText("GREEN TO START   ^", 1000, textSpeedInterval, 1000);
      }
      scrollingStarted = true;
    }

    if (messageRepetitions >= 2)
    {
      messageRepetitions = 0;
      scrollingStarted = false;
    }
    updateScrollText();

    if (scrollingComplete)
    {
      currentDealState = TAGLESS_DEAL;
    }
    break;

  case ERROR_OPTIONS:
    if (!scrollingStarted)
    {
      if (messageLine % 2 == 0)
        startScrollText("RETRY = GREEN^", 1000, textSpeedInterval, 1000);
      else
        startScrollText("EXIT = ^RED", 1000, textSpeedInterval, 1000);
      scrollingStarted = true;
    }
    updateScrollText();
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

static void getAnimFrame_P(const char (*frames)[5], uint8_t index, char out[5])
{
  memcpy_P(out, frames[index], 5); // copies 4 chars + '\0'
}

void runAnimation(const DisplayAnimation &animation)
{
  unsigned long currentTime = millis();
  static unsigned long lastAnimationTime = 0;
  static uint8_t currentFrame = 0;
  static const DisplayAnimation *lastAnimation = nullptr;
  static uint8_t lastDisplayedFrame = 255;

  char frameBuf[5]; // 4 chars + null

  if (lastAnimation != &animation)
  {
    currentFrame = 0;
    lastAnimation = &animation;
    lastAnimationTime = currentTime;
    lastDisplayedFrame = 255;

    getAnimFrame_P(animation.frames, currentFrame, frameBuf);
    displayFace(frameBuf);

    lastDisplayedFrame = currentFrame;
    return;
  }

  unsigned long interval = pgm_read_dword(&animation.intervals[currentFrame]);

  if (currentTime - lastAnimationTime >= interval)
  {
    lastAnimationTime = currentTime;

    if (&animation == &initialBlinking && currentFrame == animation.numFrames - 1)
    {
      currentFrame = 0;
      lastAnimation = nullptr;
      initialAnimationComplete = true;
      initialAnimationInProgress = false;
      return;
    }

    currentFrame = (currentFrame + 1) % animation.numFrames;

    getAnimFrame_P(animation.frames, currentFrame, frameBuf);
    displayFace(frameBuf);

    if (currentFrame != lastDisplayedFrame)
    {
      lastDisplayedFrame = currentFrame;
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
#pragma endregion 14 - Segment Display

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
    if (scrollInstructions)
    {
      currentDisplayState = SCROLL_PLACE_TAGS_TEXT;
    }
    else
    {
      buttonInitialization = true;
      currentDisplayState = SELECT_GAME;
      toolsMenuActive = false; // Switching to select game menu. Deactivating Tools menu.
    }

    break;

  case SCROLL_PLACE_TAGS_TEXT: // If we're in SCROLL_PLACE_TAGS_TEXT when advanceMenu() is called, we advance to SCROLL_PICK_GAME_TEXT.
    currentDisplayState = SCROLL_PICK_GAME_TEXT;
    break;

  case SCROLL_PICK_GAME_TEXT:
    if (blinkingAnimationActive)
    {
      currentDisplayState = SCREENSAVER;
    }
    else
    {
      currentDisplayState = SELECT_GAME;
      toolsMenuActive = false; // Switching to select game menu. Deactivating Tools menu.
    }
    break;

  case SCREENSAVER:
    blinkingAnimationActive = false;
    currentAnimation = nullptr; // Reset the active animation so if the screensaver starts again, it starts from the top.
    if (scrollInstructions)
    {
      currentGame = 0;
      currentDisplayState = SCROLL_PLACE_TAGS_TEXT;
    }
    else
    {
      buttonInitialization = true;
      currentDisplayState = SELECT_GAME;
    }
    break;

  case SELECT_GAME:
    if (currentGame == 0) // Current game is Go Fish
    {
      currentDisplayState = DEAL_CARDS;
      currentDealState = DEALING;
      remainingRoundsToDeal = goFishStartingCards;
      postCardsToDeal = 126; // Cards to deal after main deal (deal until deck is exhausted). 127 is the max value for a uint8_t.
      initialRoundsToDeal = remainingRoundsToDeal;
    }
    else if (currentGame == 1) // Current game is 21
    {
      currentDisplayState = DEAL_CARDS;
      currentDealState = DEALING;

      remainingRoundsToDeal = twentyOneStartingCards; // each player starts with 2 cards in 21
      postCardsToDeal = 1;                            // Cards to deal after main deal (only one round in 21)
      initialRoundsToDeal = remainingRoundsToDeal;
    }
    else if (currentGame == 2) // Current game is Crazy Eights
    {
      currentDisplayState = DEAL_CARDS;
      currentDealState = DEALING;
      remainingRoundsToDeal = crazyEightsStartingCards; // Crazy 8s deals 5 rounds initially
      postCardsToDeal = 126;                            // Cards to deal after main deal (deal until deck is exhausted)
      initialRoundsToDeal = remainingRoundsToDeal;
    }
    else if (currentGame == 3) // WAR
    {
      currentDisplayState = DEAL_CARDS;
      currentDealState = DEALING;
      remainingRoundsToDeal = warStartingCards; // War deals the full deck to two people.
      postCardsToDeal = 0;                      // There is no post-game in War.
      initialRoundsToDeal = remainingRoundsToDeal;
    }
    else if (currentGame == 4) // Hearts
    {
      currentDisplayState = DEAL_CARDS;
      currentDealState = DEALING;
      remainingRoundsToDeal = heartsStartingCards; // Hearts deals 13/player.
      postCardsToDeal = 0;                         // Cards to deal after main deal. Hearts has no post-deal.
      initialRoundsToDeal = remainingRoundsToDeal;
    }
    else if (currentGame == 5) // Rummy
    {
      currentDisplayState = DEAL_CARDS;
      currentDealState = DEALING;
      remainingRoundsToDeal = rummyStartingCards; // For two-player Rummy, this can be increased to 10. Better yet, it could be procedural based on how many tags are seen!
      postCardsToDeal = 126;                      // Cards to deal after main deal (deal until deck is exhausted).
      initialRoundsToDeal = remainingRoundsToDeal;
    }
    else if (currentGame == 6) // Custom Game or Tagless Deal
    {
      currentDisplayState = SELECT_CARDS;
      postCardsToDeal = 126; // Cards to deal after main deal (deal until deck is exhausted).
    }
    else if (currentGame == 7) // currentGame is "tools". This takes us into the tools menu.
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
      separatingCards = true;
      remainingRoundsToDeal = 52;
      currentDealState = DEALING;
      currentDisplayState = DEAL_CARDS;
      updateDisplay();
    }
    else if (toolsMenuActive && currentToolsMenu == 3) // COLOR TUNER
    {
      colorTuner();
    }
    else if (toolsMenuActive && currentToolsMenu == 4) // UV TUNER
    {
      uvSensorTuner();
    }
    else if (toolsMenuActive && currentToolsMenu == 5) // RESET COLOR VALUES TO DEFAULT
    {
      resetEEPROMToDefaults();
      displayFace("DONE");
      // printStoredColors();
      delay(1500);
      updateDisplay();
      fullExit = true;
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

    if (currentGame == 6)
    {
      currentDisplayState = SELECT_PLAYERS;
    }
    else
    {
      currentDisplayState = DEAL_CARDS;
      currentDealState = DEALING;
    }
    updateDisplay();
    break;

  case SELECT_PLAYERS:
    if (numPlayersLocked == false)
    {
      // Serial.print(F("Locked players number = "));
      // Serial.println(numberOfPlayers);
      numPlayersLocked = true;
      cardsInHandToDeal = numberOfPlayers;
    }

    if (currentGame == 6) // If current game is user-created...
    {
      currentDisplayState = TAGLESS_DISPLAY;
    }
    else
    {
      currentDisplayState = ERROR;
      gamesExit = true;
      currentDealState = RESET_DEALR;
    }
    updateDisplay();
    break;

  case DEAL_CARDS:
    currentDealState = DEALING;
    break;

  case TAGLESS_DISPLAY:
    messageLine = 0;
    scrollingComplete = true;
    currentDealState = TAGLESS_DEAL;
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
  case SELECT_PLAYERS:
    numCardsLocked = false;
    toolsMenuActive = false; // Switching to select game menu. Deactivating Tools menu.
    currentDisplayState = SELECT_CARDS;
    break;

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
    if (scrollInstructions) // If instructions are enabled, going back when in the "select game" menu goes to instructions.
    {
      currentDisplayState = SCROLL_PICK_GAME_TEXT;
      blinkingAnimationActive = false;
    }
    else // Otherwise, we just play the blinking animation.
    {
      initialAnimationComplete = false;
      currentDisplayState = INTRO_ANIM;
    }
    currentDealState = IDLE;
    break;

  case SCROLL_PICK_GAME_TEXT:
    messageRepetitions = 2; // Skip scrolling text when going back
    buttonInitialization = false;
    currentDisplayState = SCROLL_PLACE_TAGS_TEXT;
    blinkingAnimationActive = false;
    scrollingStarted = false;
    scrollingComplete = false;
    startPreGameAnimation();
    break;

  case SCROLL_PLACE_TAGS_TEXT:
    // We don't go back to the quick intro blink, we just stay put.
    buttonInitialization = false;
    break;

  case INTRO_ANIM:
    // Nothing happens when we go back, but we can make sure buttonInitialization = false so the screensaver starts.
    buttonInitialization = false;
    break;

  case SCREENSAVER:
    // If we're watching the screensaver and someone hits the back button, the system should wake up.
    advanceMenu();
    break;

  case TAGLESS_DISPLAY:
    numPlayersLocked = false;
    currentDisplayState = SELECT_PLAYERS;
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
  else if (currentDisplayState == SELECT_PLAYERS)
  {
    if (numberOfPlayers > 2)
    {
      numberOfPlayers--;
      // Serial.print(F("Number of players = "));
      // Serial.println(numberOfPlayers);
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
  else if (currentDisplayState == SELECT_PLAYERS)
  {
    numberOfPlayers++;
    // Serial.print(F("Number of players = "));
    // Serial.println(numberOfPlayers);
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
    currentDisplayState = LOOK_LEFT;
  }
  else if (direction == CCW) // CCW = "False"
  {
    rotatingCW = false;
    rotatingCCW = true;
    stopped = false;
    analogWrite(MOTOR_2_PWM, rotationSpeed);
    digitalWrite(MOTOR_2_PIN_1, LOW);
    digitalWrite(MOTOR_2_PIN_2, HIGH);
    currentDisplayState = LOOK_RIGHT;
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

void colorTuner() // Controls the "color tuning" operation that locks down RGB values for color tags, which can experience variation for a bunch of reasons, like room lighting and UV exposure fading.
{
  if (!scrollingStarted)
  {
    messageRepetitions = 1;
    scrollingStarted = true;
    scrollingComplete = false;
  }

  int messageCounter = 0;
  const char *messages[] = {
      "REMOVE TAGS.",
      "REPLACE UNDER SENSOR WHEN PROMPTED.",
      "TO CONFIRM, PRESS GREEN   ^"};
  const int numMessages = sizeof(messages) / sizeof(messages[0]);

  while (!scrollingComplete)
  {
    if (messageRepetitions >= 1)
    {
      messageRepetitions = 0;
      startScrollText(messages[messageCounter], textStartHoldTime, textSpeedInterval, textEndHoldTime);
      messageCounter = (messageCounter + 1) % numMessages; // Cycle through messages
    }
    updateScrollText();

    if (digitalRead(BUTTON_PIN_1) == LOW || digitalRead(BUTTON_PIN_2) == LOW || digitalRead(BUTTON_PIN_3) == LOW) // Pressing a button cancels the text animation.
    {
      messageCounter = 0;
      scrollingComplete = true;
      scrollingStarted = false;
      while (digitalRead(BUTTON_PIN_1) == LOW || digitalRead(BUTTON_PIN_2) == LOW || digitalRead(BUTTON_PIN_3) == LOW)
      {
        // Wait for "confirm" button to be released before proceeding.
      };
      break;
    }
    else if (digitalRead(BUTTON_PIN_4) == LOW)
    {
      messageCounter = 0;
      toolsExit = true;
      currentDealState = RESET_DEALR;
      updateDisplay();
      break;
    }
  }

  if (scrollingComplete)
  {
    logBlackBaseline();
    recordColors(1); // Start from index 1 (Red) instead of black.
    loadColorsFromEEPROM();
    displayFace("DONE");
    scrollingStarted = false; // reset in case we want to tune again
    scrollingComplete = false;
    insideDealrTools = false;
    delay(1500);

    currentDealState = IDLE;
    toolsMenuActive = false; // Switching to select game menu. Deactivating Tools menu.
    currentDisplayState = SELECT_GAME;
    updateDisplay();
  }
}

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
        toolsExit = true;
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
      getColorData(&r, &g, &b, &c);
      normalizeRaw(&r, &g, &b);
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

    float totalColor = avgR + avgG + avgB;
    float percentageRed = avgR / totalColor;
    float percentageGreen = avgG / totalColor;
    float percentageBlue = avgB / totalColor;

    uint8_t proportionRed = round(percentageRed * 255);
    uint8_t proportionGreen = round(percentageGreen * 255);
    uint8_t proportionBlue = round(percentageBlue * 255);
    uint16_t totalLuminance = avgC;

    RGBColor newColor = {proportionRed, proportionGreen, proportionBlue, totalLuminance};
    colors[i] = newColor;
    writeColorToEEPROM(i, newColor);
  }
}

void uvSensorTuner() // Controls the "uv tuning" operation that locks down the threshold visible light value for a card to be determined to be "marked.""
{
  if (!scrollingStarted)
  {
    messageRepetitions = 1;
    scrollingStarted = true;
    scrollingComplete = false;
  }

  int messageCounter = 0;
  const char *messages[] = {
      "STACK 10 UNMARKED CARDS IN DEALR.", // We only scan 5 cards, but if only 5 cards are in the hopper, we risk getting UV bleed around the edge of the last card. So we ask the user to stack a few extra.
      "PRESS GREEN TO START    ^"};
  const int numMessages = sizeof(messages) / sizeof(messages[0]);

  while (!scrollingComplete)
  {
    if (messageRepetitions >= 1)
    {
      messageRepetitions = 0;
      startScrollText(messages[messageCounter], textStartHoldTime, textSpeedInterval, textEndHoldTime);
      messageCounter = (messageCounter + 1) % numMessages; // Cycle through messages
    }
    updateScrollText();

    if (digitalRead(BUTTON_PIN_1) == LOW || digitalRead(BUTTON_PIN_2) == LOW || digitalRead(BUTTON_PIN_3) == LOW)
    {
      scrollingComplete = true;
      scrollingStarted = false;
      while (digitalRead(BUTTON_PIN_1) == LOW || digitalRead(BUTTON_PIN_2) == LOW || digitalRead(BUTTON_PIN_3) == LOW)
      {
        // Wait for "confirm" button to be released before proceeding.
      };
      break;
    }
    else if (digitalRead(BUTTON_PIN_4) == LOW)
    {
      toolsExit = true;
      currentDealState = RESET_DEALR;
      updateDisplay();
      break;
    }
  }

  if (scrollingComplete)
  {
    recordUVThreshold();
    loadStoredUVValueFromEEPROM(storedUVThreshold);
    displayFace("DONE");
    scrollingStarted = false;
    scrollingComplete = false;
    insideDealrTools = false;
    delay(1500);

    toolsExit = true;
    currentDealState = RESET_DEALR;
    updateDisplay();
  }
}

void recordUVThreshold() // Helper function for uvSensorTuner().
{
  int numUVReadings = 5;
  uint16_t uvReadings[numUVReadings] = {0};
  uint16_t maxUV = 0; // Variable for holding total UV values.

  for (int i = 0; i < numUVReadings; i++)
  {
    dealSingleCard();
    delay(100);
    uvReadings[i] = analogRead(UV_READER);
    if (uvReadings[i] > maxUV)
    {
      maxUV = uvReadings[i];
    }
    delay(200);
    cardDealt = false;
  }

  uint16_t inflatedMaxUV = maxUV + uvThresholdBuffer;

  EEPROM.put(UV_THRESHOLD_ADDR, inflatedMaxUV); // Save UV average to EEPROM (use an address that does not conflict with color values)

  // Convert the inflatedMaxUV to a 4-character string
  char uvValueStr[5]; // Buffer to hold the 4-character string (4 chars + null terminator)
  uvValueStr[0] = (inflatedMaxUV / 1000) % 10 + '0';
  uvValueStr[1] = (inflatedMaxUV / 100) % 10 + '0';
  uvValueStr[2] = (inflatedMaxUV / 10) % 10 + '0';
  uvValueStr[3] = inflatedMaxUV % 10 + '0';
  uvValueStr[4] = '\0';

  // Display the 4-character string on the display
  displayFace(uvValueStr);
  delay(1500);

  displayFace("SAVD");
  delay(1200);
}

void resetEEPROMToDefaults()
{
  const RGBColor *defaultsToUse;

  if (activeSensor == &nhySensor)
  {
    defaultsToUse = defaultColors_NHY3274TH;
  }
  else if (activeSensor == &ltrSensor)
  {
    defaultsToUse = defaultColors_LTR381RGB;
  }
  else
  {
    // Fallback — maybe no sensor detected
    return;
  }

  for (int i = 0; i < numColors; i++)
  {
    writeColorToEEPROM(i, defaultsToUse[i]);
  }

  EEPROM.put(UV_THRESHOLD_ADDR, defaultUVThreshold);
  EEPROM.write(EEPROM_VERSION_ADDR, EEPROM_VERSION);
}

#pragma endregion DEALR Tools

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
ERROR AND TIMEOUT HANDLING FUNCTIONS
*/
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#pragma region Errors and Timeout

void handleThrowingTimeout(unsigned long currentTime)
{
  const unsigned long retractDuration = 1000;

  // Fire once when timeout first detected
  if (!timeoutLatched && (currentTime - throwStart >= throwExpiration))
  {
    timeoutLatched = true; // lock until resetThrowCardState()
    retracting = true;
    retractStartTime = currentTime;

    feedCard.write(30); // retract wheel
    delay(20);
    flywheelOn(false);
    delay(20);
  }

  // Finish retract once after duration
  if (retracting && (currentTime - retractStartTime >= retractDuration))
  {
    retracting = false;
    retractDone = true;

    feedCard.write(90);
    delay(20);
    flywheelOff();
    delay(20);

    currentDealState = ERROR_RECOVERY;
    currentDisplayState = ERROR_OPTIONS;
    errorInProgress = true;
    cardDealt = true;
    scrollingStarted = false;
    messageLine = 0;
    updateDisplay();
  }

  // Reset latch only after state leaves recovery
  if (currentDealState == RESET_DEALR || currentDealState == IDLE)
  {
    timeoutLatched = false;
    retracting = false;
    retractDone = false;
    retractStartTime = 0;
  }
}

void handlePostDealOutcome()
{
  // If we’re in post-deal, go back to player decision
  if (postDeal)
  {
    currentDealState = AWAITING_PLAYER_DECISION;
    currentDisplayState = DEAL_CARDS; // or LOOK_STRAIGHT, whichever fits
    updateDisplay();
  }
  else
  {
    currentDealState = ADVANCING;
  }
}

void resetThrowCardState()
{
  previousSlideStep = -1;
  feedCard.write(90);
  slideStep = 0;
  throwingCard = false;
  cardDealt = false;
  overallTimeoutTag = millis();
  delay(10);
}

void handleFineAdjustTimeout() // Handles timeout for fine adjustment moves.
{
  unsigned long currentTime = millis();

  if (currentTime - adjustStart > errorTimeout && currentDealState != AWAITING_PLAYER_DECISION)
  {
    rotateStop(); // --- NEW: Stop the runaway motor! ---
    errorStartTime = currentTime;
    currentDisplayState = ERROR;
    currentDealState = RESET_DEALR; // Send it to the reset state
    errorInProgress = true;         // --- NEW: Flag the error so loops can break! ---
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
  checkIfRigged();
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
  markedCard = false;
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
  separatingCards = false;
  shufflingCards = false;
  slideStep = 0;
  startCheckingForMarked = false;
  tagsPresent = false;
  tagsChecked = false;
  taglessGame = false;
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

void checkTimeouts() // Checks to see whether device has been idle for a long time, and goes into "sleep" mode.
{
  unsigned long currentTime = millis();

  if (buttonInitialization && currentDealState == IDLE && !insideDealrTools && currentTime - overallTimeoutTag > timeUntilScreensaverStart) // List of some circumstances where we want to disable the overall timeout function.
  {
    // if (useSerial)
    // {
    //   Serial.println(F("System timed out. Returning to screensaver."));
    // }
    stopScrollText();
    buttonInitialization = false;
    blinkingAnimationActive = true;
    currentDisplayState = SCREENSAVER;
    currentDealState = IDLE;
    updateDisplay();
  }

  if (currentTime - markedLEDStart > markedLEDTimeout) // Turn off "marked card light" after 1 second
  {
    digitalWrite(LED_BUILTIN, LOW);
  }

  if (currentTime - lastRiggedCheck >= riggedCheckInterval) // Controls how often we check to see if the rig switch has been flipped.
  {
    lastRiggedCheck = currentTime; // Update the last check time
    checkIfRigged();               // Call the function to check if rigged
    if (toolsMenuActive)           // If we're currently using a DEALR tool, make sure we don't use any rigged game functions.
    {
      riggedGame = false;
    }
  }
}
#pragma endregion Errors and Timeout

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

bool allNonRiggedColorsFull() // Returns "true" if all non-rigged hands are full.
{
  for (int i = 0; i < 4; i++) // Assuming four players (Red, Yellow, Blue, Green)
  {
    if (i + 1 == riggedColor)
      continue; // Skip the rigged player (convert index to 1-based)

    if (colorStatus[i] == -1 || colorStatus[i] < initialRoundsToDeal)
    {
      return false; // At least one non-rigged player still needs cards.
    }
  }
  return true; // All non-rigged players have full hands.
}

bool isRiggedHandFull() // Returns "true" if red's hand is full.
{
  int riggedIndex = riggedColor - 1;

  if (colorStatus[riggedIndex] != -1 && colorStatus[riggedIndex] >= initialRoundsToDeal)
  {
    return true; // Rigged hand is full.
  }
  return false; // Rigged hand is not full.
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

bool isTooFarAhead(uint8_t activeColor)
{
  uint8_t activeColorIndex = activeColor - 1;
  uint8_t myCards = colorStatus[activeColorIndex];
  if (myCards == (uint8_t)-1)
    return false;

  uint8_t lowest = 255;
  for (int i = 0; i < maxTagColors; i++)
  {
    if (colorStatus[i] != -1 && i != activeColorIndex)
    {
      if (colorStatus[i] < lowest)
        lowest = colorStatus[i];
    }
  }
  if (lowest == 255)
    return false; // Only one player?

  return (myCards >= lowest + 8); // Enforces the 8-card max difference
}

bool allOtherHandsTooFarAheadOrFull(uint8_t checkColor)
{
  uint8_t checkColorIndex = checkColor - 1;
  for (int i = 0; i < maxTagColors; i++)
  {
    if (colorStatus[i] != -1 && i != checkColorIndex)
    {
      // If we find a player whose hand is NOT full and NOT too far ahead, return false
      if (colorStatus[i] < initialRoundsToDeal && !isTooFarAhead(i + 1))
      {
        return false;
      }
    }
  }
  return true; // Everyone else is locked out!
}

#pragma endregion Hand Evaluation

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
FUNCTIONS FOR READING AND WRITING TO EEPROM
*/
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#pragma region EEPROM

void initializeEEPROM()
{
  uint8_t version;
  EEPROM.get(EEPROM_VERSION_ADDR, version);

  if (version != EEPROM_VERSION)
  {
    // EEPROM not yet initialized — load default values based on detected sensor
    const RGBColor *defaultsToUse;

    if (activeSensor == &nhySensor)
    {
      defaultsToUse = defaultColors_NHY3274TH;
    }
    else if (activeSensor == &ltrSensor)
    {
      defaultsToUse = defaultColors_LTR381RGB;
    }
    else
    {
      // No valid sensor found — skip initialization
      return;
    }

    for (int i = 0; i < numColors; i++)
    {
      writeColorToEEPROM(i, defaultsToUse[i]);
    }

    EEPROM.put(UV_THRESHOLD_ADDR, defaultUVThreshold);
    EEPROM.write(EEPROM_VERSION_ADDR, EEPROM_VERSION); // Mark EEPROM initialized
  }

  // If EEPROM is already initialized, no action needed here.
}

// void writeColorToEEPROM(int index, RGBColor color) // Used for writing RGB values to EEPROM.
// {
//   int addr = index * sizeof(RGBColor) + 1;
//   EEPROM.put(addr, color);
// }

void writeColorToEEPROM(int index, RGBColor color) // Used for writing RGB values to EEPROM.
{
  int addr = index * sizeof(RGBColor) + 1;
  EEPROM.put(addr, color);

  // --- SERIAL OUTPUT FOR DEBUGGING ---
  if (useSerial)
  {
    const char *colorNames[] = {"BLAK", "RED ", "YELO", "BLUE", "GREE"};
    Serial.print(F("Saved: "));
    Serial.print(colorNames[index]);
    Serial.print(color.r);
    Serial.print(F(", "));
    Serial.print(color.g);
    Serial.print(F(", "));
    Serial.print(color.b);
    Serial.print(F(", "));
    Serial.println(color.avgC);
  }
}

void loadColorsFromEEPROM() // Whatever colors have been saved to EEPROM get loaded at startup using this function.
{
  for (int i = 0; i < numColors; i++)
  {
    colors[i] = readColorFromEEPROM(i);
  }
}

void loadStoredUVValueFromEEPROM(uint16_t &uvThreshold) // Loads stored UV threshold values from EEPROM on boot.
{
  EEPROM.get(UV_THRESHOLD_ADDR, uvThreshold);
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

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
HELPER FUNCTIONS FOR PRINTING TO SERIAL MONITOR
*/
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#pragma region Serial Monitor

void printStoredColors() // Prints RGB values for the colors that are stored (Default are Black, Red, Yellow, Green, and Blue).

{
  const char *colorNames[numColors] = {"BLAK", "RED ", "YELO", "BLUE", "GREE"};
  for (int i = 0; i < numColors; i++)
  {
    RGBColor color = colors[i];
    Serial.print(colorNames[i]);
    Serial.print(F(": ("));
    Serial.print(color.r);
    Serial.print(F(", "));
    Serial.print(color.g);
    Serial.print(F(", "));
    Serial.print(color.b);
    Serial.print(F(", "));
    Serial.print((int)color.avgC);
    Serial.println(F(")"));
  }
}

void printDealtCardsInfo() // Helper function for printing information on what colors have been seen to the serial monitor.
{
  Serial.print(F("Color status per color: "));
  for (uint8_t i = 0; i < maxTagColors; i++)
  {
    Serial.print(colorStatus[i]);
    Serial.print(F(" "));
  }
  Serial.println();
}

void printStoredUVThreshold() // Helper function for printing the stored UV threshold for determining when a card has been marked

{
  EEPROM.get(UV_THRESHOLD_ADDR, storedUVThreshold);
  Serial.print(F("Stored UV Threshld: "));
  Serial.println(storedUVThreshold);
}
#pragma endregion Serial Monitor
#pragma endregion FUNCTIONS

//////////////////////////////////////////////////////////////////////////////////////////////////////////
// END CODE
//////////////////////////////////////////////////////////////////////////////////////////////////////////