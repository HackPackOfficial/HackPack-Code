#include <avr/pgmspace.h>
#include <Arduino.h>
#ifndef GameConfig
#define GameConfig

// HANDY TOGGLES AND VALUES
bool useSerial = false;                               // Enables serial output for debugging. Set to false to disable serial output. Many statements need manual uncommenting for memory reasons.
uint16_t textSpeedInterval = 200;                      // How fast do you read?? Amount of time (in ms) between frames of scrolling text (Lower number = faster text scrolling).
uint16_t textStartHoldTime = 800;                      // Amount of time (in ms) scrolling text should pause before advancing.
uint16_t textEndHoldTime = 800;                        // Amount of time (in ms) that scrolling text should pause at the end of a scroll.
const unsigned long throwExpiration = 5000;            // If, when trying to deal a card, we take longer than this amount of time, throw an error.
const unsigned long expressionDuration = 500;          // DEALR makes faces when it deals cards. This value determines the amount of time it makes the face for.
const unsigned long errorTimeout = 6000;               // For rotations where we should have found a tag, but didn't, we throw an error after this amount of time.
const unsigned long reverseFeedTime = 400;             // Amount of time to reverse the feed servo after a deal (successful or unsuccessful).
const unsigned long flipDisplayDuration = 800;         // How long to display "FLIP" on the screen in milliseconds.

// STARTING CARDS PER PLAYER
const uint8_t goFishStartingCards = 5;
const uint8_t twentyOneStartingCards = 2;
const uint8_t crazyEightsStartingCards = 5;
const uint8_t warStartingCards = 26; // 52 cards / 2 players
const uint8_t heartsStartingCards = 13;
const uint8_t rummyStartingCards = 7;

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
EDITING DEALR'S DEALING FACES
While dealing, your Card Dealing Robot can make all kinds of faces. You can modify what these look like by editing the symbols between the quotes. Just remember, every
face must be exactly four characters long, including spaces.
*/
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

const char EFFORT[] PROGMEM = "X  X";
const char LEFT[] PROGMEM = ">  >";
const char RIGHT[] PROGMEM = "<  <";
const char LOOK_BIG[] PROGMEM = "O  O";

struct DisplayAnimation
{
    const char (*frames)[5];        // PROGMEM: array of 5-char strings (4 chars + '\0')
    const unsigned long *intervals; // PROGMEM: intervals table
    uint8_t numFrames;
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
const char introFrames[][5] PROGMEM = {
    "O  O", // Frame 1
    "-  -", // Frame 2
    "O  O"  // Frame 3
};
const unsigned long introIntervals[] PROGMEM = {
    1100, // Interval 1
    75,   // Interval 2
    1100  // Interval 3
};
const DisplayAnimation initialBlinking = {introFrames, introIntervals, ARRAY_SIZE(introFrames)};

#endif // GameConfig