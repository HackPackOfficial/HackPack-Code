// Enums that we need defined centrally for multiple files to access

#ifndef ENUMS_H
#define ENUMS_H

#include "PinDefinitions.h"

// DEAL STATE: Tracks DEALR's operational states
enum dealState
{
    IDLE,                     // Enter this state when we are in menus or finished a game.
    INITIALIZING,             // Enter this state when we want to initialize our rotational direction to the red tag.
    DEALING,                  // Enter this state when we want to deal a single card.
    ADVANCING,                // Enter this state when we want to advance from one tag to the next.
    AWAITING_PLAYER_DECISION, // Enter this state when we're pausing for a player input.
    RESET_DEALR,              // Enter this state when we want to reset dealr completely, including all of its state machine flags.
    TAGLESS_DEAL,             // Enter this state when we want to deal to an arbitrary number of players, only using the red tag for timekeeping.
    ERROR_RECOVERY            // Enter this state when we want to recover from an error condition.
};

// DISPLAY STATE: Tracks what is displayed on the 14-segment screen.
enum displayState
{
    INTRO_ANIM,             // Very first blinking animation that occurs on boot.
    SCROLL_PLACE_TAGS_TEXT, // First instructions that occur after blinking animation.
    SCROLL_PICK_GAME_TEXT,  // Animation that scrolls "PICK GAME USING BLUE/YELLOW" instructions before game selection menu.
    SELECT_GAME,            // Displays the select game menu.
    SELECT_TOOL,            // Displays the tools menu.
    SELECT_PLAYERS,         // Displays the number of players selection menu (tagless deal only).
    SELECT_CARDS,           // Displays the number of cards selection menu.
    DEAL_CARDS,             // Controls what is displayed when dealing a card.
    ERROR,                  // Displays "EROR" when an error happens.
    STRUGGLE,               // Struggling face.
    LOOK_LEFT,              // Looking left face.
    LOOK_RIGHT,             // Looking right face.
    LOOK_STRAIGHT,          // Open eyes face.
    FLIP,                   // Display state for the word "FLIP".
    TAGLESS_DISPLAY,        // Displays info on "tagless" deals, which can support >4 people.
    SCREENSAVER,            // Screensaver animation that occurs after timeout
    ERROR_OPTIONS           // Displays options after an error has occurred.
};

#endif // ENUMS_H