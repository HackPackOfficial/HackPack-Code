#include "config.h"

// ----------------- Panel / Clock Defaults -----------------
const bool Hr24Time           = true;               // LVL2 OK. -- ///{"options":["true","false"]}
const bool displayLeadingZero = true;               // LVL2 OK. -- ///{"options":["true","false"]}

// color wheel position used by your color modes (for SOLID/GRADIENT/etc)
const int  bootColor          = 128;                // LVL2 OK. Values [0 - 255] -- ///{"min":0,"max":255}

// brightness used by PanelColorController
const int  bootBrightness     = 200;                // LVL2 OK. Values [0 - 255] -- ///{"min":0,"max":255}

// base “message color” used by fillMsgColor() and possibly other UI bits
const uint8_t msg_col[3]      = { 255, 140, 20 };   // LVL2 OK. Currently Amber Color. Values [0 - 255] for all fields -- ///{"col":35,"min":0,"max":255} ///{"col":40,"min":0,"max":255} ///{"col":45,"min":0,"max":255}

// initial animated color mode: 0=RAINBOW, 1=SOLID, 2=GRADIENT, 3=FLOW, 4=WIPE, 5=PULSE, 6=BOUNCE
const uint8_t bootMode        = 3;                  // LVL2 OK. Values [0 - 6] -- ///{"min":0,"max":6}


// ----------------- Marquee-Specific -----------------
const uint16_t MARQUEE_SCROLL_MS = 250;             //  LVL 2 OK. Milliseconds each frame is shown. Values [100 - 750]. -- ///{"min":100,"max":750}

// You can edit this string freely – it’s what scrolls across the panels:
const char MARQUEE_MESSAGE[] =  "   your content here   ";    // LVL 2 OK. Type your message to be displayed here. Maybe best to keep under 100 charachters -- ///{}
