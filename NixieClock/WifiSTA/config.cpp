// config.cpp
#include "config.h"

// Access Point credentials 
const char* your_ssid = "up dog";                       // Enter your Home WiFi SSID -- ///{}
const char* your_password = "NiceTryHacker";            // And Password to access STA Mode. -- ///{}

// Startup Animation Pattern & Color
int bootMode = 6;             // 0 - Rainbow, 1 - Solid, 2 - Gradient, 3 - Flow, 4 - Wipe, 5 - Pulse, 6 - Bounce -- ///{"min":0,"max":6}
int bootColor = 200;          // 0 - 255 -- ///{"min":0,"max":255}
int bootBrightness = 150;     // 0 - 255 -- ///{"min":0,"max":255}

uint8_t glow_col = 200;       // 0 - 255 -- ///{"min":0,"max":255}
int glowBrightness = 128;     // 0 - 255 -- ///{"min":0,"max":255}

// Clock Display Preferences
bool Hr24Time = false; ///{"options":["true","false"]}
bool displayLeadingZero = false; ///{"options":["true","false"]}

int sunrise_hr = 6;   // lights sun icon, can be used as AM -- ///{"min":0,"max":23}
int sunset_hr = 19;   // lights moon icon, can be used as PM -- ///{"min":0,"max":23}

// Brightness increment value
int br_step = 15; ///{"min":1,"max":30}

// Face animation variables
long nextFaceTime = 0;    // Time for the first run of the cute face animation.
int nextBlinks = 0;       
bool blinks = true;

// Color for System messages
uint8_t msg_col[3] = {255, 255, 255};  // System message color [0 - 255] for all fields -- ///{"col":23,"min":1,"max":30} ///{"col":28,"min":1,"max":30} ///{"col":33,"min":1,"max":30}