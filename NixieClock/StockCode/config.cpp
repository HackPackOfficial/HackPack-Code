// config.cpp
#include "config.h"

// Access Point credentials
const char* ssid = "Hack Pack - Nixie Clock";
const char* password = NULL;

// Startup Animation Pattern & Color
int bootMode = 6;                 // LVL 2 OK. Values [0 - 6].
int bootColor = 200;              // LVL 2 OK. Values [0 - 255].
int bootBrightness = 150;         // LVL 2 OK. Values [0 - 255].

uint8_t glow_col = 200;           // LVL 2 OK. Values [0 - 255].
int glowBrightness = 128;         // LVL 2 OK. Values [0 - 255].

// Clock Display Preferences
bool Hr24Time = false;            // LVL 2 OK
bool displayLeadingZero = false;  // LVL 2 OK

int sunrise_hr = 6;               // LVL 2 OK. Values [0 - 23].
int sunset_hr = 19;               // LVL 2 OK. Values [0 - 23]. 

// Brightness increment value
int br_step = 15;                 // LVL 2 OK. Values [1 - 30].

// Face animation variables
long nextFaceTime = 0;            
int nextBlinks = 0;
bool blinks = true;

// Color for System messages
uint8_t msg_col[3] = {255, 255, 255};     // LVL 2 OK. Values [1 - 30] for all fields.