#pragma once
#include <Arduino.h> 

//User inputs
extern const char* ssid;
extern const char* password;

extern int bootMode;                    
extern int bootColor;
extern int bootBrightness;

extern uint8_t glow_col;
extern int glowBrightness;

extern bool Hr24Time;
extern bool displayLeadingZero;

extern int sunrise_hr;
extern int sunset_hr;

extern int br_step; 

extern long nextFaceTime;

extern int nextBlinks;
extern bool blinks;

extern uint8_t msg_col[3];