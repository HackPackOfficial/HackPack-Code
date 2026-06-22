// PanelColorController.h
/*

*/
#pragma once
#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include "glyphMap.h"

enum ColorState { RAINBOW, SOLID, GRADIENT, FLOW, WIPE, PULSE, BOUNCE, NUM_MODES };
extern ColorState col_state;

//Clock Panels
struct Panels{
  bool display[6][7];              // character segment mapping 
  uint8_t displayColors[6][7][3];  // Color segment mapping
  char panelSet[6];                // characters shown on panels
};

//Color mode Variables
struct Colors{
  unsigned long lastRecolor;  // when was color last updated

  int modeIndex;              // current index of animation
  int totalSteps;             // total steps of animation
  int t_frame;                // animation frame duration,  default 50ms
  bool modeChanged;           // flag for mode setup on start !!!IMPORTANT!!!

  bool wipeCol;               // is wiping new color
  int wipeIndex;              // number of panels wiped

  int pulseIndex;             // number of panels pulsed
  bool pulseDir;              // pulse expand or contract
  
  int bounceIndex;            // panel that it being faded
  bool bounceDir;             // animation direction

  uint32_t strt_col1;         // save some colors to use
  uint32_t strt_col2;
  uint32_t end_col1;
  uint32_t end_col2;
  uint32_t now_col1;
  uint32_t now_col2;
};

// User Inputs
struct Settings{
  int cursorPos;              // cursor index

  int colorPos;               // Color Wheel index
  int brightness;             // 0 - 255

  uint8_t msg_r;              // ui message color
  uint8_t msg_g;
  uint8_t msg_b;
};


class PanelColorController {
public:
    PanelColorController(int pin, int numPixels); // initializes LEDs

    // LED Setup
    void begin();                 // initalize neoPixels
    void setBrightness();         // 0 - 255 
    
    // Struct Getters
    Panels& getPanels();          // segment & color mapping, current chars
    Colors& getColors();          // mode data, frame count, saved colors, etc.
    Settings& getSettings();      // User inputs of settings and colors. 

    // UI Coloring
    void highlightCursor(uint8_t r, uint8_t g, uint8_t b);  // set cursor RGB
    void fillColorUI();                              // Set color for UI menu
    void fillMsgColor();                             // Set color for system msg
    
    // Animation 
    void update();                // step the animation frame

    //Panels
    void showString(const String& s);               // e.g. "abc123"
    void showString(const char* s);                 // If you prefer a C-string overload:
    void setDisplayChars(char c0, char c1, char c2, char c3, char c4, char c5);
    void showCharAt(uint8_t panelIndex, char c);    // 0..5
    void displayPanels();                           // equivalent to .show();


    // Color Utility Functions
    uint32_t Wheel(uint8_t pos);
    uint32_t dimColor(uint32_t col, float percent);
    uint32_t getPixelColor(uint16_t index) const;
    uint32_t makeColor(uint8_t r, uint8_t g, uint8_t b) const;
    uint32_t colorFade(uint32_t c1, uint32_t c2, int step, int maxSteps);
    uint8_t Red(uint32_t c), Green(uint32_t c), Blue(uint32_t c);


// these functions execute their actions based upon the values in the  above structs.
private:
    Adafruit_NeoPixel strip;

    Panels panels;
    Colors colors;
    Settings usr;

    // Animation
    void modeSetup();
    void onComplete();
    void Increment(int totalSteps);

    // Animation Logic
    void updateRainbow();
    void updateSolid();
    void updateGradient();
    void updateFlow();
    void updateWipe();
    void updatePulse();
    void updateBounce();

    // Panel Char Mapping
    void mapCharToSegments(char val, bool out7[7]) const;
};
