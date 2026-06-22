// PanelColorController.cpp
#include "PanelColorController.h"
#include "glyphMap.h"
#include "config.h"
#include "stateEnums.h"

ColorState col_state = static_cast<ColorState>(bootMode);

PanelColorController::PanelColorController(int pin, int numPixels)
  : strip(numPixels, pin, NEO_GRB + NEO_KHZ800){


  // Initialize structs
  panels = {{{false}}, {{{0}}}, {'*', '*', '*', '*', '*', '*'}};
  colors = {0, 0, 0, 50, true, true, 0, 0, true, 0, true, 0, 0, 0, 0, 0, 0};
  usr = {0, bootColor, bootBrightness, msg_col[0], msg_col[1], msg_col[2]};
}

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> Setup <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
void PanelColorController::begin() {
  strip.begin();
  strip.show();
  strip.setBrightness(usr.brightness);
}

void PanelColorController::setBrightness() {
  strip.setBrightness(usr.brightness);
  strip.show();
}

// >>>>>>>>>>>>>>>>>>>>>>>>>>>> Struct Getters <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
Panels& PanelColorController::getPanels() {
  return panels;
}

Colors& PanelColorController::getColors() {
  return colors;
}

Settings& PanelColorController::getSettings() {
  return usr;
}

// >>>>>>>>>>>>>>>>>>>>>>>>>>> Public Functions <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
void PanelColorController::highlightCursor(uint8_t r, uint8_t g, uint8_t b){
  for(int i = 0; i < 7; i++){
    panels.displayColors[usr.cursorPos][i][0] = r;
    panels.displayColors[usr.cursorPos][i][1] = g;
    panels.displayColors[usr.cursorPos][i][2] = b;    
  }
}

void PanelColorController::fillColorUI(){
  for(int i = 0; i < 6; i++){
    for(int j = 0; j < 7; j++){
      panels.displayColors[i][j][0] = Red(Wheel(usr.colorPos));
      panels.displayColors[i][j][1] = Green(Wheel(usr.colorPos));
      panels.displayColors[i][j][2] = Blue(Wheel(usr.colorPos));
    }  
  }
}

void PanelColorController::fillMsgColor(){
  for(int i = 0; i < 6; i++){
    for(int j = 0; j < 7; j++){
      panels.displayColors[i][j][0] = usr.msg_r;
      panels.displayColors[i][j][1] = usr.msg_g;
      panels.displayColors[i][j][2] = usr.msg_b; 
    }   
  }
}

void PanelColorController:: displayPanels(){
  for(int i = 0; i < 6; i++){
    for(int j = 0; j < 7; j++){
      if(panels.display[i][j]){
        strip.setPixelColor(i * 7 + j, 
                            strip.Color(panels.displayColors[i][j][0], 
                                        panels.displayColors[i][j][1],
                                        panels.displayColors[i][j][2]));
      } else {
        strip.setPixelColor(i * 7 + j, strip.Color(0, 0, 0));
      }
    }
  }
  strip.show();
}

uint32_t PanelColorController::Wheel(uint8_t WheelPos) {  // returns color of rainbow from 0 - 255
  WheelPos = 255 - WheelPos;
  if(WheelPos < 85) {
    return strip.Color(255 - WheelPos * 3, 0, WheelPos * 3);
  }
  if(WheelPos < 170) {
    WheelPos -= 85;
    return strip.Color(0, WheelPos * 3, 255 - WheelPos * 3);
  }
  WheelPos -= 170;
  return strip.Color(WheelPos * 3, 255 - WheelPos * 3, 0);
}

// >>>>>>>>>>>>>>>>>>>>>>>>> Color Mode Setup <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
void PanelColorController::modeSetup() {
  switch(col_state){
    case RAINBOW:
      colors.totalSteps = 255;
      colors.t_frame = 50;
      break;
    case SOLID:
      colors.totalSteps = 1;
      colors.t_frame = 50;
      break;
    case GRADIENT:
      colors.totalSteps = 50;
      colors.t_frame = 50;
      colors.strt_col1 = Wheel(usr.colorPos + random(-30, 30));
      colors.strt_col2 = Wheel(usr.colorPos + random(-30, 30));
      colors.end_col1 = Wheel(usr.colorPos + random(-30, 30));
      colors.end_col2 = Wheel(usr.colorPos + random(-30, 30));
      colors.now_col1 = colors.strt_col1;
      colors.now_col2 = colors.strt_col2;
      break;
    case FLOW:
      colors.totalSteps = 50;
      colors.t_frame = 20;
      colors.strt_col1 = Wheel(random(0, 255));
      colors.strt_col2 = Wheel(random(0, 255));
      colors.now_col1 = colors.strt_col1;
      colors.now_col2 = colors.strt_col2;
      colors.end_col1 = Wheel(random(0, 255));
      colors.end_col2 = Wheel(random(0, 255));
      break;
    case WIPE:{
      colors.totalSteps = 30;
      colors.t_frame = 20;
      colors.wipeIndex = 0;
      colors.wipeCol = true;
      uint8_t col = random(0, 255);
      while(abs(col - usr.colorPos) < 15){
        col = random(0, 255);
      }
      colors.strt_col1 = Wheel(col);
      break;
    }
    case PULSE:{
      colors.totalSteps = 40;
      colors.t_frame = 10;
      colors.pulseIndex = 0;
      colors.pulseDir = true;
      uint8_t col = random(0, 255);
      while(abs(col - usr.colorPos) < 15){
        col = random(0, 255);
      }
      colors.strt_col1 = Wheel(col);
      break;
    }
    case BOUNCE:{
      colors.totalSteps = 40;
      colors.t_frame = 10;
      colors.bounceIndex = 0;
      colors.bounceDir = true;
      uint8_t col = random(0, 255);
      while(abs(col - usr.colorPos) < 15){
        col = random(0, 255);
      }
      colors.strt_col1 = Wheel(usr.colorPos);
      colors.strt_col2 = Wheel(col);
      for(int i = 0; i < 6; i++){
        for(int j = 0; j< 7; j++){
          panels.displayColors[i][j][0] = Red(colors.strt_col1);
          panels.displayColors[i][j][1] = Green(colors.strt_col1);
          panels.displayColors[i][j][2] = Blue(colors.strt_col1);
        }
      }
      displayPanels();
      break;
    }
    default:
      break;
  }
}

void PanelColorController::update() {
  if(millis() - colors.lastRecolor < colors.t_frame) return;
  colors.lastRecolor = millis();

  if(colors.modeChanged){
    modeSetup();
    colors.modeChanged = false;
  }

  switch(col_state){
    case RAINBOW: updateRainbow(); break;
    case SOLID: updateSolid(); break;
    case GRADIENT: updateGradient(); break;
    case FLOW: updateFlow(); break;
    case WIPE: updateWipe(); break;
    case PULSE: updatePulse(); break;
    case BOUNCE: updateBounce(); break;
    default: break;
  }
  Increment(colors.totalSteps);
}

void PanelColorController::Increment(int totalSteps){
  colors.modeIndex++;
  if(colors.modeIndex > totalSteps){
    colors.modeIndex = 0;
    onComplete();
  }
}

void PanelColorController::onComplete() {
  switch(col_state){
    case GRADIENT:
      colors.strt_col1 = colors.end_col1;
      colors.strt_col2 = colors.end_col2;
      colors.now_col1 = colors.strt_col1;
      colors.now_col2 = colors.strt_col2;
      colors.end_col1 = Wheel(usr.colorPos + random(-30, 30));
      colors.end_col2 = Wheel(usr.colorPos + random(-30, 30));
      break;
    case FLOW: 
      colors.strt_col1 = colors.end_col1;
      colors.strt_col2 = colors.end_col2;
      colors.now_col1 = colors.strt_col1;
      colors.now_col2 = colors.strt_col2;
      colors.end_col1 = Wheel(random(0, 255));
      colors.end_col2 = Wheel(random(0, 255));
      break;
    case WIPE:
      colors.wipeIndex++;
      if(colors.wipeIndex > 5){
        colors.wipeCol = !colors.wipeCol;
        colors.wipeIndex = 0;
        if(colors.wipeCol){
          uint8_t col = random(0, 255);
          while(abs(col - usr.colorPos) < 15){
            col = random(0, 255);
          }
          colors.strt_col1 = Wheel(col);
        }
      }
      break;
    case PULSE:
      if(colors.pulseDir){ // expanding pulse
        colors.pulseIndex++;
      }
      else{  // contracting pulse
        colors.pulseIndex--;
      }
      if(colors.pulseIndex == 0 && !colors.pulseDir){ // if done contracting, make new color
        uint8_t col = random(0, 255);
        while(abs(col - usr.colorPos) < 15){
          col = random(0, 255);
        }
        colors.strt_col1 = Wheel(col);
      }
      if(colors.pulseIndex > 3){ // done expanding, swap direction
        colors.pulseDir = !colors.pulseDir;
        colors.pulseIndex = 3;
      }
      if(colors.pulseIndex < 0){ // done contracting, swap direction
        colors.pulseDir = !colors.pulseDir;
        colors.pulseIndex = 1;
      }      
      break;
    case BOUNCE:
      if(colors.bounceDir){
        colors.bounceIndex++;
      } 
      else {
        colors.bounceIndex--;
      }
      if(colors.bounceIndex > 5){
        uint8_t col = random(0, 255);
        while(abs(col - usr.colorPos) < 15){
          col = random(0, 255);
        }
        colors.strt_col1 = colors.strt_col2;
        colors.strt_col2 = Wheel(col);
        colors.bounceDir = !colors.bounceDir;
        colors.bounceIndex = 5;
      }
      if(colors.bounceIndex < 0){
        uint8_t col = random(0, 255);
        while(abs(col - usr.colorPos) < 15){
          col = random(0, 255);
        }
        colors.strt_col1 = colors.strt_col2;
        colors.strt_col2 = Wheel(col);
        colors.bounceDir = !colors.bounceDir;
        colors.bounceIndex = 0;
      }
      break;
    default:
      break;
  }
}

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>> Color Mode Logic <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
void PanelColorController::updateRainbow(){
   for(int i = 0; i < 6; i++){
    for(int j = 0; j < 7; j++){
      uint32_t c = Wheel(25 * i + 5 * j +  colors.modeIndex);
      panels.displayColors[i][j][0] = Red(c);
      panels.displayColors[i][j][1] = Green(c);
      panels.displayColors[i][j][2] = Blue(c);
    }
  }
}

void PanelColorController::updateSolid(){
  for(int i = 0; i < 6; i++){
    for(int j = 0; j < 7; j++){
      panels.displayColors[i][j][0] = Red(Wheel(usr.colorPos));
      panels.displayColors[i][j][1] = Green(Wheel(usr.colorPos));
      panels.displayColors[i][j][2] = Blue(Wheel(usr.colorPos));          
    }
  }
}

void PanelColorController::updateGradient(){
  for(int i = 0; i < 6; i++){
    for(int j = 0; j < 7; j++){
      uint32_t panelColor = colorFade(colors.now_col1, colors.now_col2, i, 5);
      panels.displayColors[i][j][0] = Red(panelColor);
      panels.displayColors[i][j][1] = Green(panelColor);
      panels.displayColors[i][j][2] = Blue(panelColor);
    }
  }

  colors.now_col1 = colorFade(colors.strt_col1, colors.end_col1,  colors.modeIndex, colors.totalSteps);
  colors.now_col2 = colorFade(colors.strt_col2, colors.end_col2,  colors.modeIndex, colors.totalSteps);
}

void PanelColorController::updateFlow(){
  for(int i = 0; i < 6; i++){
    for(int j = 0; j < 7; j++){
      uint32_t panelColor = colorFade(colors.now_col1, colors.now_col2, i, 5);
      panels.displayColors[i][j][0] = Red(panelColor);
      panels.displayColors[i][j][1] = Green(panelColor);
      panels.displayColors[i][j][2] = Blue(panelColor);
    }
  }

  colors.now_col1 = colorFade(colors.strt_col1, colors.end_col1,  colors.modeIndex, colors.totalSteps);
  colors.now_col2 = colorFade(colors.strt_col2, colors.end_col2,  colors.modeIndex, colors.totalSteps);
}

void PanelColorController::updateWipe(){
  if(colors.wipeCol){
    uint32_t fadeCol = colorFade(Wheel(usr.colorPos), colors.strt_col1,  colors.modeIndex, colors.totalSteps);

    for(int i = 0; i < 6; i++){
      for(int j = 0; j < 7; j++){
        if(i < colors.wipeIndex){
          panels.displayColors[i][j][0] = Red(colors.strt_col1);
          panels.displayColors[i][j][1] = Green(colors.strt_col1);
          panels.displayColors[i][j][2] = Blue(colors.strt_col1);
        } 
        if(i == colors.wipeIndex) {
          panels.displayColors[i][j][0] = Red(fadeCol);
          panels.displayColors[i][j][1] = Green(fadeCol);
          panels.displayColors[i][j][2] = Blue(fadeCol); 
        }
        if(i > colors.wipeIndex) {
          panels.displayColors[i][j][0] = Red(Wheel(usr.colorPos));
          panels.displayColors[i][j][1] = Green(Wheel(usr.colorPos));
          panels.displayColors[i][j][2] = Blue(Wheel(usr.colorPos)); 
        }
      }
    }
  }

  else {
    uint32_t fadeCol = colorFade(colors.strt_col1, Wheel(usr.colorPos),  colors.modeIndex, colors.totalSteps);

    for(int i = 0; i < 6; i++){
      for(int j = 0; j < 7; j++){
        if(i > colors.wipeIndex){
          panels.displayColors[i][j][0] = Red(colors.strt_col1);
          panels.displayColors[i][j][1] = Green(colors.strt_col1);
          panels.displayColors[i][j][2] = Blue(colors.strt_col1);
        } 
        if(i == colors.wipeIndex) {
          panels.displayColors[i][j][0] = Red(fadeCol);
          panels.displayColors[i][j][1] = Green(fadeCol);
          panels.displayColors[i][j][2] = Blue(fadeCol); 
        }
        if(i < colors.wipeIndex) {
          panels.displayColors[i][j][0] = Red(Wheel(usr.colorPos));
          panels.displayColors[i][j][1] = Green(Wheel(usr.colorPos));
          panels.displayColors[i][j][2] = Blue(Wheel(usr.colorPos)); 
        }
      }
    }
  }
}

void PanelColorController::updatePulse(){
  for(int i = 0; i < 6; i++){
    for(int j = 0; j < 7; j++){
      if(i < 3 - colors.pulseIndex  || i > 2 + colors.pulseIndex){
        uint32_t col = Wheel(usr.colorPos);
        panels.displayColors[i][j][0] = Red(col);
        panels.displayColors[i][j][1] = Green(col);
        panels.displayColors[i][j][2] = Blue(col);
      } 

      else if(i == 3 - colors.pulseIndex || i == 2 + colors.pulseIndex){
        uint32_t col = 0;
        if(colors.pulseDir){
          col = colorFade(Wheel(usr.colorPos), colors.strt_col1, colors.modeIndex, colors.totalSteps);
        } else {
          col = colorFade(colors.strt_col1, Wheel(usr.colorPos), colors.modeIndex, colors.totalSteps);
        }

        panels.displayColors[i][j][0] = Red(col);
        panels.displayColors[i][j][1] = Green(col);
        panels.displayColors[i][j][2] = Blue(col);
      } 

      else{
        panels.displayColors[i][j][0] = Red(colors.strt_col1);
        panels.displayColors[i][j][1] = Green(colors.strt_col1);
        panels.displayColors[i][j][2] = Blue(colors.strt_col1);
      }
    }
  }
}

void PanelColorController::updateBounce(){
  if(colors.bounceIndex == 5 && !colors.bounceDir){
    for(int j = 0; j < 7; j++){
      uint32_t col = 0;
      col = colorFade(colors.strt_col1, colors.strt_col2, colors.modeIndex, colors.totalSteps);
    
      panels.displayColors[5][j][0] = Red(col);
      panels.displayColors[5][j][1] = Green(col);
      panels.displayColors[5][j][2] = Blue(col);
    }
    return;
  }
  else if(colors.bounceIndex == 0 && colors.bounceDir){
     for(int j = 0; j < 7; j++){
      uint32_t col = 0;
      col = colorFade(colors.strt_col1, colors.strt_col2, colors.modeIndex, colors.totalSteps);
    
      panels.displayColors[0][j][0] = Red(col);
      panels.displayColors[0][j][1] = Green(col);
      panels.displayColors[0][j][2] = Blue(col);
    }
    return;
  }

  for(int i = 0; i < 6; i++){
    for(int j = 0; j < 7; j++){
      if(colors.bounceDir){
        if(i == colors.bounceIndex - 1){
          uint32_t col = 0;
          col = colorFade(colors.strt_col2, Wheel(usr.colorPos), colors.modeIndex, colors.totalSteps);
          
          panels.displayColors[i][j][0] = Red(col);
          panels.displayColors[i][j][1] = Green(col);
          panels.displayColors[i][j][2] = Blue(col);
        }

        else if(i == colors.bounceIndex){
          uint32_t col = 0;
          col = colorFade(Wheel(usr.colorPos), colors.strt_col2, colors.modeIndex, colors.totalSteps);

          panels.displayColors[i][j][0] = Red(col);
          panels.displayColors[i][j][1] = Green(col);
          panels.displayColors[i][j][2] = Blue(col);
        }

        else{
          uint32_t col = Wheel(usr.colorPos);
          panels.displayColors[i][j][0] = Red(col);
          panels.displayColors[i][j][1] = Green(col);
          panels.displayColors[i][j][2] = Blue(col);
        }
      }

      else{
        if(i == colors.bounceIndex + 1){
          uint32_t col = 0;
          col = colorFade(colors.strt_col2, Wheel(usr.colorPos), colors.modeIndex, colors.totalSteps);

          panels.displayColors[i][j][0] = Red(col);
          panels.displayColors[i][j][1] = Green(col);
          panels.displayColors[i][j][2] = Blue(col);
        }

        else if(i == colors.bounceIndex){
          uint32_t col = 0;
          col = colorFade(Wheel(usr.colorPos), colors.strt_col2, colors.modeIndex, colors.totalSteps);
          
          panels.displayColors[i][j][0] = Red(col);
          panels.displayColors[i][j][1] = Green(col);
          panels.displayColors[i][j][2] = Blue(col);
        }

        else{
          uint32_t col = Wheel(usr.colorPos);
          panels.displayColors[i][j][0] = Red(col);
          panels.displayColors[i][j][1] = Green(col);
          panels.displayColors[i][j][2] = Blue(col);
        }
      }
    }
  }
}

// ===== display text API =====

void PanelColorController::showString(const char* s) {
  char buf[6] = {' ',' ',' ',' ',' ',' '};
  if(s) for(int i=0;i<6 && s[i];i++) buf[i] = sanitizeChar(s[i]);
  setDisplayChars(buf[0], buf[1], buf[2], buf[3], buf[4], buf[5]);
}

void PanelColorController::showString(const String& s) {
  char buf[6] = {' ',' ',' ',' ',' ',' '};
  for(int i=0;i<6 && i<s.length(); i++) buf[i] = sanitizeChar(s[i]);
  setDisplayChars(buf[0], buf[1], buf[2], buf[3], buf[4], buf[5]);
}

void PanelColorController::setDisplayChars(char c0,char c1,char c2,char c3,char c4,char c5) {
  char chars[6] = {
    sanitizeChar(c0),sanitizeChar(c1),sanitizeChar(c2),
    sanitizeChar(c3),sanitizeChar(c4),sanitizeChar(c5)
  };

  for(int p=0;p<6;p++) {
    bool segs[7];
    mapCharToSegments(chars[p], segs);
    for(int s=0;s<7;s++) panels.display[p][s] = segs[s];
    panels.panelSet[p] = chars[p];
  }
}

void PanelColorController::showCharAt(uint8_t idx, char c) {
  // exit if panel index out of range
  if(idx>5) return;
  c = sanitizeChar(c);
  bool segs[7];
  mapCharToSegments(c, segs);
  // fill display array to enable segments
  for(int s=0;s<7;s++) panels.display[idx][s] = segs[s];
  // update current glyph status
  panels.panelSet[idx] = c;
}

// ===== mapping helpers =====

char PanelColorController::sanitizeChar(char c) const {
  //collapse all letters to lowercase
  if(c>='A' && c<='Z') return c-'A'+'a';
  return c;
}

void PanelColorController::mapCharToSegments(char val, bool out7[7]) const {
  for (size_t i = 0; i < sizeof(NIXIE_CHARSET); ++i) {
    if (NIXIE_CHARSET[i] == val) {
      memcpy(out7, NIXIE_CHARSEGMENTS[i], 7);
      return;
    }
  }
  //return off if char is not found
  for(int s = 0; s < 7; s++) out7[s]=0;
}

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> COLOR HELPER FUNCTIONS <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
uint32_t PanelColorController::dimColor(uint32_t col, float percent){
  uint8_t blue = Blue(col)* percent / 100.0;
  uint8_t green = Green(col)* percent / 100.0;
  uint8_t red = Red(col)* percent / 100.0;

  return strip.Color(red, green, blue);
}

uint32_t PanelColorController::getPixelColor(uint16_t index) const {
  return strip.getPixelColor(index);
}

uint32_t PanelColorController::makeColor(uint8_t r, uint8_t g, uint8_t b) const {
  return strip.Color(r, g, b);
}

uint32_t PanelColorController::colorFade(uint32_t col_from, uint32_t col_to, int index, int t_steps){
  uint32_t r_from = Red(col_from);
  uint32_t g_from = Green(col_from);
  uint32_t b_from = Blue(col_from);
  
  uint32_t r_to = Red(col_to);
  uint32_t g_to = Green(col_to);
  uint32_t b_to = Blue(col_to);

  uint8_t r_fade = (r_from * (t_steps - index) + r_to * index) / t_steps;
  uint8_t g_fade = (g_from * (t_steps - index) + g_to * index) / t_steps;
  uint8_t b_fade = (b_from * (t_steps - index) + b_to * index) / t_steps;

  return(strip.Color(r_fade, g_fade, b_fade));
}

uint8_t PanelColorController::Red(uint32_t c){
  return (c >> 16);
}

uint8_t PanelColorController::Green(uint32_t c){
  return (c >> 8);
}

uint8_t PanelColorController::Blue(uint32_t c){
  return c;
}