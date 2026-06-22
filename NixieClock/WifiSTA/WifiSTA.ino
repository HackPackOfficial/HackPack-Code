#include <WiFi.h>
#include <ESPmDNS.h>
#include <time.h>
#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>
#include <OneButton.h>
#include <Adafruit_NeoPixel.h>

#include "config.h"           // User editable variables
#include "WebServerManager.h"
#include "ButtonHandler.h"
#include "PanelColorController.h"
#include "UnderGlow.h"
#include "TimeKeeper.h"
#include "stateEnums.h"

// For ESP32-S3 SuperMini
/*
#define BTN_TOP 1
#define BTN_UP 5
#define BTN_DOWN 4
#define BTN_LEFT 3
#define BTN_RIGHT 2
#define BTN_CENTER 6

#define LED_PIN  8
#define GLOW_PIN  9

#define REC 12
#define PLAY 13
*/

//For Meizhi ESP32-C3 Mini 

#define BTN_TOP 19
#define BTN_CENTER 10
#define BTN_UP 9
#define BTN_DOWN 8
#define BTN_LEFT 7
#define BTN_RIGHT 6

#define LED_PIN  0
#define GLOW_PIN  1

#define REC 4
#define PLAY 5


// Object Declarations -----------------------------------------------------

//Buttons
OneButton b_top(BTN_TOP, true);
OneButton dp_u(BTN_UP, true);
OneButton dp_d(BTN_DOWN, true);
OneButton dp_l(BTN_LEFT, true);
OneButton dp_r(BTN_RIGHT, true);
OneButton dp_c(BTN_CENTER, true);

// Web server on port 80
AsyncWebServer server(80);

// LEDs
PanelColorController display(LED_PIN, 42);
UnderGlow glow(GLOW_PIN, 13);

// Clock Timekeeping
TimeKeeper nixie(REC, PLAY);    // pins for clock UI to interact w audio


//Web Page Handling
WebServerManager web(server, display, nixie, glow);

// States
DisplayState display_state = TIME;
UIState ui_state = NONE;

// Static IP (so the web UI is always at the same address)
IPAddress sta_local_ip(192, 168, 1, 222);
IPAddress sta_gateway (192, 168, 1, 1);
IPAddress sta_subnet  (255, 255, 255, 0);

// How often to resync from NTP (once per 6 hours)
const unsigned long NTP_SYNC_INTERVAL_MS = 6UL * 60UL * 60UL * 1000UL;

unsigned long lastNtpSync = 0;

void runFace(){
  if(millis() > nextFaceTime){
    nextFaceTime = millis() + random(20000, 45000);

    display.showString("O __O ");
    display.displayPanels();
    delay(1000);
    display.showString(" O__ O");
    display.displayPanels();
    delay(1000);
    display.showString(" O<>O ");
    display.displayPanels();
    delay(500);
    display.showString(" -<>- ");
    display.displayPanels();
    delay(250);
    display.showString(" O<>O ");
    display.displayPanels();
    delay(1500);
    display.showString(" ^<>^ ");
    display.displayPanels();
    delay(1500);
  }
}

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> LOOP & SETUP <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
void setup() {
  Serial.begin(115200);

  DefaultHeaders::Instance().addHeader("Connection", "close");
  DefaultHeaders::Instance().addHeader("Cache-Control", "no-store, no-cache, must-revalidate");

  setupButtons();
  display.begin();
  glow.begin(glowBrightness);
  pinMode(REC, OUTPUT);
  pinMode(PLAY, OUTPUT);

  web.begin();
}


// Loop ------------------------------

void loop() {
  // Run High Level Nixie Clock Actions
  tickButtons();
  
  display.update();
  glow.update(
  display,                      // PanelColorController&
  nixie,
  display_state == TIME,
  clck_state == RUN_TIMER,
  clck_state == RUN_ALARM
  );

  if(display_state == TIME){ 
    //runFace();                // Uncomment this for face character animation
    switch(clck_state){
      case RUN_CLOCK: 
        nixie.updateTime(display);
        display.displayPanels();
        break;
      case RUN_TIMER:
        nixie.updateTimer(display);
        display.displayPanels();
        break;
      case RUN_ALARM:
        nixie.updateTime(display);
        nixie.updateAlarm();
        display.displayPanels();
        break;
    }
    // Run Webpage & Auto Time Sync
    web.loop();
    return;
  }

  if(display_state == MENU){
    switch(ui_state){
      case NONE:
        display.fillMsgColor();
        display.showString("select");
        display.displayPanels();
        break;
      case SET_TIME:
        display.fillMsgColor();
        display.showString("t-set ");
        display.displayPanels();
        break;
      case SET_ALARM:
        display.fillMsgColor();
        display.showString("alarm ");
        display.displayPanels();
        break;
      case SET_TIMER:
        display.fillMsgColor();
        display.showString("timer ");
        display.displayPanels();
        break;
      case SET_COLOR:
        display.fillMsgColor();
        display.showString("color ");
        display.displayPanels();
        break;
      case SET_24:
        display.fillMsgColor();
        display.showString("24hour");
        display.displayPanels();
        break;
      case SET_ZERO:
        display.fillMsgColor();
        display.showString("zeros ");
        display.displayPanels();
        break;
    }
    return;
  }

  if(display_state == EDIT){
    switch(ui_state){
      case NONE:
        display_state = TIME; 
        break;
      case SET_TIME:
        display.setDisplayChars('0' + (char)nixie.getClock().clckSet[0], '0' + (char)nixie.getClock().clckSet[1], '0' + (char)nixie.getClock().clckSet[2], '0' + (char)nixie.getClock().clckSet[3], '0' + (char)nixie.getClock().clckSet[4], '0' + (char)nixie.getClock().clckSet[5]);
        display.highlightCursor(255, 0, 0);
        display.displayPanels();
        break;
      case SET_ALARM:
        display.setDisplayChars('0' + (char)nixie.getClock().alarmSet[0], '0' + (char)nixie.getClock().alarmSet[1], '0' + (char)nixie.getClock().alarmSet[2], '0' + (char)nixie.getClock().alarmSet[3], '0' + (char)nixie.getClock().alarmSet[4], '0' + (char)nixie.getClock().alarmSet[5]);
        display.highlightCursor(255, 0, 0);
        display.displayPanels();
        break;
      case SET_TIMER:
        display.setDisplayChars('0' + (char)nixie.getClock().timerSet[0], '0' + (char)nixie.getClock().timerSet[1], '0' + (char)nixie.getClock().timerSet[2], '0' + (char)nixie.getClock().timerSet[3], '0' + (char)nixie.getClock().timerSet[4], '0' + (char)nixie.getClock().timerSet[5]);
        display.highlightCursor(255, 0, 0);
        display.displayPanels();
        break;
      case SET_COLOR:
        display.fillColorUI();
        display.showString("------");
        display.displayPanels();
        break;
      case SET_24:
        display.fillMsgColor();
        if(nixie.getClock()._24HrMode){
          display.showString("yes   ");
        } else {
          display.showString("no    ");
        }
        display.displayPanels();
        break;
      case SET_ZERO:
        display.fillMsgColor();
        if(nixie.getClock().showLeadZero){
          display.showString("yes   ");
        } else {
          display.showString("no    ");
        }
        display.displayPanels();
        break;
    }
    return;
  }
}

