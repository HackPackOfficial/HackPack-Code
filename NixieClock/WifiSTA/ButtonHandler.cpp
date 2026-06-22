#include <Arduino.h>
#include "ButtonHandler.h"

// Pull in your types & globals only in the .cpp (safe include order)
#include "config.h"
#include "stateEnums.h"
#include "PanelColorController.h"
#include "TimeKeeper.h"
#include <OneButton.h>

// ---- externs for globals defined in your main project ----
extern PanelColorController display;
extern TimeKeeper          nixie;

extern OneButton b_top, dp_u, dp_d, dp_l, dp_r, dp_c;

extern TimerState timr;
extern AlarmState alrm;
extern ClockState clck_state;

extern DisplayState display_state;
extern UIState      ui_state;

extern int br_step;

// ---------------------- Alarm (red) ----------------------
static void onPressTop() {  Serial.println("Alarm pressed"); }

static void onLongPressStartTop() {
  Serial.println("Alarm long press start");
  timr = IDLE;
  alrm = UNSET;
  display_state = TIME;
  ui_state = NONE;
  clck_state = RUN_CLOCK;

  nixie.getClock().ringTime = 0;
  display.getSettings().cursorPos = 0;

  display.fillMsgColor();
  display.showString("clear ");
  display.displayPanels();
  delay(800);
}

static void onClickTop() {
  Serial.println("Alarm clicked");
  if (clck_state == RUN_TIMER && timr == ACTIVE) {
    timr = IDLE;
    if (alrm == SET) clck_state = RUN_ALARM;
    clck_state = RUN_CLOCK;

    display.fillMsgColor();
    display.showString("stop  ");
    display.displayPanels();
    delay(800);
    return;
  }
  if (alrm == PLAYING) {
    alrm = SET;
    display.fillMsgColor();
    display.showString("stop  ");
    display.displayPanels();
    delay(800);
  }
}

// ---------------------- Up ----------------------
static void onPressDPU() { Serial.println("Up pressed"); }
static void onLongPressStartDPU() { Serial.println("Up long press start"); }

static void onClickDPU() {
  Serial.println("Up clicked");

  if (display_state == TIME) {
    display.getSettings().brightness += br_step;
    display.getSettings().brightness = constrain(display.getSettings().brightness, 0, 255);
    display.setBrightness();
    return;
  }

  if (display_state == MENU) {
    ui_state = static_cast<UIState>((ui_state + 1) % NUM_UI);
  }

  if (display_state == EDIT) {
    switch (ui_state) {
      case NONE: break;
      case SET_TIME:
        if (display.getSettings().cursorPos == 0) {
          nixie.getClock().clckSet[0] = (nixie.getClock().clckSet[0] + 1) % 3;
          if (nixie.getClock().clckSet[0] == 2 && nixie.getClock().clckSet[1] > 3) {
            nixie.getClock().clckSet[1] = 0;
          }
        } else if (display.getSettings().cursorPos == 1 && nixie.getClock().clckSet[0] == 2) {
          nixie.getClock().clckSet[1] = (nixie.getClock().clckSet[1] + 1) % 4;
        } else if (display.getSettings().cursorPos == 2 || display.getSettings().cursorPos == 4) {
          int idx = display.getSettings().cursorPos;
          nixie.getClock().clckSet[idx] = (nixie.getClock().clckSet[idx] + 1) % 6;
        } else {
          int idx = display.getSettings().cursorPos;
          nixie.getClock().clckSet[idx] = (nixie.getClock().clckSet[idx] + 1) % 10;
        }
        break;

      case SET_ALARM:
        if (display.getSettings().cursorPos == 0) {
          nixie.getClock().alarmSet[0] = (nixie.getClock().alarmSet[0] + 1) % 3;
          if (nixie.getClock().alarmSet[0] == 2 && nixie.getClock().alarmSet[1] > 3) {
            nixie.getClock().alarmSet[1] = 0;
          }
        } else if (display.getSettings().cursorPos == 1 && nixie.getClock().clckSet[0] == 2) {
          nixie.getClock().alarmSet[1] = (nixie.getClock().alarmSet[1] + 1) % 4;
        } else if (display.getSettings().cursorPos == 2 || display.getSettings().cursorPos == 4) {
          int idx = display.getSettings().cursorPos;
          nixie.getClock().alarmSet[idx] = (nixie.getClock().alarmSet[idx] + 1) % 6;
        } else {
          int idx = display.getSettings().cursorPos;
          nixie.getClock().alarmSet[idx] = (nixie.getClock().alarmSet[idx] + 1) % 10;
        }
        break;

      case SET_TIMER: {
        int idx = display.getSettings().cursorPos;
        nixie.getClock().timerSet[idx] = (nixie.getClock().timerSet[idx] + 1) % 10;
        break;
      }

      case SET_COLOR:
        display.getSettings().colorPos = (display.getSettings().colorPos + 8) % 256;
        break;

      case SET_24:
        nixie.getClock()._24HrMode = !nixie.getClock()._24HrMode;
        break;

      case SET_ZERO:
        nixie.getClock().showLeadZero = !nixie.getClock().showLeadZero;
        break;
    }
  }
}

// ---------------------- Down ----------------------
static void onPressDPD() { Serial.println("Down pressed"); }
static void onLongPressStartDPD() { Serial.println("Down long press start"); }

static void onClickDPD() {
  Serial.println("Down clicked");

  if (display_state == TIME) {
    display.getSettings().brightness -= br_step;
    display.getSettings().brightness = constrain(display.getSettings().brightness, 0, 255);
    display.setBrightness();
    return;
  }

  if (display_state == MENU) {
    int uiMode = ui_state - 1;
    if (uiMode < 0) uiMode += NUM_UI;
    ui_state = static_cast<UIState>(uiMode);
  }

  if (display_state == EDIT) {
    switch (ui_state) {
      case NONE: break;

      case SET_TIME: {
        int c = display.getSettings().cursorPos;
        if (c == 0) {
          nixie.getClock().clckSet[0]--;
          if (nixie.getClock().clckSet[0] < 0) nixie.getClock().clckSet[0] += 3;
          if (nixie.getClock().clckSet[0] == 2 && nixie.getClock().clckSet[1] > 3) {
            nixie.getClock().clckSet[1] = 0;
          }
        } else if (c == 1 && nixie.getClock().clckSet[0] == 2) {
          nixie.getClock().clckSet[1]--;
          if (nixie.getClock().clckSet[1] < 0) nixie.getClock().clckSet[1] += 4;
        } else if (c == 2 || c == 4) {
          nixie.getClock().clckSet[c]--;
          if (nixie.getClock().clckSet[c] < 0) nixie.getClock().clckSet[c] += 6;
        } else {
          nixie.getClock().clckSet[c]--;
          if (nixie.getClock().clckSet[c] < 0) nixie.getClock().clckSet[c] += 10;
        }
        break;
      }

      case SET_ALARM: {
        int c = display.getSettings().cursorPos;
        if (c == 0) {
          nixie.getClock().alarmSet[0]--;
          if (nixie.getClock().alarmSet[0] < 0) nixie.getClock().alarmSet[0] += 3;
          if (nixie.getClock().alarmSet[0] == 2 && nixie.getClock().alarmSet[1] > 3) {
            nixie.getClock().alarmSet[1] = 0;
          }
        } else if (c == 1 && nixie.getClock().alarmSet[0] == 2) {
          nixie.getClock().alarmSet[1]--;
          if (nixie.getClock().alarmSet[1] < 0) nixie.getClock().alarmSet[1] += 4;
        } else if (c == 2 || c == 4) {
          nixie.getClock().alarmSet[c]--;
          if (nixie.getClock().alarmSet[c] < 0) nixie.getClock().alarmSet[c] += 6;
        } else {
          nixie.getClock().alarmSet[c]--;
          if (nixie.getClock().alarmSet[c] < 0) nixie.getClock().alarmSet[c] += 10;
        }
        break;
      }

      case SET_TIMER: {
        int c = display.getSettings().cursorPos;
        nixie.getClock().timerSet[c]--;
        if (nixie.getClock().timerSet[c] < 0) nixie.getClock().timerSet[c] += 10;
        break;
      }

      case SET_COLOR:
        display.getSettings().colorPos -= 8;
        if (display.getSettings().colorPos < 0) display.getSettings().colorPos += 256;
        break;

      case SET_24:
        nixie.getClock()._24HrMode = !nixie.getClock()._24HrMode;
        break;

      case SET_ZERO:
        nixie.getClock().showLeadZero = !nixie.getClock().showLeadZero;
        break;
    }
    return;
  }
}

// ---------------------- Left ----------------------
static void onPressDPL() { Serial.println("Left pressed"); }
static void onLongPressStartDPL() { Serial.println("Left long press start"); }

static void onClickDPL() {
  Serial.println("Left clicked");

  if (display_state == TIME) {
    int mode = static_cast<int>(col_state) - 1;
    if (mode < 0) mode += static_cast<int>(NUM_MODES);
    col_state = static_cast<ColorState>(mode);
    // restart animation to unwanted memory artifacts
    display.getColors().modeIndex = 0;
    display.getColors().modeChanged = true;
    return;
  }

  if (display_state == EDIT) {
    if (display.getSettings().cursorPos > 0) display.getSettings().cursorPos--;
    return;
  }
}

// ---------------------- Right ----------------------
static void onPressDPR() { Serial.println("Right pressed"); }
static void onLongPressStartDPR() { Serial.println("Right long press start"); }

static void onClickDPR() {
  Serial.println("Right clicked");

  if (display_state == TIME) {
    int mode = (static_cast<int>(col_state) + 1) % static_cast<int>(NUM_MODES);
    col_state = static_cast<ColorState>(mode);
    display.getColors().modeIndex = 0;
    display.getColors().modeChanged = true;
    return;
  }

  if (display_state == EDIT) {
    if (display.getSettings().cursorPos < 5) display.getSettings().cursorPos++;
    return;
  }
}

// ---------------------- Center ----------------------
static void onPressDPC() { Serial.println("Center pressed"); }

static void onLongPressStartDPC() {
  Serial.println("Center long press start");
  if (display_state == MENU) {
    display_state = TIME;
    display.getColors().modeIndex = 0;
    display.getColors().modeChanged = true;
    return;
  }
  if (display_state == TIME) {
    display_state = MENU;
    return;
  }
}

static void onClickDPC() {
  Serial.println("Center clicked");
  if (display_state == TIME) return;

  if (display_state == MENU) {
    display_state = EDIT;
    return;
  }

  if (display_state == EDIT) {
    display_state = TIME;
    switch (ui_state) {
      case NONE: timr = IDLE; break;

      case SET_TIME:
        nixie.getTime().s_hour   = nixie.getClock().clckSet[0] * 10 + nixie.getClock().clckSet[1];
        nixie.getTime().s_minute = nixie.getClock().clckSet[2] * 10 + nixie.getClock().clckSet[3];
        nixie.getTime().s_sec    = nixie.getClock().clckSet[4] * 10 + nixie.getClock().clckSet[5];
        nixie.setClockTime();
        nixie.getClock().timeSet = true;
        timr = IDLE; ui_state = NONE;

        display.fillMsgColor();
        display.showString("set   ");
        display.displayPanels();
        delay(800);
        break;

      case SET_ALARM:
        timr = IDLE; alrm = SET; clck_state = RUN_ALARM; ui_state = NONE;

        display.fillMsgColor();
        display.showString("set   ");
        display.displayPanels();
        delay(800);
        break;

      case SET_TIMER:
        timr = ACTIVE; clck_state = RUN_TIMER; ui_state = NONE;
        nixie.getClock().ringTime =
            millis() +
            nixie.getClock().timerSet[0] * 1000UL * 60 * 60 * 10 +
            nixie.getClock().timerSet[1] * 1000UL * 60 * 60 +
            nixie.getClock().timerSet[2] * 1000UL * 60 * 10 +
            nixie.getClock().timerSet[3] * 1000UL * 60 +
            nixie.getClock().timerSet[4] * 1000UL * 10 +
            nixie.getClock().timerSet[5] * 1000UL;

        display.fillMsgColor();
        display.showString("start ");
        display.displayPanels();
        delay(800);
        break;

      case SET_COLOR:
      case SET_24:
      case SET_ZERO:
        ui_state = NONE;
        display.fillMsgColor();
        display.showString("set   ");
        display.displayPanels();
        delay(800);
        break;
    }
  }
}

// ---------------------- Public API ----------------------
void setupButtons() {
  b_top.attachPress(onPressTop);
  b_top.attachLongPressStart(onLongPressStartTop);
  b_top.attachClick(onClickTop);

  dp_u.attachPress(onPressDPU);
  dp_u.attachLongPressStart(onLongPressStartDPU);
  dp_u.attachClick(onClickDPU);

  dp_d.attachPress(onPressDPD);
  dp_d.attachLongPressStart(onLongPressStartDPD);
  dp_d.attachClick(onClickDPD);

  dp_l.attachPress(onPressDPL);
  dp_l.attachLongPressStart(onLongPressStartDPL);
  dp_l.attachClick(onClickDPL);

  dp_r.attachPress(onPressDPR);
  dp_r.attachLongPressStart(onLongPressStartDPR);
  dp_r.attachClick(onClickDPR);

  dp_c.attachPress(onPressDPC);
  dp_c.attachLongPressStart(onLongPressStartDPC);
  dp_c.attachClick(onClickDPC);
}

void tickButtons() {
  b_top.tick();
  dp_u.tick();
  dp_d.tick();
  dp_l.tick();
  dp_r.tick();
  dp_c.tick();
}
