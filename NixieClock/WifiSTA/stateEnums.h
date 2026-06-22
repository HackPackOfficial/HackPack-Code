#pragma once

enum DisplayState { TIME, MENU, EDIT };
extern DisplayState display_state;

enum UIState { NONE, SET_TIME, SET_ALARM, SET_TIMER, SET_24, SET_ZERO, SET_COLOR, NUM_UI };
extern UIState ui_state;
