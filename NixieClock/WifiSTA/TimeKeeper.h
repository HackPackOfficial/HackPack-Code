#pragma once
#include <Arduino.h>
#include <TimeLib.h>

class PanelColorController;

// timer states: running, not running
enum TimerState {IDLE, ACTIVE};
extern TimerState timr;

// alarm states: set, unset, playing
enum AlarmState {SET, UNSET, PLAYING};
extern AlarmState alrm;

// clock state: run default clock, run alarm clock, or run timer.
enum ClockState {RUN_CLOCK, RUN_ALARM, RUN_TIMER};
extern ClockState clck_state;

// Time variables
struct Time{
  // on start(s_)
  int s_year;
  int s_month;
  int s_day;
  int s_hour;
  int s_minute;
  int s_sec;
  int s_m_sec;

  // current (c_)
  int c_year;
  int c_month;
  int c_day;
  int c_hour;
  int c_minute;
  int c_sec;
  int c_m_sec;

  bool isPm;

  int sunRise;
  int sunSet;
};

// Clock variables
struct Clock{
  bool timeSet;               // has clock been set

  bool showLeadZero;          // 01:00 vs 1:00
  bool _24HrMode;             // 24 Hour Time
  
  unsigned long startTime;    // internal millis when clock was set

  unsigned long ringTime;     // internal millis to play speaker message
  
  int clckSet[6];             // init values for manual time settings, 12:00
  int alarmSet[6];            // init values for alarm in 24 hour time, 7am
  int timerSet[6];            // init values for timer. 5 min
};

class TimeKeeper {
public:
  TimeKeeper(int rec, int play);

  // “set clock” helpers
  void setClockAbsolute();  // sets date and time
  void setClockTime();      // sets time
  void setClockDate();      // sets date

  // Get Time from NTP webserver
  bool syncFromNTP(unsigned long timeoutMs = 10000);

  // Render the current time to panels
  void updateTime(PanelColorController& panels);

  // Alarm/timer logic (timer now needs the controller to render)
  void updateAlarm();
  void updateTimer(PanelColorController& panels);

  // Struct getters
  Time&  getTime();
  Clock& getClock(); 

private:
  Time  tme;
  Clock clck;
    
  int recordPin;
  int playPin;
    
  bool Hr24Time;
  bool displayLeadingZero;
};
