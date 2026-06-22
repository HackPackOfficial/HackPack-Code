#include <Arduino.h>
#include <time.h>
#include "TimeKeeper.h"
#include "PanelColorController.h"
#include "config.h"
#include "stateEnums.h"

TimerState timr;
AlarmState alrm;
ClockState clck_state;

TimeKeeper::TimeKeeper(int rec, int play){
  recordPin = rec;
  playPin = play;

  // init structs
  tme = {0,0,0,0,0,0,0,  0,0,0,0,0,0,0,  false, 0,0};
  clck = {false, displayLeadingZero, Hr24Time, 0, 0, {1,2,0,0,0,0}, {0,7,0,0,0,0}, {0,0,0,5,0,0}};

  // clock state
  timr = IDLE;
  alrm = UNSET;
  clck_state = RUN_CLOCK;
}

Time& TimeKeeper::getTime(){ return tme; }
Clock& TimeKeeper::getClock(){ return clck; }

void TimeKeeper::setClockAbsolute() { setTime(tme.s_hour, tme.s_minute, tme.s_sec, tme.s_day,   tme.s_month, tme.s_year); }
void TimeKeeper::setClockTime()     { setTime(tme.s_hour, tme.s_minute, tme.s_sec, tme.c_day,   tme.c_month, tme.c_year); }
void TimeKeeper::setClockDate()     { setTime(tme.c_hour, tme.c_minute, tme.c_sec, tme.s_day,   tme.s_month, tme.s_year); }

bool TimeKeeper::syncFromNTP(unsigned long timeoutMs) {
  struct tm timeinfo;

  if (!getLocalTime(&timeinfo, timeoutMs)) {
    Serial.println("Failed to obtain NTP time");
    return false;
  }

  // Fill the "stored absolute time" fields that setClockAbsolute() uses
  tme.s_year   = timeinfo.tm_year + 1900;   // tm_year is years since 1900
  tme.s_month  = timeinfo.tm_mon + 1;       // tm_mon is 0..11
  tme.s_day    = timeinfo.tm_mday;
  tme.s_hour   = timeinfo.tm_hour;
  tme.s_minute = timeinfo.tm_min;
  tme.s_sec    = timeinfo.tm_sec;

  // store ms
  //tme.s_m_sec  = (millis() % 1000);

  // This sets startTime, internal counters, etc.
  setClockAbsolute();
  clck.startTime = millis();
  clck.timeSet = true;

  Serial.printf(
    "TimeKeeper::syncFromNTP -> %04d-%02d-%02d %02d:%02d:%02d\n",
    tme.s_year, tme.s_month, tme.s_day, tme.s_hour, tme.s_minute, tme.s_sec
  );

  return true;
}


void TimeKeeper::updateTime(PanelColorController& panels) {
  unsigned long elapsed = millis() - clck.startTime;

  // AM/PM flag
  tme.isPm = !(hour() >= sunrise_hr && hour() <= sunset_hr);

  // 12/24 hr handling based on user setting in PanelColorController
  if (!clck._24HrMode && hour() > 12) {
    tme.c_hour = hour() - 12;
  } else if (!clck._24HrMode && hour() == 0 && clck.timeSet) {
    tme.c_hour = hour() + 12;
  } else {
    tme.c_hour = hour();
  }


  tme.c_minute = minute();
  tme.c_sec    = second();
  tme.c_day    = day();
  tme.c_month  = month();
  tme.c_year   = year();

  // update currnet ms (may be needed for animations)
  int total_ms = tme.s_m_sec + (elapsed % 1000);
  if (total_ms >= 1000) total_ms -= 1000;
  tme.c_m_sec = total_ms;

  // Handle Lead Zero
  if(!clck.showLeadZero && tme.c_hour < 10 && clck.timeSet){
    // Render HHMMSS without first zero digit
    panels.setDisplayChars(
      (char)(' '),
      (char)('0' + (tme.c_hour   % 10)),
      (char)('0' + (tme.c_minute / 10)),
      (char)('0' + (tme.c_minute % 10)),
      (char)('0' + (tme.c_sec    / 10)),
      (char)('0' + (tme.c_sec    % 10))
    );
  } else {
    // Render HHMMSS
    panels.setDisplayChars(
      (char)('0' + (tme.c_hour   / 10)),
      (char)('0' + (tme.c_hour   % 10)),
      (char)('0' + (tme.c_minute / 10)),
      (char)('0' + (tme.c_minute % 10)),
      (char)('0' + (tme.c_sec    / 10)),
      (char)('0' + (tme.c_sec    % 10))
    );
  }
}

void TimeKeeper::updateAlarm(){
  // if not playing and now is target time
  if (hour()   == clck.alarmSet[0] * 10 + clck.alarmSet[1] &&
      minute() == clck.alarmSet[2] * 10 + clck.alarmSet[3] &&
      second() == clck.alarmSet[4] * 10 + clck.alarmSet[5] &&
      alrm != PLAYING)
  {
    alrm = PLAYING;
    clck.ringTime = millis();
  }

  // if is playing
  if (alrm == PLAYING){
    // holds the play pin high for 100ms every 10s
    if ((millis() - clck.ringTime) % 10000 < 100){
      digitalWrite(playPin, HIGH);
      Serial.println("playing");
    } else {
      digitalWrite(playPin, LOW);
    }
  } else {
    digitalWrite(playPin, LOW);
  }

  // pressing top button will disable PLAYING flag
}

// ticks down timer and renders remaining HH:MM:SS
void TimeKeeper::updateTimer(PanelColorController& panels){
  // check if timer has ended
  if (millis() >= clck.ringTime){
    if (timr == ACTIVE){ 
      // trigger the sound every 10 sec 
      if ((millis() - clck.ringTime) % 10000 < 100){
        digitalWrite(playPin, HIGH);
        Serial.println("sound start");
      } else {
        digitalWrite(playPin, LOW);
      }
    } else { 
      digitalWrite(playPin, LOW);
    }
    panels.setDisplayChars('0','0','0','0','0','0');
    return;
  } 

  // Calculate Time Left
  unsigned long timeLeft    = clck.ringTime - millis();
  unsigned int secondsLeft  = (timeLeft / 1000) % 60;
  unsigned int minutesLeft  = (timeLeft / 1000 / 60) % 60;
  unsigned int hoursLeft    = (timeLeft / 1000 / 3600);

  // Split to digits
  unsigned int d0 = hoursLeft   / 10;
  unsigned int d1 = hoursLeft   % 10;
  unsigned int d2 = minutesLeft / 10;
  unsigned int d3 = minutesLeft % 10;
  unsigned int d4 = secondsLeft / 10;
  unsigned int d5 = secondsLeft % 10;

  panels.setDisplayChars(
    (char)('0' + d0),
    (char)('0' + d1),
    (char)('0' + d2),
    (char)('0' + d3),
    (char)('0' + d4),
    (char)('0' + d5)
  );
}
