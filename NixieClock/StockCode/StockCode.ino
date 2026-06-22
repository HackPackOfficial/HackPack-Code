/*
Dear Hack Pack User,

This is a Box 15, the Nixie Clock. 
*/
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>
#include <OneButton.h>
#include <Adafruit_NeoPixel.h>

#include "config.h"           // User editable variables
#include "ButtonHandler.h"
#include "PanelColorController.h"
#include "UnderGlow.h"
#include "TimeKeeper.h"
#include "stateEnums.h"
#include "index.html.h"


// For ESP32-S3 SuperMini]
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

//For Hack Pack ESP32-C3 Mini 

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
UnderGlow glow(GLOW_PIN, 13);

OneButton b_top(BTN_TOP, true);
OneButton dp_u(BTN_UP, true);
OneButton dp_d(BTN_DOWN, true);
OneButton dp_l(BTN_LEFT, true);
OneButton dp_r(BTN_RIGHT, true);
OneButton dp_c(BTN_CENTER, true);

// Web server on port 80
AsyncWebServer server(80);
PanelColorController display(LED_PIN, 42);
TimeKeeper nixie(REC, PLAY);    // pins for clock UI to interact w audio

DisplayState display_state = TIME;
UIState ui_state = NONE;

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


// Uses unique chip MAC address to create a unique Wifi Network
// Multiple Clocks can coexist without their APs all fighting over the same SSID
static String makeSuffixFromChip() {
  // Use lower 24 bits of the unique eFuse MAC
  uint32_t low = (uint32_t)(ESP.getEfuseMac() & 0xFFFFFFULL);
  char buf[7];                 // 6 hex chars + NUL
  sprintf(buf, "%06X", low);   // zero-padded uppercase hex
  return String(buf);
}

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> LOOP & SETUP <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
void setup() {
  //setCpuFrequencyMhz(240);
  Serial.begin(115200);

  DefaultHeaders::Instance().addHeader("Connection", "close");
  DefaultHeaders::Instance().addHeader("Cache-Control", "no-store, no-cache, must-revalidate");

  setupButtons();
  display.begin();

  glow.begin(glowBrightness);

  pinMode(REC, OUTPUT);
  pinMode(PLAY, OUTPUT);

  // ----------- WIFI --------------
  const String prefix   = ssid;
  const String suffix   = makeSuffixFromChip();           // e.g. "A1B2C3"
  const String unique_ssid     = prefix + " - " + suffix; // e.g. "NIXIE-A1B2C3"
  const String hostname = unique_ssid;                    // keep them consistent

  WiFi.softAP(hostname.c_str(), password);  // Change SSID and password in config.h
  WiFi.setTxPower(WIFI_POWER_8_5dBm);
  
  Serial.println("SoftAP started.");
  Serial.print("AP IP address: ");
  Serial.println(WiFi.softAPIP());

  server.onNotFound([](AsyncWebServerRequest *req){
    AsyncResponseStream* res = req->beginResponseStream("text/plain");
    res->addHeader("Connection", "close");
    res->print("not found");
    req->send(res);
  });

  // Host webpage with auto timesync
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/html", htmlPageTime);
  });

  // Once connection is established. Sync all time data.
  server.on("/time", HTTP_GET, [](AsyncWebServerRequest *request){
    if (request->hasParam("hour") && request->hasParam("min") && request->hasParam("sec") &&
        request->hasParam("ms") && request->hasParam("day") && request->hasParam("month") && request->hasParam("year")) {

      nixie.getTime().s_hour = request->getParam("hour")->value().toInt();
      nixie.getTime().s_minute = request->getParam("min")->value().toInt();
      nixie.getTime().s_sec = request->getParam("sec")->value().toInt();
      nixie.getTime().s_day = request->getParam("day")->value().toInt();
      nixie.getTime().s_month = request->getParam("month")->value().toInt();
      nixie.getTime().s_year = request->getParam("year")->value().toInt();
      nixie.getTime().s_m_sec = request->getParam("ms")->value().toInt();
      
      nixie.getClock().startTime = millis();

      nixie.setClockAbsolute();
      nixie.getClock().timeSet = true;

      Serial.println("Time set from browser:");
      Serial.printf("%04d-%02d-%02d %02d:%02d:%02d\n", nixie.getTime().s_year, nixie.getTime().s_month, nixie.getTime().s_day, nixie.getTime().s_hour, nixie.getTime().s_minute, nixie.getTime().s_sec);
    }
    request->send(200, "text/plain", "Time Set");
  });

  // Webpage for clock control UI
  server.on("/ui", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/html", htmlPageUI);
  });

  // ------------------------------ Quick Settings Handlers ---------------------------------
  server.on("/toggle24hr", HTTP_GET, [](AsyncWebServerRequest *request){
    Serial.println("Toggled 24hr mode");
    nixie.getClock()._24HrMode = !nixie.getClock()._24HrMode;
    Serial.println(nixie.getClock()._24HrMode);
    request->send(200, "text/plain", "Toggled 24hr mode");
  });
  // Toggles 01:00 vs 1:00
  server.on("/toggleLeadingZero", HTTP_GET, [](AsyncWebServerRequest *request){
    Serial.println("Toggled Leading Zero");
    nixie.getClock().showLeadZero = !nixie.getClock().showLeadZero;
    Serial.println(nixie.getClock().showLeadZero);
    request->send(200, "text/plain", "Toggled Leading Zero");
  });
  // Updates colorPos in PanelColorController
  server.on("/setColorPos", HTTP_GET, [](AsyncWebServerRequest *request){
    if (request->hasParam("val")) {
      int val = request->getParam("val")->value().toInt();
      val = constrain(val, 0, 255);
      display.getSettings().colorPos = val;
      Serial.print("Color Position set to: ");
      Serial.println(val);
      request->send(200, "text/plain", "Color Position updated");
    } else {
      request->send(400, "text/plain", "Missing 'val' parameter");
    }
  });
  // Pushes chosen color to display on the web UI
  server.on("/getColor", HTTP_GET, [](AsyncWebServerRequest *request){
    uint32_t c = display.Wheel(display.getSettings().colorPos);
    uint8_t r=(c>>16)&0xFF, g=(c>>8)&0xFF, b=c&0xFF;
    String json = "{\"r\":"+String(r)+",\"g\":"+String(g)+",\"b\":"+String(b)+"}";
    auto *res = request->beginResponse(200, "application/json", json);
    res->addHeader("Connection","close");
    res->addHeader("Cache-Control","no-store");
    request->send(res);
  });
  // Updates the chosen aninmation pattern
  server.on("/setColorState", HTTP_GET, [](AsyncWebServerRequest *request){
    if (request->hasParam("mode")) {
      String modeStr = request->getParam("mode")->value();
      if (modeStr == "RAINBOW") col_state = RAINBOW;
      else if (modeStr == "SOLID") col_state = SOLID;
      else if (modeStr == "GRADIENT") col_state = GRADIENT;
      else if (modeStr == "FLOW") col_state = FLOW;
      else if (modeStr == "WIPE") col_state = WIPE;
      else if (modeStr == "PULSE") col_state = PULSE;
      else if (modeStr == "BOUNCE") col_state = BOUNCE;
    }
    // Flag for initalization of new mode.
    display.getColors().modeChanged = true;

    request->send(200, "text/plain", "Color mode set");
  });
  // ---------------- PANEL BRIGHTNESS ----------------
  server.on("/setBrightness", HTTP_GET, [](AsyncWebServerRequest *request){
    if (request->hasParam("val")) {
      int pct = request->getParam("val")->value().toInt();
      pct = constrain(pct, 0, 100);
      int b = map(pct, 0, 100, 0, 255);

      display.getSettings().brightness = b;
      display.setBrightness();

      Serial.printf("Brightness set to %d%% (%d)\n", pct, b);
      request->send(200, "text/plain", "Brightness updated");
    } else {
      request->send(400, "text/plain", "Missing 'val' parameter");
    }
  });

  server.on("/getBrightness", HTTP_GET, [](AsyncWebServerRequest *request){
    int b = display.getSettings().brightness;
    b = constrain(b, 0, 255);
    int pct = map(b, 0, 255, 0, 100);

    String json = "{\"pct\":" + String(pct) + "}";
    auto *res = request->beginResponse(200, "application/json", json);
    res->addHeader("Connection","close");
    res->addHeader("Cache-Control","no-store");
    request->send(res);
  });

  // ---------------- ALARM SET ----------------
  server.on("/setAlarm", HTTP_GET, [](AsyncWebServerRequest *request){
    if (!request->hasParam("hour") ||
        !request->hasParam("min")  ||
        !request->hasParam("ampm")) {
      request->send(400, "text/plain", "Missing parameter(s)");
      return;
    }

    int hour  = request->getParam("hour")->value().toInt();   // 1–12 from UI
    int minute= request->getParam("min")->value().toInt();    // 0–59 from UI
    String ap = request->getParam("ampm")->value();           // "AM"/"PM"

    // Clamp to sane ranges
    hour   = constrain(hour,   1, 12);
    minute = constrain(minute, 0, 59);

    ap.toUpperCase();
    bool isPm = (ap == "PM");

    // Convert to 24-hr for storage (0–23)
    int h24 = hour % 12;
    if (isPm) h24 += 12;

    auto &clk = nixie.getClock();

    // Write digit array that the button UI uses
    clk.alarmSet[0] = h24 / 10;          // hour tens
    clk.alarmSet[1] = h24 % 10;          // hour ones
    clk.alarmSet[2] = minute / 10;       // minute tens
    clk.alarmSet[3] = minute % 10;       // minute ones
    clk.alarmSet[4] = 0;                 // seconds tens (fixed 00)
    clk.alarmSet[5] = 0;                 // seconds ones

    // Mark alarm as "set" if you want
    alrm       = SET;
    clck_state = RUN_ALARM;

    Serial.printf("Web alarm set: %02d:%02d %s (24h=%02d:%02d)\n",
                  hour, minute, ap.c_str(), h24, minute);

    request->send(200, "text/plain", "Alarm updated");
  });


  // ---------------- TIMER SET ----------------
  server.on("/setTimer", HTTP_GET, [](AsyncWebServerRequest *request){
    if (!request->hasParam("h") ||
        !request->hasParam("m") ||
        !request->hasParam("s")) {
      request->send(400, "text/plain", "Missing parameter(s)");
      return;
    }

    int h = request->getParam("h")->value().toInt();   // 0–99
    int m = request->getParam("m")->value().toInt();   // 0–59
    int s = request->getParam("s")->value().toInt();   // 0–59

    // Clamp
    h = constrain(h, 0, 99);
    m = constrain(m, 0, 59);
    s = constrain(s, 0, 59);

    auto &clk = nixie.getClock();

    // Write digit array (HH:MM:SS)
    clk.timerSet[0] = (h / 10) % 10;    // hours tens
    clk.timerSet[1] = h % 10;           // hours ones
    clk.timerSet[2] = m / 10;           // minutes tens
    clk.timerSet[3] = m % 10;           // minutes ones
    clk.timerSet[4] = s / 10;           // seconds tens
    clk.timerSet[5] = s % 10;           // seconds ones

    // Start timer like your center-button code does
    timr       = ACTIVE;
    clck_state = RUN_TIMER;

    clk.ringTime = millis()
                + clk.timerSet[0] * 1000UL * 60 * 60 * 10   // ten hours
                + clk.timerSet[1] * 1000UL * 60 * 60        // hours
                + clk.timerSet[2] * 1000UL * 60 * 10        // ten minutes
                + clk.timerSet[3] * 1000UL * 60             // minutes
                + clk.timerSet[4] * 1000UL * 10             // ten seconds
                + clk.timerSet[5] * 1000UL;                 // seconds

    Serial.printf("Web timer set: %02d:%02d:%02d\n", h, m, s);

    request->send(200, "text/plain", "Timer set");
  });


  // ---------------- CLEAR ALARM ----------------
  server.on("/clearAlarm", HTTP_GET, [](AsyncWebServerRequest *request){
    // Clear alarm state + digits
    alrm       = UNSET;
    clck_state = RUN_CLOCK;
    nixie.getClock().ringTime = 0;

    Serial.println("Alarm cleared via web");
    request->send(200, "text/plain", "Alarm cleared");
  });

  // ---------------- STOP TIMER ----------------
  server.on("/stopTimer", HTTP_GET, [](AsyncWebServerRequest *request){
    timr       = IDLE;
    clck_state = RUN_CLOCK;
    nixie.getClock().ringTime = 0;

    Serial.println("Timer stopped via web");
    request->send(200, "text/plain", "Timer stopped");
  });

  // ---------------- UnderGlow Color Position (0..255) ----------------
  server.on("/setUGColorPos", HTTP_GET, [](AsyncWebServerRequest *request){
    if (request->hasParam("val")) {
      int val = request->getParam("val")->value().toInt();
      val = constrain(val, 0, 255);
      glow.setColorPos(static_cast<uint8_t>(val));
      Serial.print("UnderGlow ColorPos set to: ");
      Serial.println(val);
      request->send(200, "text/plain", "UnderGlow ColorPos updated");
    } else {
      request->send(400, "text/plain", "Missing 'val' parameter");
    }
  });

  // ---------------- UnderGlow Color----------------
  server.on("/getUGColor", HTTP_GET, [](AsyncWebServerRequest *request){
    uint8_t r, g, b;
    glow.getCurrentColor(r, g, b);

    String json = "{\"r\":"+String(r)+",\"g\":"+String(g)+",\"b\":"+String(b)+"}";
    auto *res = request->beginResponse(200, "application/json", json);
    res->addHeader("Connection","close");
    res->addHeader("Cache-Control","no-store");
    request->send(res);
  });

  // UnderGlow brightness: 0–100%
  server.on("/setUGBrightness", HTTP_GET, [](AsyncWebServerRequest *request){
    if (request->hasParam("val")) {
      int pct = request->getParam("val")->value().toInt();
      pct = constrain(pct, 0, 100);
      glow.setBrightnessPct(pct);   // see UnderGlow class changes below
      Serial.print("UnderGlow brightness set to: ");
      Serial.println(pct);
      request->send(200, "text/plain", "UnderGlow brightness updated");
    } else {
      request->send(400, "text/plain", "Missing 'val' parameter");
    }
  });

  server.on("/getUGBrightness", HTTP_GET, [](AsyncWebServerRequest *request){
    uint8_t pct = glow.getBrightnessPct();
    String json = "{\"pct\":" + String(pct) + "}";
    auto *res = request->beginResponse(200, "application/json", json);
    res->addHeader("Connection","close");
    res->addHeader("Cache-Control","no-store");
    request->send(res);
  });

  server.begin();
}

void loop() {
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
    //runFace();                // Uncomment this for every character face animation
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

