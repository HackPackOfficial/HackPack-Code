#include <WiFi.h>
#include <ESPmDNS.h>
#include <time.h>
#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>

#include "config.h"
#include "PanelColorController.h"
#include "UnderGlow.h"
#include "TimeKeeper.h"
#include "stateEnums.h"
#include "index.html.h"
#include "WebServerManager.h"


// NTP servers
static const char* ntpServer1 = "pool.ntp.org";
static const char* ntpServer2 = "time.google.com";
static const char* ntpServer3 = "time.nist.gov";

// e.g. once per 6 hours
const unsigned long WebServerManager::NTP_SYNC_INTERVAL_MS =
    6UL * 60UL * 60UL * 1000UL;

// How often to check that STA WiFi is still healthy
const unsigned long WebServerManager::WIFI_CHECK_INTERVAL_MS =
    10UL * 1000UL;

// Minimum time between forced reconnect attempts
const unsigned long WebServerManager::WIFI_RECONNECT_INTERVAL_MS =
    30UL * 1000UL;

// These states are defined in TimeKeeper.cpp
extern TimerState  timr;
extern AlarmState  alrm;
extern ClockState  clck_state;

WebServerManager::WebServerManager(AsyncWebServer& server,
                                   PanelColorController& display,
                                   TimeKeeper& nixie,
                                   UnderGlow& glow)
  : _server(server),
    _display(display),
    _nixie(nixie),
    _glow(glow)
{
}

void WebServerManager::begin() {
  setupWiFi();
  setupTime();
  setupRoutes();
  initialNtpSync();
  _server.begin();
}

// ----------------- WIFI / NTP -----------------

void WebServerManager::setupWiFi() {
  WiFi.mode(WIFI_STA);

  // These help long-running ESP32 WiFi projects stay reachable.
  // persistent(false) avoids unnecessary flash writes.
  // setSleep(false) avoids power-save latency / stale connections on some routers.
  WiFi.persistent(false);
  WiFi.setSleep(false);
  WiFi.setAutoReconnect(true);

  Serial.print("Connecting to WiFi: ");
  Serial.println(your_ssid);

  WiFi.begin(your_ssid, your_password);

  int retry = 0;
  while (WiFi.status() != WL_CONNECTED && retry < 30) { // ~15s timeout
    delay(500);
    Serial.print(".");
    retry++;
  }
  Serial.println();

  if (wifiReady()) {
    Serial.print("Connected! IP address: ");
    Serial.println(WiFi.localIP());
    startMDNS();
  } else {
    Serial.println("Failed to connect to STA WiFi. Will keep retrying in loop().");
  }
}

bool WebServerManager::wifiReady() {
  return WiFi.status() == WL_CONNECTED && WiFi.localIP() != IPAddress(0, 0, 0, 0);
}

void WebServerManager::startMDNS() {
  if (!wifiReady()) return;

  String mdnsName = "hp-nixie-clock";

  // Restarting mDNS after a reconnect keeps hp-nixie-clock.local alive.
  if (_mdnsStarted) {
    MDNS.end();
    _mdnsStarted = false;
  }

  if (MDNS.begin(mdnsName.c_str())) {
    MDNS.addService("http", "tcp", 80);
    _mdnsStarted = true;
    Serial.println("mDNS responder started");
    Serial.print("Open: http://");
    Serial.print(mdnsName);
    Serial.println(".local/ui");
  } else {
    Serial.println("Error setting up mDNS responder!");
  }
}

void WebServerManager::maintainWiFi() {
  unsigned long now = millis();
  if (now - _lastWiFiCheckMs < WIFI_CHECK_INTERVAL_MS) return;
  _lastWiFiCheckMs = now;

  if (wifiReady()) return;

  if (now - _lastReconnectAttemptMs < WIFI_RECONNECT_INTERVAL_MS) return;
  _lastReconnectAttemptMs = now;

  Serial.println("WiFi is down/stale. Forcing STA reconnect...");

  if (_mdnsStarted) {
    MDNS.end();
    _mdnsStarted = false;
  }

  WiFi.disconnect(false, false);   // disconnect radio, keep saved credentials
  delay(100);
  WiFi.begin(your_ssid, your_password);

  // Short blocking wait so the web app comes back quickly, but not forever.
  unsigned long start = millis();
  while (!wifiReady() && millis() - start < 5000) {
    delay(100);
  }

  if (wifiReady()) {
    Serial.print("WiFi reconnected. IP address: ");
    Serial.println(WiFi.localIP());
    startMDNS();

    // Re-arm SNTP after reconnect. This is cheap and helps after router changes.
    setupTime();
    syncNtpWithReconnectOnFail("post-reconnect NTP sync", 10000);
  } else {
    Serial.println("Reconnect attempt failed; will retry.");
  }
}

bool WebServerManager::syncNtpWithReconnectOnFail(const char* reason, unsigned long timeoutMs) {
  if (!wifiReady()) return false;

  Serial.print("Attempting ");
  Serial.print(reason);
  Serial.println("...");

  if (_nixie.syncFromNTP(timeoutMs)) {
    _lastNtpSyncMs = millis();
    _ntpFailCount = 0;
    Serial.print(reason);
    Serial.println(" complete.");
    return true;
  }

  _ntpFailCount++;
  Serial.print(reason);
  Serial.println(" failed.");

  // If WiFi says connected but NTP repeatedly fails, the connection may be stale.
  // Force a reconnect so the next loop() can recover.
  if (_ntpFailCount >= 2) {
    Serial.println("Repeated NTP failures; forcing WiFi reconnect next pass.");
    WiFi.disconnect(false, false);
    _lastReconnectAttemptMs = 0;
  }

  return false;
}

void WebServerManager::setupTime() {
  // Use TZ string + NTP servers
  configTzTime(
    _tzString.c_str(),
    ntpServer1,
    ntpServer2,
    ntpServer3
  );
}

void WebServerManager::initialNtpSync() {
  syncNtpWithReconnectOnFail("initial NTP sync", 10000);
}

// Called every loop() to do WiFi maintenance and periodic NTP resync
void WebServerManager::loop() {
  maintainWiFi();
  if (!wifiReady()) return;

  unsigned long now = millis();

  // Actual 6-hour NTP resync. The old code defined the interval but did not use it.
  if (_lastNtpSyncMs == 0 || now - _lastNtpSyncMs >= NTP_SYNC_INTERVAL_MS) {
    syncNtpWithReconnectOnFail("scheduled NTP resync", 10000);
  }

  // Keep the existing once-per-day midnight resync too.
  Time &t = _nixie.getTime();
  if (t.c_hour == 0 && t.c_minute == 0 && t.c_sec < 5) {
    if (_lastSyncDay != t.c_day) {
      if (syncNtpWithReconnectOnFail("midnight NTP resync", 10000)) {
        _lastSyncDay = t.c_day;
      }
    }
  }
}


// ----------------- ROUTES / WEB UI -----------------

void WebServerManager::setupRoutes() {
  // Redirect unknown paths to /ui
  _server.onNotFound([this](AsyncWebServerRequest *req){
    req->redirect("/ui");
  });

  // Root = main UI
  _server.on("/", HTTP_GET, [this](AsyncWebServerRequest *request){
    request->send_P(200, "text/html", htmlPageUI);
  });

  _server.on("/ui", HTTP_GET, [this](AsyncWebServerRequest *request){
    request->send_P(200, "text/html", htmlPageUI);
  });

    // ----- Get current color mode (for initial highlight) -----
  _server.on("/getColorState", HTTP_GET, [this](AsyncWebServerRequest *request){
    String modeStr = "RAINBOW";  // fallback

    switch (col_state) {
      case RAINBOW:  modeStr = "RAINBOW";  break;
      case SOLID:    modeStr = "SOLID";    break;
      case GRADIENT: modeStr = "GRADIENT"; break;
      case FLOW:     modeStr = "FLOW";     break;
      case WIPE:     modeStr = "WIPE";     break;
      case PULSE:    modeStr = "PULSE";    break;
      case BOUNCE:   modeStr = "BOUNCE";   break;
      default:       modeStr = "RAINBOW";  break;
    }

    String json = "{\"mode\":\"" + modeStr + "\"}";
    auto *res = request->beginResponse(200, "application/json", json);
    res->addHeader("Connection","close");
    res->addHeader("Cache-Control","no-store");
    request->send(res);
  });

  // ----- 24hr toggle -----
  _server.on("/toggle24hr", HTTP_GET, [this](AsyncWebServerRequest *request){
    Serial.println("Toggled 24hr mode");
    _nixie.getClock()._24HrMode = !_nixie.getClock()._24HrMode;
    Serial.println(_nixie.getClock()._24HrMode);
    request->send(200, "text/plain", "Toggled 24hr mode");
  });

  // ----- Leading zero toggle -----
  _server.on("/toggleLeadingZero", HTTP_GET, [this](AsyncWebServerRequest *request){
    Serial.println("Toggled Leading Zero");
    _nixie.getClock().showLeadZero = !_nixie.getClock().showLeadZero;
    Serial.println(_nixie.getClock().showLeadZero);
    request->send(200, "text/plain", "Toggled Leading Zero");
  });

  // ----- Panel color position -----
  _server.on("/setColorPos", HTTP_GET, [this](AsyncWebServerRequest *request){
    if (request->hasParam("val")) {
      int val = request->getParam("val")->value().toInt();
      val = constrain(val, 0, 255);
      _display.getSettings().colorPos = val;
      Serial.print("Color Position set to: ");
      Serial.println(val);
      request->send(200, "text/plain", "Color Position updated");
    } else {
      request->send(400, "text/plain", "Missing 'val' parameter");
    }
  });

  _server.on("/getColor", HTTP_GET, [this](AsyncWebServerRequest *request){
    uint32_t c = _display.Wheel(_display.getSettings().colorPos);
    uint8_t r=(c>>16)&0xFF, g=(c>>8)&0xFF, b=c&0xFF;
    String json = "{\"r\":"+String(r)+",\"g\":"+String(g)+",\"b\":"+String(b)+"}";
    auto *res = request->beginResponse(200, "application/json", json);
    res->addHeader("Connection","close");
    res->addHeader("Cache-Control","no-store");
    request->send(res);
  });

  // ----- Color mode -----
  _server.on("/setColorState", HTTP_GET, [this](AsyncWebServerRequest *request){
    if (request->hasParam("mode")) {
      String modeStr = request->getParam("mode")->value();
      if      (modeStr == "RAINBOW") col_state = RAINBOW;
      else if (modeStr == "SOLID")   col_state = SOLID;
      else if (modeStr == "GRADIENT")col_state = GRADIENT;
      else if (modeStr == "FLOW")    col_state = FLOW;
      else if (modeStr == "WIPE")    col_state = WIPE;
      else if (modeStr == "PULSE")   col_state = PULSE;
      else if (modeStr == "BOUNCE")  col_state = BOUNCE;
    }
    _display.getColors().modeChanged = true;
    request->send(200, "text/plain", "Color mode set");
  });

  // ----- Panel brightness -----
  _server.on("/setBrightness", HTTP_GET, [this](AsyncWebServerRequest *request){
    if (request->hasParam("val")) {
      int pct = request->getParam("val")->value().toInt();
      pct = constrain(pct, 0, 100);
      int b = map(pct, 0, 100, 0, 255);

      _display.getSettings().brightness = b;
      _display.setBrightness();

      Serial.printf("Brightness set to %d%% (%d)\n", pct, b);
      request->send(200, "text/plain", "Brightness updated");
    } else {
      request->send(400, "text/plain", "Missing 'val' parameter");
    }
  });

  _server.on("/getBrightness", HTTP_GET, [this](AsyncWebServerRequest *request){
    int b = _display.getSettings().brightness;
    b = constrain(b, 0, 255);
    int pct = map(b, 0, 255, 0, 100);

    String json = "{\"pct\":" + String(pct) + "}";
    auto *res = request->beginResponse(200, "application/json", json);
    res->addHeader("Connection","close");
    res->addHeader("Cache-Control","no-store");
    request->send(res);
  });

  // ----- Alarm set / clear -----
  _server.on("/setAlarm", HTTP_GET, [this](AsyncWebServerRequest *request){
    if (!request->hasParam("hour") ||
        !request->hasParam("min")  ||
        !request->hasParam("ampm")) {
      request->send(400, "text/plain", "Missing parameter(s)");
      return;
    }

    int hour  = request->getParam("hour")->value().toInt();
    int minute= request->getParam("min")->value().toInt();
    String ap = request->getParam("ampm")->value();

    hour   = constrain(hour,   1, 12);
    minute = constrain(minute, 0, 59);

    ap.toUpperCase();
    bool isPm = (ap == "PM");

    int h24 = hour % 12;
    if (isPm) h24 += 12;

    auto &clk = _nixie.getClock();
    clk.alarmSet[0] = h24 / 10;
    clk.alarmSet[1] = h24 % 10;
    clk.alarmSet[2] = minute / 10;
    clk.alarmSet[3] = minute % 10;
    clk.alarmSet[4] = 0;
    clk.alarmSet[5] = 0;

    alrm       = SET;
    clck_state = RUN_ALARM;

    Serial.printf("Web alarm set: %02d:%02d %s (24h=%02d:%02d)\n",
                  hour, minute, ap.c_str(), h24, minute);

    request->send(200, "text/plain", "Alarm updated");
  });

  _server.on("/clearAlarm", HTTP_GET, [this](AsyncWebServerRequest *request){
    alrm       = UNSET;
    clck_state = RUN_CLOCK;
    _nixie.getClock().ringTime = 0;

    Serial.println("Alarm cleared via web");
    request->send(200, "text/plain", "Alarm cleared");
  });

  // ----- Timer set / stop -----
  _server.on("/setTimer", HTTP_GET, [this](AsyncWebServerRequest *request){
    if (!request->hasParam("h") ||
        !request->hasParam("m") ||
        !request->hasParam("s")) {
      request->send(400, "text/plain", "Missing parameter(s)");
      return;
    }

    int h = request->getParam("h")->value().toInt();
    int m = request->getParam("m")->value().toInt();
    int s = request->getParam("s")->value().toInt();

    h = constrain(h, 0, 99);
    m = constrain(m, 0, 59);
    s = constrain(s, 0, 59);

    auto &clk = _nixie.getClock();

    clk.timerSet[0] = (h / 10) % 10;
    clk.timerSet[1] = h % 10;
    clk.timerSet[2] = m / 10;
    clk.timerSet[3] = m % 10;
    clk.timerSet[4] = s / 10;
    clk.timerSet[5] = s % 10;

    timr       = ACTIVE;
    clck_state = RUN_TIMER;

    clk.ringTime = millis()
                + clk.timerSet[0] * 1000UL * 60 * 60 * 10
                + clk.timerSet[1] * 1000UL * 60 * 60
                + clk.timerSet[2] * 1000UL * 60 * 10
                + clk.timerSet[3] * 1000UL * 60
                + clk.timerSet[4] * 1000UL * 10
                + clk.timerSet[5] * 1000UL;

    Serial.printf("Web timer set: %02d:%02d:%02d\n", h, m, s);
    request->send(200, "text/plain", "Timer set");
  });

  _server.on("/stopTimer", HTTP_GET, [this](AsyncWebServerRequest *request){
    timr       = IDLE;
    clck_state = RUN_CLOCK;
    _nixie.getClock().ringTime = 0;

    Serial.println("Timer stopped via web");
    request->send(200, "text/plain", "Timer stopped");
  });

  // ----- UnderGlow color position / brightness -----
  _server.on("/setUGColorPos", HTTP_GET, [this](AsyncWebServerRequest *request){
    if (request->hasParam("val")) {
      int val = request->getParam("val")->value().toInt();
      val = constrain(val, 0, 255);
      _glow.setColorPos(static_cast<uint8_t>(val));
      Serial.print("UnderGlow ColorPos set to: ");
      Serial.println(val);
      request->send(200, "text/plain", "UnderGlow ColorPos updated");
    } else {
      request->send(400, "text/plain", "Missing 'val' parameter");
    }
  });

  _server.on("/getUGColor", HTTP_GET, [this](AsyncWebServerRequest *request){
    uint8_t r, g, b;
    _glow.getCurrentColor(r, g, b);

    String json = "{\"r\":"+String(r)+",\"g\":"+String(g)+",\"b\":"+String(b)+"}";
    auto *res = request->beginResponse(200, "application/json", json);
    res->addHeader("Connection","close");
    res->addHeader("Cache-Control","no-store");
    request->send(res);
  });

  _server.on("/setUGBrightness", HTTP_GET, [this](AsyncWebServerRequest *request){
    if (request->hasParam("val")) {
      int pct = request->getParam("val")->value().toInt();
      pct = constrain(pct, 0, 100);
      _glow.setBrightnessPct(pct);
      Serial.print("UnderGlow brightness set to: ");
      Serial.println(pct);
      request->send(200, "text/plain", "UnderGlow brightness updated");
    } else {
      request->send(400, "text/plain", "Missing 'val' parameter");
    }
  });

  _server.on("/getUGBrightness", HTTP_GET, [this](AsyncWebServerRequest *request){
    uint8_t pct = _glow.getBrightnessPct();
    String json = "{\"pct\":" + String(pct) + "}";
    auto *res = request->beginResponse(200, "application/json", json);
    res->addHeader("Connection","close");
    res->addHeader("Cache-Control","no-store");
    request->send(res);
  });

  // ----- Timezone selection (/setTZ) -----
  _server.on("/setTZ", HTTP_GET, [this](AsyncWebServerRequest *request){
    if (!request->hasParam("zone")) {
      request->send(400, "text/plain", "Missing 'zone' parameter");
      return;
    }

    String zone = request->getParam("zone")->value();
    zone.trim();

    // Map dropdown choices to POSIX TZ rule strings
    if      (zone == "US_PACIFIC")   _tzString = "PST8PDT,M3.2.0/2,M11.1.0/2";
    else if (zone == "US_MOUNTAIN")  _tzString = "MST7MDT,M3.2.0/2,M11.1.0/2";
    else if (zone == "US_CENTRAL")   _tzString = "CST6CDT,M3.2.0/2,M11.1.0/2";
    else if (zone == "US_EASTERN")   _tzString = "EST5EDT,M3.2.0/2,M11.1.0/2";
    else if (zone == "US_ALASKA")    _tzString = "AKST9AKDT,M3.2.0/2,M11.1.0/2";
    else if (zone == "US_HAWAII")    _tzString = "HST10";   // no DST

    // Americas
    else if (zone == "MEXICO_CITY")  _tzString = "CST6";    // no DST (simplified)
    else if (zone == "BOGOTA")       _tzString = "COT5";
    else if (zone == "LIMA")         _tzString = "PET5";
    else if (zone == "SAO_PAULO")    _tzString = "BRT3";
    else if (zone == "BUENOS_AIRES") _tzString = "ART3";
    else if (zone == "SANTIAGO")     _tzString = "CLT4";

    // Europe
    else if (zone == "UK")           _tzString = "GMT0BST,M3.5.0/1,M10.5.0/2";
    else if (zone == "EU_WEST")      _tzString = "WET0WEST,M3.5.0/1,M10.5.0/2";
    else if (zone == "EU_CENTRAL")   _tzString = "CET-1CEST,M3.5.0/2,M10.5.0/3";
    else if (zone == "EU_EAST")      _tzString = "EET-2EEST,M3.5.0/3,M10.5.0/4";
    else if (zone == "MOSCOW")       _tzString = "MSK-3";

    // Africa
    else if (zone == "SOUTH_AFRICA") _tzString = "SAST-2";
    else if (zone == "EGYPT")        _tzString = "EET-2";
    else if (zone == "EAST_AFRICA")  _tzString = "EAT-3";
    else if (zone == "WEST_AFRICA")  _tzString = "WAT-1";

    // Middle East
    else if (zone == "TURKEY")       _tzString = "TRT-3";
    else if (zone == "ISRAEL")       _tzString = "IST-2IDT,M3.5.0/2,M10.5.0/2";
    else if (zone == "SAUDI_ARABIA") _tzString = "AST-3";
    else if (zone == "UAE")          _tzString = "GST-4";

    // Asia
    else if (zone == "INDIA")        _tzString = "IST-5:30";
    else if (zone == "CHINA")        _tzString = "CST-8";
    else if (zone == "HONG_KONG")    _tzString = "HKT-8";
    else if (zone == "TAIWAN")       _tzString = "CST-8";
    else if (zone == "SINGAPORE")    _tzString = "SGT-8";
    else if (zone == "JAPAN")        _tzString = "JST-9";
    else if (zone == "KOREA")        _tzString = "KST-9";
    else if (zone == "BANGKOK")      _tzString = "ICT-7";
    else if (zone == "JAKARTA")      _tzString = "WIB-7";

    // Australia / NZ / Pacific
    else if (zone == "NZ")           _tzString = "NZST-12NZDT,M9.5.0/2,M4.1.0/3";
    else if (zone == "AUS_EAST")     _tzString = "AEST-10AEDT,M10.1.0/2,M4.1.0/3";
    else if (zone == "AUS_QLD")      _tzString = "AEST-10";
    else if (zone == "AUS_CENTRAL")  _tzString = "ACST-9:30ACDT,M10.1.0/2,M4.1.0/3";
    else if (zone == "AUS_NT")       _tzString = "ACST-9:30";
    else if (zone == "AUS_WEST")     _tzString = "AWST-8";
    else if (zone == "FIJI")         _tzString = "FJT-12";

    // Raw UTC
    else if (zone == "UTC")          _tzString = "UTC0";

    else {
      request->send(400, "text/plain", "Unknown timezone");
      return;
    }

    // Re-configure SNTP with updated TZ
    configTzTime(
      _tzString.c_str(),
      ntpServer1,
      ntpServer2,
      ntpServer3
    );

    Serial.print("Timezone updated to: ");
    Serial.print(zone);
    Serial.print(" (");
    Serial.print(_tzString);
    Serial.println(")");

    // Optional: immediately resync from NTP so the display jumps to correct local time
    if (WiFi.status() == WL_CONNECTED) {
      if (_nixie.syncFromNTP(10000)) {
        Serial.println("Time re-synced after timezone change.");
      } else {
        Serial.println("NTP sync failed after timezone change.");
      }
    }

    request->send(200, "text/plain", "Timezone updated");
  });

}
