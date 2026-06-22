#pragma once

#include <ESPAsyncWebServer.h>

class PanelColorController;
class TimeKeeper;
class UnderGlow;

class WebServerManager {
public:
  WebServerManager(AsyncWebServer& server,
                   PanelColorController& display,
                   TimeKeeper& nixie,
                   UnderGlow& glow);

  // Call once from setup()
  void begin();

  // Call each loop() for periodic NTP resync, etc.
  void loop();

private:
  AsyncWebServer&        _server;
  PanelColorController&  _display;
  TimeKeeper&            _nixie;
  UnderGlow&             _glow;

  // Time / NTP
  int                    _lastSyncDay = -1;
  String                 _tzString    = "PST8PDT,M3.2.0/2,M11.1.0/2";
  unsigned long          _lastNtpSyncMs = 0;
  uint8_t                _ntpFailCount = 0;

  // WiFi reconnect health check
  unsigned long          _lastWiFiCheckMs = 0;
  unsigned long          _lastReconnectAttemptMs = 0;
  bool                   _mdnsStarted = false;

  static const unsigned long NTP_SYNC_INTERVAL_MS;
  static const unsigned long WIFI_CHECK_INTERVAL_MS;
  static const unsigned long WIFI_RECONNECT_INTERVAL_MS;

  // Internal helpers
  void setupWiFi();
  void setupTime();
  void setupRoutes();
  void initialNtpSync();
  void maintainWiFi();
  bool wifiReady();
  void startMDNS();
  bool syncNtpWithReconnectOnFail(const char* reason, unsigned long timeoutMs = 10000);
};
