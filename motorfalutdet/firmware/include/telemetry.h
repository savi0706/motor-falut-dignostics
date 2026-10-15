/*
 * ============================================================
 * telemetry.h — Wi-Fi Dashboard (Lightweight HTTP Server)
 * Adaptive Multi-Sensor TinyML Motor Fault Diagnosis
 *
 * Hosts a simple JSON REST endpoint + minimal HTML dashboard.
 * No external IoT platform — no credentials beyond WiFi SSID/pass.
 * ============================================================
 */

#pragma once
#ifndef TELEMETRY_H
#define TELEMETRY_H

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ArduinoJson.h>
#include "config.h"
#include "logger.h"

// ==================== SHARED TELEMETRY DATA ====================
struct TelemetryPayload {
    uint32_t ts_ms;
    float vib_rms;
    float cur_rms_mA;
    float rpm;
    float temp_motor;
    float temp_bearing;
    int   health_pct;
    const char* fault;
    float confidence;
    int   as_state;
    float fft_peak_freq;
    float fft_peak_mag;
    bool  safety_trip;
};

static TelemetryPayload _tele_data = {};
static WebServer        _web_server(TELEMETRY_PORT);
static bool             _wifi_connected = false;

// ==================== HTML DASHBOARD ====================
static const char _HTML_PAGE[] PROGMEM = R"rawhtml(
<!DOCTYPE html><html><head>
<meta charset='utf-8'>
<meta http-equiv='refresh' content='2'>
<title>Motor Fault Monitor</title>
<style>
  body{font-family:monospace;background:#111;color:#0f0;margin:20px}
  h2{color:#0af}
  table{border-collapse:collapse;width:100%}
  td,th{border:1px solid #333;padding:6px 12px}
  th{background:#222;color:#0af}
  .fault{color:#f44;font-weight:bold}
  .ok{color:#0f0}
  .warn{color:#fa0}
</style>
</head><body>
<h2>&#x26A1; Motor Fault Diagnosis — TinyML</h2>
<table id='t'>
<tr><th>Parameter</th><th>Value</th></tr>
</table>
<p style='color:#555;font-size:0.8em'>Auto-refreshes every 2 s | GET /data for JSON</p>
<script>
fetch('/data').then(r=>r.json()).then(d=>{
  var t=document.getElementById('t');
  t.innerHTML='<tr><th>Parameter</th><th>Value</th></tr>';
  for(var k in d){
    var cls=(k==='fault'&&d[k]!=='Healthy')?'fault':
            (k==='health_pct'&&d[k]<50)?'warn':'ok';
    t.innerHTML+='<tr><td>'+k+'</td><td class="'+cls+'">'+d[k]+'</td></tr>';
  }
});
</script>
</body></html>
)rawhtml";

// ==================== HANDLERS ====================
static void handle_root() {
    _web_server.send_P(200, "text/html", _HTML_PAGE);
}

static void handle_data() {
    JsonDocument doc;
    doc["ts_ms"]        = _tele_data.ts_ms;
    doc["vib_rms"]      = _tele_data.vib_rms;
    doc["cur_rms_A"]    = _tele_data.cur_rms_mA / 1000.0f;
    doc["rpm"]          = _tele_data.rpm;
    doc["temp_motor_C"] = _tele_data.temp_motor;
    doc["temp_brg_C"]   = _tele_data.temp_bearing;
    doc["health_pct"]   = _tele_data.health_pct;
    doc["fault"]        = _tele_data.fault ? _tele_data.fault : "Unknown";
    doc["confidence"]   = _tele_data.confidence;
    doc["as_state"]     = _tele_data.as_state;
    doc["fft_freq_hz"]  = _tele_data.fft_peak_freq;
    doc["safety_trip"]  = _tele_data.safety_trip;

    String output;
    serializeJson(doc, output);
    _web_server.send(200, "application/json", output);
}

// ==================== PUBLIC API ====================
inline bool telemetry_init() {
#if WIFI_ENABLED == 0
    LOG_INFO("WiFi disabled (no secrets.h)");
    return false;
#endif
    if (strlen(WIFI_SSID) == 0) {
        LOG_WARN("WiFi SSID empty — telemetry disabled");
        return false;
    }

    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    LOG_INFO("WiFi connecting to: " + String(WIFI_SSID));

    uint32_t t0 = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - t0 < 10000) {
        delay(200);
    }

    if (WiFi.status() != WL_CONNECTED) {
        LOG_WARN("WiFi connection failed — telemetry disabled");
        return false;
    }

    _wifi_connected = true;
    LOG_INFO("WiFi OK. IP: " + WiFi.localIP().toString());

    _web_server.on("/",     handle_root);
    _web_server.on("/data", handle_data);
    _web_server.begin();
    LOG_INFO("Web server started on port " + String(TELEMETRY_PORT));
    return true;
}

inline void telemetry_push(const TelemetryPayload& payload) {
    _tele_data = payload;
}

inline void telemetry_handle_client() {
    if (_wifi_connected) _web_server.handleClient();
}

inline bool telemetry_connected() { return _wifi_connected; }

#endif // TELEMETRY_H
