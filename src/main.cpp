#include <Arduino.h>
/*
  AgriNode - smart plant/crop monitor for the Freenove ESP32-WROVER (camera) Basic Starter Kit (FNK0061)

  Measures : soil moisture (DIY two-nail probe), air temperature + humidity (DHT module), light (photoresistor)
  Decides  : auto-irrigation (pump output = LED on GPIO2, swap for a relay later), frost/heat/dry alerts
  Shows    : web dashboard on your WiFi (live values, 6 h history, camera snapshot, settings)

  Arduino IDE setup (same as the Freenove tutorial):
    Board: "ESP32 Wrover Module"   PSRAM: "Enabled"   Partition: "Huge APP" (or default)
    Libraries: install "PubSubClient" (Nick O'Leary) via Library Manager. Everything else ships with the ESP32 core.
  Cloud path: ESP32 -> WiFi -> MQTT (this sketch) -> Telegraf -> InfluxDB -> Grafana  (configs in ../cloud)
  Local path: WiFi web server/API (dashboard, /pump, /set, /cal)  +  MQTT commands on agrinode/<NODE_ID>/cmd
  Remove the microSD card while this sketch runs (the SD slot shares GPIO2/14/15).
  Edit WIFI_SSID / WIFI_PASS below, upload, open Serial Monitor (115200) for the IP, or http://agrinode.local
*/

#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <Preferences.h>
#include <PubSubClient.h>
#include <DHT.h>
#include <time.h>
#include "esp_camera.h"

// ------------------------------------------------------------------ user config
const char *WIFI_SSID = "#######"; // Hidden on purpose
const char *WIFI_PASS = "#######"; // Hidden on purpose
const char *TZ_INFO   = "AEST-10AEDT,M10.1.0,M4.1.0/3";   // Sydney. Change for other places.

// MQTT broker = the machine running Mosquitto (your laptop / class server / cloud VM). Use its LAN or public IP.
const char    *MQTT_HOST = "broker.emqx.io";    // free public broker from the lecture (anyone can read it: keep NODE_ID unique)
const uint16_t MQTT_PORT = 1883;
const char    *MQTT_USER = "";                 // leave "" if the broker allows anonymous
const char    *MQTT_PASS = "";
const char    *NODE_ID   = "uts-roshanben-agri-4821";           // appears in topics: agrinode/<NODE_ID>/telemetry|status|cmd
#define MQTT_PUBLISH_MS 10000UL

// ------------------------------------------------------------------ pins (all free on the Freenove WROVER cam board)
#define PIN_SOIL_PWR 15   // powers the soil probe ONLY while measuring (stops nails corroding)
#define PIN_SOIL_ADC 32   // ADC1 - soil divider midpoint
#define PIN_LDR_PWR  12   // photoresistor "top" end, driven HIGH while reading it
#define PIN_DHT      13   // DHT temperature + humidity module, DATA pin
#define PIN_AIN      33   // ADC1 - photoresistor midpoint (photoresistor + 10k to GND)
#define DHT_TYPE     DHT11  // use DHT22 if your module is marked DHT22 / AM2302
#define PIN_PUMP     2    // LED (or relay IN) = irrigation output
#define PIN_BUZZER   14   // active OR passive buzzer

// ADC2 pins cannot be read while WiFi is on, and the camera uses the other ADC1 pins,
// so everything analog is squeezed onto GPIO32 + GPIO33 using the pin-as-supply trick above.

// ------------------------------------------------------------------ camera pins (Freenove ESP32-WROVER)
#define CAM_PWDN  -1
#define CAM_RESET -1
#define CAM_XCLK  21
#define CAM_SIOD  26
#define CAM_SIOC  27
#define CAM_Y9    35
#define CAM_Y8    34
#define CAM_Y7    39
#define CAM_Y6    36
#define CAM_Y5    19
#define CAM_Y4    18
#define CAM_Y3    5
#define CAM_Y2    4
#define CAM_VSYNC 25
#define CAM_HREF  23
#define CAM_PCLK  22

// ------------------------------------------------------------------ tuning
#define SENSE_INTERVAL_MS   2000UL
#define HISTORY_INTERVAL_S  60        // one history point per minute
#define HISTORY_LEN         360       // = 6 hours
#define DAY_LIGHT_INDEX     25.0f     // light index above this counts as "daylight"
#define FROST_C             2.0f
#define HEAT_C              38.0f
#define DRY_ALERT_PCT       15
#define MAX_PUMP_SEC        60

struct Settings {
  int  lowPct   = 35;     // water when soil% falls below this
  int  runSec   = 8;      // pump run time per watering
  int  coolMin  = 15;     // minimum minutes between waterings
  bool autoMode = true;
  int  dryMv    = 200;    // soil probe reading in dry air/soil  (calibrate!)
  int  wetMv    = 2000;   // soil probe reading in a glass of water (calibrate!)
} cfg;

Preferences prefs;
WebServer server(80);

// ------------------------------------------------------------------ state
float soilPct = NAN, tempC = NAN, lightIdx = NAN, airHum = NAN;
DHT dht(PIN_DHT, DHT_TYPE);
int   soilMv = 0;
bool  pumpOn = false, everWatered = false;
uint32_t pumpOffAt = 0, lastWaterAt = 0, lastBeepAt = 0, muteUntil = 0, lastSenseAt = 0, lastHistAt = 0;
bool  muted = false;
float lightSecToday = 0;
int   lastYday = -1;
String alertText = "";

// analytics (see computeTrend)
float dryRate = NAN;        // soil % lost per hour over the last ~30 min (positive = drying)
float minsToLow = NAN;      // predicted minutes until soil hits the irrigation threshold
uint32_t pumpRuns = 0, pumpSecTotal = 0;   // since boot

WiFiClient netClient;
PubSubClient mqtt(netClient);
String tTele, tStatus, tCmd;
uint32_t lastMqttTry = 0, lastMqttPub = 0;

float hSoil[HISTORY_LEN], hTemp[HISTORY_LEN], hLight[HISTORY_LEN];
int   hCount = 0, hHead = 0;   // ring buffer

// ------------------------------------------------------------------ helpers
static void readDht() {
  static uint8_t fails = 0;
  float t = dht.readTemperature(), h = dht.readHumidity();
  if (isnan(t) || isnan(h)) {                                  // tolerate a few bad reads before declaring a fault
    if (++fails >= 5) { tempC = NAN; airHum = NAN; }
    return;
  }
  fails = 0; tempC = t; airHum = h;
}

static float readLightIdx() {
  pinMode(PIN_LDR_PWR, OUTPUT);
  digitalWrite(PIN_LDR_PWR, HIGH);                             // photoresistor only powered while measuring
  delay(10);
  uint32_t sum = 0;
  for (int i = 0; i < 16; i++) { sum += analogReadMilliVolts(PIN_AIN); delayMicroseconds(500); }
  pinMode(PIN_LDR_PWR, INPUT);
  float mv = sum / 16.0f;
  return constrain(mv / 3300.0f * 100.0f, 0.0f, 100.0f);      // 0 = dark, 100 = very bright (relative index)
}

static int readSoilMv() {
  digitalWrite(PIN_SOIL_PWR, HIGH);
  delay(30);
  uint32_t sum = 0;
  for (int i = 0; i < 16; i++) { sum += analogReadMilliVolts(PIN_SOIL_ADC); delayMicroseconds(500); }
  digitalWrite(PIN_SOIL_PWR, LOW);
  return sum / 16;
}

static float soilPctFromMv(int mv) {
  if (cfg.wetMv <= cfg.dryMv + 50) return NAN;                // bad calibration
  return constrain((mv - cfg.dryMv) * 100.0f / (cfg.wetMv - cfg.dryMv), 0.0f, 100.0f);
}

static void beep(int n) {
  for (int i = 0; i < n; i++) {
    for (int j = 0; j < 300; j++) {                           // ~2 kHz for ~150 ms (works for active and passive buzzers)
      digitalWrite(PIN_BUZZER, HIGH); delayMicroseconds(250);
      digitalWrite(PIN_BUZZER, LOW);  delayMicroseconds(250);
    }
    delay(120);
  }
}

static void loadSettings() {
  prefs.begin("agri", true);
  cfg.lowPct   = prefs.getInt("low", cfg.lowPct);
  cfg.runSec   = prefs.getInt("run", cfg.runSec);
  cfg.coolMin  = prefs.getInt("cool", cfg.coolMin);
  cfg.autoMode = prefs.getBool("auto", cfg.autoMode);
  cfg.dryMv    = prefs.getInt("dry", cfg.dryMv);
  cfg.wetMv    = prefs.getInt("wet", cfg.wetMv);
  prefs.end();
}

static void saveSettings() {
  prefs.begin("agri", false);
  prefs.putInt("low", cfg.lowPct);
  prefs.putInt("run", cfg.runSec);
  prefs.putInt("cool", cfg.coolMin);
  prefs.putBool("auto", cfg.autoMode);
  prefs.putInt("dry", cfg.dryMv);
  prefs.putInt("wet", cfg.wetMv);
  prefs.end();
}

// ------------------------------------------------------------------ irrigation + alerts
static void setPump(bool on, int seconds = 0) {
  pumpOn = on;
  digitalWrite(PIN_PUMP, on ? HIGH : LOW);
  if (on) {
    seconds = constrain(seconds, 1, MAX_PUMP_SEC);
    pumpOffAt = millis() + seconds * 1000UL;
    lastWaterAt = millis();
    everWatered = true;
    pumpRuns++;
    pumpSecTotal += seconds;
    Serial.printf("[pump] ON for %d s\n", seconds);
  } else {
    Serial.println("[pump] OFF");
  }
}

static void irrigationTick() {
  uint32_t now = millis();
  if (pumpOn && (int32_t)(now - pumpOffAt) >= 0) setPump(false);
  if (!cfg.autoMode || pumpOn || isnan(soilPct)) return;
  bool cooled = !everWatered || (now - lastWaterAt) >= (uint32_t)cfg.coolMin * 60000UL;
  bool frost  = !isnan(tempC) && tempC < FROST_C;             // never irrigate into a frost
  if (soilPct < cfg.lowPct && cooled && !frost) setPump(true, cfg.runSec);
}

static void alertTick() {
  alertText = "";
  if (!isnan(tempC) && tempC < FROST_C)  alertText += "Frost risk. ";
  if (!isnan(tempC) && tempC > HEAT_C)   alertText += "Heat stress. ";
  if (!isnan(soilPct) && soilPct < DRY_ALERT_PCT) alertText += "Soil very dry. ";
  if (!isnan(airHum) && airHum > 90)     alertText += "High humidity (disease risk). ";
  if (isnan(tempC))                      alertText += "Temp/humidity sensor not detected. ";
  if (isnan(soilPct))                    alertText += "Soil probe not calibrated. ";

  uint32_t now = millis();
  if (muted && (int32_t)(now - muteUntil) >= 0) muted = false;
  if (alertText.length() && !muted && (now - lastBeepAt) > 30000UL) { beep(3); lastBeepAt = now; }
}

static void daylightTick(float dtSec) {
  struct tm t;
  if (!getLocalTime(&t, 0)) return;                           // NTP not synced yet
  if (t.tm_yday != lastYday) { lastYday = t.tm_yday; lightSecToday = 0; }
  if (!isnan(lightIdx) && lightIdx > DAY_LIGHT_INDEX) lightSecToday += dtSec;
}

// Least-squares slope of the last 30 one-minute soil points -> drying rate and time-to-threshold prediction.
static void computeTrend() {
  int n = hCount < 30 ? hCount : 30;
  double sx = 0, sy = 0, sxx = 0, sxy = 0; int m = 0;
  for (int i = 0; i < n; i++) {
    int idx = (hHead - n + i + HISTORY_LEN * 2) % HISTORY_LEN;
    float y = hSoil[idx];
    if (isnan(y)) continue;
    double x = i;
    sx += x; sy += y; sxx += x * x; sxy += x * y; m++;
  }
  double den = m * sxx - sx * sx;
  if (m < 10 || fabs(den) < 1e-6) { dryRate = NAN; minsToLow = NAN; return; }
  double slope = (m * sxy - sx * sy) / den;                  // % per minute
  dryRate = (float)(-slope * 60.0);
  if (slope < -0.01 && !isnan(soilPct) && soilPct > cfg.lowPct) minsToLow = (float)((soilPct - cfg.lowPct) / -slope);
  else minsToLow = NAN;
}

static void pushHistory() {
  hSoil[hHead] = soilPct; hTemp[hHead] = tempC; hLight[hHead] = lightIdx;
  hHead = (hHead + 1) % HISTORY_LEN;
  if (hCount < HISTORY_LEN) hCount++;
  computeTrend();
}

// ------------------------------------------------------------------ MQTT (global path: ESP32 -> broker -> Telegraf -> InfluxDB -> Grafana)
static void onMqtt(char *topic, byte *payload, unsigned int len) {
  String m; for (unsigned int i = 0; i < len && i < 64; i++) m += (char)payload[i];
  m.trim();
  int eq = m.indexOf('=');
  String k = eq < 0 ? m : m.substring(0, eq);
  int v = eq < 0 ? 0 : m.substring(eq + 1).toInt();
  Serial.printf("[mqtt] cmd %s\n", m.c_str());
  if      (k == "pump") { if (v) setPump(true, cfg.runSec); else setPump(false); }
  else if (k == "auto") cfg.autoMode = v != 0;
  else if (k == "low")  cfg.lowPct = constrain(v, 0, 100);
  else if (k == "run")  cfg.runSec = constrain(v, 1, MAX_PUMP_SEC);
  else if (k == "cool") cfg.coolMin = constrain(v, 1, 1440);
  else return;
  saveSettings();
}

static void mqttPublish() {
  char b[420]; int n = 0; bool first = true;
  n += snprintf(b + n, sizeof(b) - n, "{");
  auto addf = [&](const char *k, float v, int d) {
    if (isnan(v)) return;                                    // omit missing readings instead of sending nulls
    n += snprintf(b + n, sizeof(b) - n, "%s\"%s\":%.*f", first ? "" : ",", k, d, v);
    first = false;
  };
  addf("soil_pct", soilPct, 1);
  addf("soil_mv", soilMv, 0);
  addf("temp_c", tempC, 2);
  addf("humidity_pct", airHum, 1);
  addf("light_idx", lightIdx, 1);
  addf("daylight_h", lightSecToday / 3600.0f, 3);
  addf("dry_rate_pph", dryRate, 2);
  addf("mins_to_low", minsToLow, 0);
  addf("pump", pumpOn ? 1 : 0, 0);
  addf("pump_runs", pumpRuns, 0);
  addf("pump_sec_total", pumpSecTotal, 0);
  addf("auto", cfg.autoMode ? 1 : 0, 0);
  addf("alert", alertText.length() ? 1 : 0, 0);
  addf("rssi", WiFi.RSSI(), 0);
  snprintf(b + n, sizeof(b) - n, "}");
  bool ok = mqtt.publish(tTele.c_str(), b);
  Serial.printf("[mqtt] publish %s: %s\n", ok ? "ok" : "FAILED", b);
}

static void mqttTick() {
  if (WiFi.status() != WL_CONNECTED) return;
  uint32_t now = millis();
  if (!mqtt.connected()) {
    if (now - lastMqttTry < 5000UL) return;
    lastMqttTry = now;
    String cid = String("agrinode-") + NODE_ID;
    bool ok = strlen(MQTT_USER)
      ? mqtt.connect(cid.c_str(), MQTT_USER, MQTT_PASS, tStatus.c_str(), 1, true, "offline")
      : mqtt.connect(cid.c_str(), tStatus.c_str(), 1, true, "offline");   // last-will: broker marks us offline if we vanish
    if (ok) {
      mqtt.publish(tStatus.c_str(), "online", true);
      mqtt.subscribe(tCmd.c_str());
      Serial.printf("[mqtt] connected to %s:%u\n", MQTT_HOST, MQTT_PORT);
    } else {
      Serial.printf("[mqtt] connect failed rc=%d\n", mqtt.state());
    }
    return;
  }
  mqtt.loop();
  if (now - lastMqttPub >= MQTT_PUBLISH_MS) { lastMqttPub = now; mqttPublish(); }
}

static void senseTick() {
  soilMv   = readSoilMv();
  float rawSoil = soilPctFromMv(soilMv);
  // exponential moving average (alpha 0.3) so one noisy reading cannot trigger the pump
  soilPct  = isnan(rawSoil) ? NAN : (isnan(soilPct) ? rawSoil : 0.7f * soilPct + 0.3f * rawSoil);
  readDht();
  lightIdx = readLightIdx();
  daylightTick(SENSE_INTERVAL_MS / 1000.0f);
  irrigationTick();
  alertTick();
  // Teleplot (VS Code extension) reads lines that start with ">name:value"
  if (!isnan(soilPct))  Serial.printf(">soil_pct:%.1f\n", soilPct);
  Serial.printf(">soil_mv:%d\n", soilMv);
  if (!isnan(tempC))    Serial.printf(">temp_c:%.1f\n", tempC);
  if (!isnan(airHum))   Serial.printf(">humidity_pct:%.0f\n", airHum);
  if (!isnan(lightIdx)) Serial.printf(">light_idx:%.0f\n", lightIdx);
  Serial.printf(">pump:%d\n", pumpOn ? 1 : 0);
  Serial.printf("soil %.0f%% (%d mV) | temp %.1f C | light %.0f | pump %s | %s\n",
                soilPct, soilMv, tempC, lightIdx, pumpOn ? "ON" : "off", alertText.c_str());
}

// ------------------------------------------------------------------ camera
static bool initCamera() {
  camera_config_t c = {};
  c.ledc_channel = LEDC_CHANNEL_0;
  c.ledc_timer   = LEDC_TIMER_0;
  c.pin_d0 = CAM_Y2; c.pin_d1 = CAM_Y3; c.pin_d2 = CAM_Y4; c.pin_d3 = CAM_Y5;
  c.pin_d4 = CAM_Y6; c.pin_d5 = CAM_Y7; c.pin_d6 = CAM_Y8; c.pin_d7 = CAM_Y9;
  c.pin_xclk = CAM_XCLK; c.pin_pclk = CAM_PCLK; c.pin_vsync = CAM_VSYNC; c.pin_href = CAM_HREF;
  c.pin_sscb_sda = CAM_SIOD; c.pin_sscb_scl = CAM_SIOC;
  c.pin_pwdn = CAM_PWDN; c.pin_reset = CAM_RESET;
  c.xclk_freq_hz = 20000000;
  c.pixel_format = PIXFORMAT_JPEG;
  c.frame_size   = FRAMESIZE_VGA;
  c.jpeg_quality = 12;
  c.fb_count     = 2;
  c.fb_location  = CAMERA_FB_IN_PSRAM;
  c.grab_mode    = CAMERA_GRAB_LATEST;
  if (!psramFound()) {                                         // fall back so it still boots without PSRAM enabled
    c.frame_size = FRAMESIZE_QVGA; c.fb_count = 1; c.fb_location = CAMERA_FB_IN_DRAM;
  }
  esp_err_t err = esp_camera_init(&c);
  if (err != ESP_OK) { Serial.printf("Camera init failed 0x%x\n", err); return false; }
  return true;
}

bool cameraOk = false;

// ------------------------------------------------------------------ web UI
const char PAGE[] PROGMEM = R"rawliteral(<!doctype html><html><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1"><title>AgriNode</title>
<style>
body{font-family:system-ui,sans-serif;background:#f3f6f1;color:#1f2d1f;margin:0;padding:12px;max-width:900px;margin:auto}
h1{font-size:1.3rem;margin:4px 0 12px}
.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(190px,1fr));gap:10px}
.card{background:#fff;border-radius:10px;padding:12px;box-shadow:0 1px 3px #0002;margin-bottom:10px}
.card h3{margin:0 0 4px;font-size:.8rem;color:#5b6b5b;text-transform:uppercase;letter-spacing:.04em}
.big{font-size:2rem;font-weight:600}.sub{font-size:.8rem;color:#6a7a6a}
canvas{width:100%;height:46px;display:block;margin-top:6px}
.alert{background:#fde8e6;color:#8a1c12;border-radius:10px;padding:10px;margin-bottom:10px;font-weight:600}
.on{color:#0b7a3b}.off{color:#888}
label{display:block;font-size:.85rem;margin:6px 0}
input[type=number]{width:70px}
button{background:#2f7d32;color:#fff;border:0;border-radius:8px;padding:8px 12px;margin:3px 3px 3px 0;cursor:pointer}
button.alt{background:#566}
img{width:100%;border-radius:8px;background:#ddd;min-height:120px}
</style></head><body>
<h1>AgriNode</h1>
<div id="alert" class="alert" style="display:none"></div>
<div class="grid">
 <div class="card"><h3>Soil moisture</h3><div class="big" id="soil">--</div><div class="sub" id="soilmv"></div><canvas id="c_soil"></canvas></div>
 <div class="card"><h3>Air temperature</h3><div class="big" id="temp">--</div><div class="sub" id="hum">humidity --</div><canvas id="c_temp"></canvas></div>
 <div class="card"><h3>Light index</h3><div class="big" id="light">--</div><div class="sub" id="lighthrs"></div><canvas id="c_light"></canvas></div>
 <div class="card"><h3>Irrigation</h3><div class="big" id="pump">--</div><div class="sub" id="pumpsub"></div>
  <button onclick="act('/pump?on=1')">Water now</button><button class="alt" onclick="act('/pump?on=0')">Stop</button></div>
</div>
<div class="card"><h3>Crop camera</h3><img id="cam" alt="camera"><div class="sub">snapshot refreshes every 10 s</div></div>
<div class="card"><h3>Settings</h3>
 <label><input type="checkbox" id="auto"> Automatic irrigation</label>
 <label>Water when soil below <input type="number" id="low" min="0" max="100"> %</label>
 <label>Pump run time <input type="number" id="run" min="1" max="60"> s</label>
 <label>Minimum gap between waterings <input type="number" id="cool" min="1" max="1440"> min</label>
 <button onclick="save()">Save</button><button class="alt" onclick="act('/mute')">Mute alerts 1 h</button>
 <hr>
 <div class="sub">Calibrate: probe in dry air, press Dry. Probe in a glass of water, press Wet.</div>
 <button class="alt" onclick="act('/cal?which=dry')">Set DRY</button><button class="alt" onclick="act('/cal?which=wet')">Set WET</button>
</div>
<script>
const $=id=>document.getElementById(id);
const f=(v,d)=>v===null?'--':Number(v).toFixed(d);
async function j(u){const r=await fetch(u);return r.json();}
async function act(u){await fetch(u);tick();}
async function save(){
 await fetch('/set?low='+$('low').value+'&run='+$('run').value+'&cool='+$('cool').value+'&auto='+($('auto').checked?1:0));tick();}
let first=true;
async function tick(){try{
 const s=await j('/api/state');
 $('soil').textContent=f(s.soil,0)+' %';$('soilmv').textContent='probe '+s.soilMv+' mV';
 $('temp').textContent=f(s.temp,1)+' C';$('hum').textContent='humidity '+f(s.hum,0)+' %';
 $('light').textContent=f(s.light,0);$('lighthrs').textContent='daylight today: '+(s.lightHrs).toFixed(1)+' h';
 $('pump').textContent=s.pump?'WATERING':'idle';$('pump').className='big '+(s.pump?'on':'off');
 $('pumpsub').textContent=s.auto?('auto, next allowed in '+s.cooldownLeft+' s'):'manual mode';
 const a=$('alert');a.style.display=s.alert?'block':'none';a.textContent=s.alert;
 if(first){$('low').value=s.low;$('run').value=s.run;$('cool').value=s.cool;$('auto').checked=s.auto;first=false;}
}catch(e){}}
function draw(id,arr){const c=$(id),x=c.getContext('2d');c.width=c.clientWidth*2;c.height=92;x.clearRect(0,0,c.width,c.height);
 const v=arr.filter(a=>a!==null);if(v.length<2)return;let lo=Math.min(...v),hi=Math.max(...v);if(hi-lo<1){hi=lo+1;}
 x.strokeStyle='#2f7d32';x.lineWidth=3;x.beginPath();let st=false;
 arr.forEach((a,i)=>{if(a===null){st=false;return;}const px=i/(arr.length-1)*c.width,py=c.height-6-(a-lo)/(hi-lo)*(c.height-12);
  if(!st){x.moveTo(px,py);st=true;}else x.lineTo(px,py);});x.stroke();}
async function hist(){try{const h=await j('/api/history');draw('c_soil',h.soil);draw('c_temp',h.temp);draw('c_light',h.light);}catch(e){}}
function cam(){const i=$('cam');i.src='/capture?'+Date.now();}
tick();hist();cam();setInterval(tick,2000);setInterval(hist,30000);setInterval(cam,10000);
</script></body></html>)rawliteral";

static void jnum(String &s, const char *k, float v, int d) {
  s += '"'; s += k; s += "\":";
  if (isnan(v)) s += "null"; else s += String(v, d);
}

static void handleState() {
  uint32_t now = millis();
  long cdLeft = 0;
  if (everWatered) {
    long left = (long)cfg.coolMin * 60L - (long)((now - lastWaterAt) / 1000UL);
    cdLeft = left > 0 ? left : 0;
  }
  String s = "{";
  jnum(s, "soil", soilPct, 0);   s += ',';
  jnum(s, "temp", tempC, 1);     s += ',';
  jnum(s, "hum", airHum, 0);     s += ',';
  jnum(s, "light", lightIdx, 0); s += ',';
  s += "\"soilMv\":" + String(soilMv) + ",";
  jnum(s, "lightHrs", lightSecToday / 3600.0f, 2); s += ',';
  s += "\"pump\":" + String(pumpOn ? "true" : "false") + ",";
  s += "\"auto\":" + String(cfg.autoMode ? "true" : "false") + ",";
  s += "\"low\":" + String(cfg.lowPct) + ",\"run\":" + String(cfg.runSec) + ",\"cool\":" + String(cfg.coolMin) + ",";
  s += "\"cooldownLeft\":" + String(cdLeft) + ",";
  s += "\"rssi\":" + String(WiFi.RSSI()) + ",";
  s += "\"alert\":\"" + alertText + "\"}";
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "application/json", s);
}

static void appendSeries(String &s, const char *name, float *buf) {
  s += '"'; s += name; s += "\":[";
  for (int i = 0; i < hCount; i++) {
    int idx = (hHead - hCount + i + HISTORY_LEN * 2) % HISTORY_LEN;   // oldest -> newest
    if (i) s += ',';
    if (isnan(buf[idx])) s += "null"; else s += String(buf[idx], 1);
  }
  s += ']';
}

static void handleHistory() {
  String s = "{";
  appendSeries(s, "soil", hSoil);   s += ',';
  appendSeries(s, "temp", hTemp);   s += ',';
  appendSeries(s, "light", hLight);
  s += '}';
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "application/json", s);
}

static void handleCapture() {
  if (!cameraOk) { server.send(503, "text/plain", "camera not available"); return; }
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) { server.send(503, "text/plain", "capture failed"); return; }
  WiFiClient client = server.client();
  server.sendHeader("Cache-Control", "no-store");
  server.setContentLength(fb->len);
  server.send(200, "image/jpeg", "");
  client.write(fb->buf, fb->len);
  esp_camera_fb_return(fb);
}

static void handleSet() {
  if (server.hasArg("low"))  cfg.lowPct  = constrain(server.arg("low").toInt(), 0, 100);
  if (server.hasArg("run"))  cfg.runSec  = constrain(server.arg("run").toInt(), 1, MAX_PUMP_SEC);
  if (server.hasArg("cool")) cfg.coolMin = constrain(server.arg("cool").toInt(), 1, 1440);
  if (server.hasArg("auto")) cfg.autoMode = server.arg("auto").toInt() != 0;
  saveSettings();
  server.send(200, "text/plain", "ok");
}

static void handlePump() {
  if (server.arg("on").toInt()) setPump(true, cfg.runSec); else setPump(false);
  server.send(200, "text/plain", "ok");
}

static void handleCal() {
  int mv = readSoilMv();
  if (server.arg("which") == "dry") cfg.dryMv = mv;
  else if (server.arg("which") == "wet") cfg.wetMv = mv;
  saveSettings();
  server.send(200, "text/plain", String("saved ") + mv + " mV");
}

static void handleMute() {
  muted = true; muteUntil = millis() + 3600000UL;
  server.send(200, "text/plain", "muted");
}

// ------------------------------------------------------------------ setup / loop
void setup() {
  Serial.begin(115200);
  delay(300);
  pinMode(PIN_SOIL_PWR, OUTPUT); digitalWrite(PIN_SOIL_PWR, LOW);
  dht.begin();
  pinMode(PIN_LDR_PWR, INPUT);
  pinMode(PIN_PUMP, OUTPUT);     digitalWrite(PIN_PUMP, LOW);
  pinMode(PIN_BUZZER, OUTPUT);   digitalWrite(PIN_BUZZER, LOW);
  analogSetPinAttenuation(PIN_AIN, ADC_11db);
  analogSetPinAttenuation(PIN_SOIL_ADC, ADC_11db);
  for (int i = 0; i < HISTORY_LEN; i++) hSoil[i] = hTemp[i] = hLight[i] = NAN;

  loadSettings();
  cameraOk = initCamera();

  WiFi.mode(WIFI_STA);
  WiFi.setHostname("agrinode");
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.print("Connecting to WiFi");
  for (int i = 0; i < 40 && WiFi.status() != WL_CONNECTED; i++) { delay(500); Serial.print('.'); }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("\nOpen http://%s  (or http://agrinode.local)\n", WiFi.localIP().toString().c_str());
    configTzTime(TZ_INFO, "pool.ntp.org", "time.google.com");
  } else {
    WiFi.mode(WIFI_AP);
    WiFi.softAP("AgriNode", "growgrowgrow");
    Serial.printf("\nWiFi failed - AP mode. Join 'AgriNode' (pw growgrowgrow), open http://%s\n",
                  WiFi.softAPIP().toString().c_str());
  }
  MDNS.begin("agrinode");

  tTele   = String("agrinode/") + NODE_ID + "/telemetry";
  tStatus = String("agrinode/") + NODE_ID + "/status";
  tCmd    = String("agrinode/") + NODE_ID + "/cmd";
  mqtt.setServer(MQTT_HOST, MQTT_PORT);
  mqtt.setCallback(onMqtt);
  mqtt.setBufferSize(512);
  mqtt.setSocketTimeout(3);                                  // keep a dead broker from freezing the web server

  server.on("/", []() { server.send_P(200, "text/html", PAGE); });
  server.on("/api/state", handleState);
  server.on("/api/history", handleHistory);
  server.on("/capture", handleCapture);
  server.on("/set", handleSet);
  server.on("/pump", handlePump);
  server.on("/cal", handleCal);
  server.on("/mute", handleMute);
  server.begin();

  beep(1);
  senseTick();
  lastSenseAt = lastHistAt = millis();
}

void loop() {
  server.handleClient();
  mqttTick();
  uint32_t now = millis();
  if (now - lastSenseAt >= SENSE_INTERVAL_MS) { lastSenseAt = now; senseTick(); }
  if (now - lastHistAt >= HISTORY_INTERVAL_S * 1000UL) { lastHistAt = now; pushHistory(); }
  if (pumpOn && (int32_t)(now - pumpOffAt) >= 0) setPump(false);   // stop on time even between sense ticks
  delay(2);
}