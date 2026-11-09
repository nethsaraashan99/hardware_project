#include "HardwareConfig.h"
#if __has_include("secrets.h")
#include "secrets.h"
#else
#include "secrets.example.h"
#endif
#define BLYNK_MAX_SENDBYTES 1200
#define BLYNK_MAX_READBYTES 1200
#include <Arduino.h>
#include <WiFi.h>
#include <BlynkSimpleEsp32_SSL.h>
#include <ArduinoJson.h>
#include <Preferences.h>
#include <esp_random.h>
#include "ControlCore.h"
#if ENABLE_LD2410
#include <ld2410.h>
ld2410 radar;
#endif

using namespace HomeAuto;
ControlCore core;
Preferences preferences;
SemaphoreHandle_t snapshotMutex;
QueueHandle_t commandQueue;
struct Incoming { uint8_t pin; char json[600]; };
struct Ack { char id[41] = ""; bool ok = false; char code[40] = "none"; };
Ack currentAck, cache[8];
bool validId(const char *id);
bool readInt(JsonVariantConst value, int32_t low, int32_t high, int32_t &out);
bool parseConfig(JsonDocument &doc, Config &c);
void acknowledge(const char *id, Result result, bool remember);
void enqueue(uint8_t pin, const char *text);
void processCommand(const Incoming &item, uint32_t now);
void makeSnapshot(uint32_t now);
void networkTask(void *parameter);
int cacheIndex = 0;
char snapshot[1025] = "", deviceId[41], bootId[41], configId[41] = "initial";
uint32_t sequenceNumber = 0;
volatile uint32_t queueDrops = 0;
bool storageOK = false;

bool validId(const char *id) {
  if (!id || !id[0] || strlen(id) > 40) return false;
  for (const char *p = id; *p; ++p) if (!isalnum(static_cast<unsigned char>(*p)) && *p != '-' && *p != '_') return false;
  return true;
}
bool readInt(JsonVariantConst value, int32_t low, int32_t high, int32_t &out) {
  if (!value.is<int32_t>()) return false;
  const int32_t n = value.as<int32_t>(); if (n < low || n > high) return false; out = n; return true;
}
bool parseConfig(JsonDocument &doc, Config &c) {
  const char *room = doc["room"] | ""; if (!validId(room)) return false;
  int32_t gas, hys, dark, vac, minimum, confirm, hold, range, autoRule;
  if (!readInt(doc["gas"],1,4095,gas) || !readInt(doc["hys"],1,4094,hys) || hys >= gas || !readInt(doc["dark"],1,99,dark) || !readInt(doc["vac"],5000,7200000,vac) || !readInt(doc["min"],1000,120000,minimum) || !readInt(doc["confirm"],1,5,confirm) || !readInt(doc["hold"],1000,7200000,hold) || !readInt(doc["range"],50,600,range) || !readInt(doc["auto"],0,1,autoRule)) return false;
  strlcpy(c.room,room,sizeof(c.room)); c.gas=gas; c.hys=hys; c.dark=dark; c.vac=vac; c.min=minimum; c.confirm=confirm; c.hold=hold; c.range=range; c.autoRule=autoRule;
  return true;
}
void acknowledge(const char *id, Result result, bool remember = true) {
  strlcpy(currentAck.id, id, sizeof(currentAck.id)); currentAck.ok = result.ok; strlcpy(currentAck.code, result.code, sizeof(currentAck.code));
  if (remember) { cache[cacheIndex] = currentAck; cacheIndex = (cacheIndex + 1) % 8; }
}
void enqueue(uint8_t pin, const char *text) {
  if (!text || strlen(text) >= sizeof(Incoming::json)) { ++queueDrops; return; }
  Incoming item; item.pin = pin; strlcpy(item.json, text, sizeof(item.json));
  if (xQueueSend(commandQueue, &item, 0) != pdTRUE) ++queueDrops;
}
BLYNK_WRITE(V8) { enqueue(8, param.asStr()); }
BLYNK_WRITE(V9) { enqueue(9, param.asStr()); }
// Deliberately do not sync/replay retained V8/V9 commands on reconnect.
// V6 is output telemetry, not a second unacknowledged command path.

void processCommand(const Incoming &item, uint32_t now) {
  JsonDocument doc;
  if (deserializeJson(doc, item.json)) return;
  const char *id = doc["id"] | ""; if (!validId(id)) return;
  const char *boot = doc["b"] | "";
  if (strcmp(boot,bootId)) { acknowledge(id,{false,"stale_boot"},false); return; }
  for (const auto &a : cache) if (!strcmp(a.id,id)) { currentAck = a; return; }
  if (item.pin == 9) {
    if (core.alarm) { acknowledge(id,{false,"gas_alarm_setting_lock"}); return; }
    Config candidate;
    if (!parseConfig(doc,candidate)) { acknowledge(id,{false,"invalid_config"}); return; }
    if (!storageOK || preferences.putString("config",item.json) == 0) { acknowledge(id,{false,"storage_failed"}); return; }
    core.applyConfig(candidate,now); strlcpy(configId,id,sizeof(configId)); acknowledge(id,{true,"ok"});
  } else {
    const char *op = doc["op"] | "";
    int32_t on=0, hold=static_cast<int32_t>(core.config.hold);
    if (strcmp(op,"resume") && (!readInt(doc["on"],0,1,on) || (doc["hold"].isNull() ? false : !readInt(doc["hold"],1000,7200000,hold)))) { acknowledge(id,{false,"invalid_command"}); return; }
    acknowledge(id,core.command(op,on == 1,hold,now,!strcmp(op,"routine") ? "routine" : "manual"));
  }
}
void makeSnapshot(uint32_t now) {
  JsonDocument doc;
  doc["v"]=2; doc["d"]=deviceId; doc["b"]=bootId; doc["q"]=++sequenceNumber; doc["u"]=now;
  doc["g"]=core.last.gasRaw; doc["m"]=core.last.motion ? 1:0; doc["p"]=core.previousPresence ? 1:0; doc["l"]=core.last.light; doc["r"]=core.relay ? 1:0;
  doc["al"]=core.alarm ? 1:0; doc["w"]=core.warming ? 1:0; doc["ov"]=core.remaining(now); doc["cfg"]=configId;
  doc["rd"]=ENABLE_LD2410; doc["rv"]=core.last.radarValid ? 1:0;
  if (core.last.distanceCm < 0) doc["dc"]=nullptr; else doc["dc"]=core.last.distanceCm;
  auto c=doc["c"].to<JsonObject>(); const Config &v=core.config;
  c["room"]=v.room; c["gas"]=v.gas; c["hys"]=v.hys; c["dark"]=v.dark; c["vac"]=v.vac; c["min"]=v.min; c["confirm"]=v.confirm; c["hold"]=v.hold; c["range"]=v.range; c["auto"]=v.autoRule ? 1:0;
  doc["ec"]=core.eventCounter; doc["er"]=queueDrops;
  auto ack=doc["ack"].to<JsonObject>(); ack["i"]=currentAck.id; ack["ok"]=currentAck.ok ? 1:0; ack["c"]=currentAck.code;
  auto events=doc["e"].to<JsonArray>();
  for (int i=0;i<core.eventCount;++i) { auto e=events.add<JsonObject>(); e["i"]=core.events[i].id; e["t"]=core.events[i].time; e["k"]=core.events[i].kind; e["s"]=core.events[i].source; }
  while (measureJson(doc)>1000 && events.size()) events.remove(0);
  char out[1025]; serializeJson(doc,out,sizeof(out));
  if (xSemaphoreTake(snapshotMutex,pdMS_TO_TICKS(2)) == pdTRUE) { strlcpy(snapshot,out,sizeof(snapshot)); xSemaphoreGive(snapshotMutex); }
}
void networkTask(void *) {
  Blynk.config(BLYNK_AUTH_TOKEN,BLYNK_CLOUD_HOST,443);
  uint32_t wifiAttempt=0, cloudAttempt=0, published=0;
  for (;;) {
    const uint32_t now=millis();
    if (WiFi.status()!=WL_CONNECTED) { if (now-wifiAttempt>=10000) { wifiAttempt=now; WiFi.reconnect(); } }
    else if (!Blynk.connected()) { if (now-cloudAttempt>=5000) { cloudAttempt=now; Blynk.connect(1000); } }
    else {
      Blynk.run();
      if (now-published>=PUBLISH_INTERVAL_MS) {
        char copy[1025]="";
        if (xSemaphoreTake(snapshotMutex,pdMS_TO_TICKS(5)) == pdTRUE) { strlcpy(copy,snapshot,sizeof(copy)); xSemaphoreGive(snapshotMutex); }
        if (copy[0]) {
          Blynk.virtualWrite(V7,copy);
          // Retain the original read-only sensor/dashboard channels.
          JsonDocument data;
          if (!deserializeJson(data,copy)) { Blynk.virtualWrite(V0,data["g"].as<int>()); Blynk.virtualWrite(V1,data["m"].as<int>()); Blynk.virtualWrite(V2,data["l"].as<int>()); Blynk.virtualWrite(V6,data["r"].as<int>()); }
        }
        published=now;
      }
    }
    vTaskDelay(pdMS_TO_TICKS(20));
  }
}
void setup() {
  Serial.begin(115200);
  digitalWrite(PIN_RELAY,RELAY_ACTIVE_LOW ? HIGH:LOW); pinMode(PIN_RELAY,OUTPUT);
  digitalWrite(PIN_BUZZER,LOW); pinMode(PIN_BUZZER,OUTPUT); pinMode(PIN_PIR,INPUT);
  analogReadResolution(12); analogSetPinAttenuation(PIN_MQ4,ADC_11db); analogSetPinAttenuation(PIN_LDR,ADC_11db);
  snapshotMutex=xSemaphoreCreateMutex(); commandQueue=xQueueCreate(6,sizeof(Incoming));
  if (!snapshotMutex || !commandQueue) { Serial.println("Initialization failed; output remains OFF."); while(true) delay(1000); }
  snprintf(deviceId,sizeof(deviceId),"SH04-%06lx",static_cast<unsigned long>(ESP.getEfuseMac() & 0xFFFFFF));
  snprintf(bootId,sizeof(bootId),"boot-%08lx",static_cast<unsigned long>(esp_random()));
  storageOK=preferences.begin("homeauto2",false);
  if (storageOK) { String saved=preferences.getString("config",""); JsonDocument doc; Config c; if (saved.length() && !deserializeJson(doc,saved) && parseConfig(doc,c)) { core.config=c; const char *id=doc["id"] | "saved"; strlcpy(configId,id,sizeof(configId)); } }
  core.emit("restart","controller",millis());
#if ENABLE_LD2410
  Serial2.begin(256000,SERIAL_8N1,RADAR_RX_PIN,RADAR_TX_PIN); radar.begin(Serial2,false);
#endif
  WiFi.mode(WIFI_STA); WiFi.persistent(false); WiFi.setAutoReconnect(true); WiFi.setSleep(false);
  if (strcmp(WIFI_SSID,"YOUR_2_4_GHZ_WIFI") && strcmp(BLYNK_AUTH_TOKEN,"YOUR_DEVICE_AUTH_TOKEN")) {
    WiFi.begin(WIFI_SSID,WIFI_PASSWORD);
    if (xTaskCreatePinnedToCore(networkTask,"network",12288,nullptr,1,nullptr,0)!=pdPASS) Serial.println("Network task unavailable; local controller only.");
  } else Serial.println("Fill secrets.h to enable Blynk. Local low-voltage controller is running.");
  Serial.println("Home Automation V2 ready. GPIO relay only; mains installation is outside this sketch's commissioning.");
}
void loop() {
  static uint32_t sampled=0, snapAt=0; static float gasFiltered=-1, lightFiltered=-1;
  const uint32_t now=millis();
#if ENABLE_LD2410
  radar.read();
#endif
  Incoming item;
  if (xQueueReceive(commandQueue,&item,0)==pdTRUE) processCommand(item,now);
  if (now-sampled>=SENSOR_INTERVAL_MS) {
    sampled=now; const int gas=analogRead(PIN_MQ4), light=analogRead(PIN_LDR);
    gasFiltered=gasFiltered<0 ? gas:gasFiltered+0.35f*(gas-gasFiltered); lightFiltered=lightFiltered<0 ? light:lightFiltered+0.35f*(light-lightFiltered);
    Input x; x.now=now; x.gasRaw=static_cast<int>(gasFiltered+0.5f); x.light=constrain(static_cast<int>(lightFiltered*100.0f/4095.0f),0,100); x.motion=now>=PIR_STARTUP_INHIBIT_MS && digitalRead(PIN_PIR)==HIGH;
#if ENABLE_LD2410
    x.radarEnabled=true; x.radarValid=radar.isConnected(); x.radarPresent=x.radarValid && radar.presenceDetected();
    const int moving=radar.movingTargetDetected() ? radar.movingTargetDistance():-1, stationary=radar.stationaryTargetDetected() ? radar.stationaryTargetDistance():-1;
    x.distanceCm=moving>=0 && stationary>=0 ? min(moving,stationary):max(moving,stationary);
#endif
    core.tick(x);
  }
  // Commands cannot force ordinary switching during a gas alert. Outputs are
  // driven solely from the local core, independently of cloud blocking calls.
  digitalWrite(PIN_RELAY,(core.relay != RELAY_ACTIVE_LOW) ? HIGH:LOW); digitalWrite(PIN_BUZZER,core.alarm ? HIGH:LOW);
  if (now-snapAt>=PUBLISH_INTERVAL_MS) { snapAt=now; makeSnapshot(now); }
  delay(2);
}
