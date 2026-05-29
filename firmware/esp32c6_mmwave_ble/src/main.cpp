// EV-ADS · ESP32-C6 + Seeed MR60BHA2 雷达 → BLE GATT (EVADAR-C6)
//
// 改自 MMRADAR_test/MMRADAR/src/main.cpp，保留：
//   - 串口打印 breath_rate / heart_rate / distance / presence
//   - OLED (SSD1306, SW_I2C on D0/D10)
//   - NeoPixel (D1, 心率<75 红，否则绿)
// 新增：
//   - NimBLE GATT Server，设备名 EVADAR-C6，UUID 见 docs/ble_protocol.md
//   - Notify 紧凑 JSON: {"v":1,"seq":..,"t":..,"br":..,"hr":..,"d":..,"st":..}
//   - 5-10 Hz 节流；Health Characteristic 周期发布
//
#include <Arduino.h>
#include <U8x8lib.h>
#include <Adafruit_NeoPixel.h>
#include "Seeed_Arduino_mmWave.h"

#include "project_config.h"
#include "radar_payload.h"
#include "ble_radar_server.h"

#ifndef EVADAR_FIRMWARE_VERSION
#define EVADAR_FIRMWARE_VERSION "0.1.0"
#endif

#ifdef ESP32
#  include <HardwareSerial.h>
HardwareSerial mmWaveSerial(0);
#else
#  define mmWaveSerial Serial1
#endif

U8X8_SSD1306_128X64_NONAME_SW_I2C u8x8(
    /*clock=*/D0, /*data=*/D10, /*reset=*/U8X8_PIN_NONE);

Adafruit_NeoPixel pixels(1, D1, NEO_GRB + NEO_KHZ800);

SEEED_MR60BHA2 mmWave;
static const char* TAG_Breath   = "BreathRate";
static const char* TAG_Heart    = "HeartRate";
static const char* TAG_Distance = "Distance";

enum DisplayLabel { LABEL_BREATH, LABEL_HEART, LABEL_DISTANCE };

static void updateDisplay(DisplayLabel label, float value) {
  static float last_breath = -1.0f;
  static float last_heart  = -1.0f;
  static float last_dist   = -1.0f;
  switch (label) {
    case LABEL_BREATH:
      if (value == last_breath) return;
      u8x8.setCursor(11, 3); u8x8.print(value); last_breath = value; break;
    case LABEL_HEART:
      if (value == last_heart) return;
      u8x8.setCursor(11, 5); u8x8.print(value); last_heart = value; break;
    case LABEL_DISTANCE:
      if (value == last_dist) return;
      u8x8.setCursor(11, 7); u8x8.print(value); last_dist = value; break;
  }
}

// ---- 状态聚合：将一轮 update 内读到的字段合并成 RadarSample ----
struct RoundState {
  bool got_breath = false;
  bool got_heart  = false;
  bool got_dist   = false;
  bool got_presence = false;
  bool presence_value = false;
  float breath = 0.f;
  float heart  = 0.f;
  float dist   = 0.f;
};

static uint32_t g_seq = 0;
static uint32_t g_last_notify_ms = 0;
static uint32_t g_last_health_ms = 0;
static uint32_t g_last_radar_data_ms = 0;
static uint32_t g_last_waiting_log_ms = 0;

void setup() {
  Serial.begin(115200);
  delay(1500);
  Serial.println("=========================================");
  Serial.print  ("EV-ADS mmWave BLE firmware v");
  Serial.println(EVADAR_FIRMWARE_VERSION);
  Serial.println("=========================================");

  mmWave.begin(&mmWaveSerial);
  Serial.println("[mmWave] driver started");

  evadar::ble_begin();

  pixels.begin();
  pixels.setPixelColor(0, pixels.Color(0, 125, 0));
  pixels.setBrightness(8);
  pixels.show();

  u8x8.begin();
  u8x8.setFlipMode(3);
  u8x8.clearDisplay();
  u8x8.setFont(u8x8_font_victoriamedium8_r);
  u8x8.setCursor(1, 0); u8x8.print("EV-ADS Vital");
  u8x8.setCursor(0, 3); u8x8.print(TAG_Breath);
  u8x8.setCursor(0, 5); u8x8.print(TAG_Heart);
  u8x8.setCursor(0, 7); u8x8.print(TAG_Distance);
  u8x8.setFont(u8x8_font_chroma48medium8_n);
}

static void serial_print_round(const RoundState& r) {
  // 保持与 MMRADAR_test 一致的可读串口格式，便于沿用现有调试脚本。
  if (r.got_breath)
    { Serial.print("breath_rate: "); Serial.println(r.breath, 2); }
  if (r.got_heart)
    { Serial.print("heart_rate : "); Serial.println(r.heart, 2); }
  if (r.got_dist)
    { Serial.print("distance   : "); Serial.println(r.dist, 2); }
  if (r.got_presence)
    { Serial.print("presence   : "); Serial.println(r.presence_value ? 1 : 0); }
}

void loop() {
  RoundState r;

  if (mmWave.update(100)) {
    g_last_radar_data_ms = millis();

    bool detected = false;
    if (mmWave.getHumanDetected(detected)) {
      r.got_presence = true;
      r.presence_value = detected;
    }

    float v;
    if (mmWave.getDistance(v))   { r.got_dist = true;   r.dist   = v; }
    if (mmWave.getBreathRate(v)) { r.got_breath = true; r.breath = v; }
    if (mmWave.getHeartRate(v))  { r.got_heart = true;  r.heart  = v; }

    // 若驱动未上报 presence，则用距离回退判定
    if (!r.got_presence && r.got_dist) {
      r.got_presence = true;
      r.presence_value = (r.dist > 0.f && r.dist <= EVADAR_PRESENCE_MAX_CM);
    }

    if (r.got_breath) updateDisplay(LABEL_BREATH, r.breath);
    if (r.got_heart)  updateDisplay(LABEL_HEART,  r.heart);
    if (r.got_dist)   updateDisplay(LABEL_DISTANCE, r.dist);

    if (r.got_heart) {
      if (r.heart > 0.f && r.heart < EVADAR_HR_LOW_RED_BPM)
        pixels.setPixelColor(0, pixels.Color(125, 0, 0));
      else
        pixels.setPixelColor(0, pixels.Color(0, 125, 0));
      pixels.show();
    }

    serial_print_round(r);
  } else if (millis() - g_last_waiting_log_ms > 2000) {
    Serial.println("[mmWave] waiting for data...");
    g_last_waiting_log_ms = millis();
  }

  // ---- BLE Notify 节流：每 EVADAR_NOTIFY_MIN_INTERVAL_MS 一帧 ----
  uint32_t now = millis();
  if (now - g_last_notify_ms >= EVADAR_NOTIFY_MIN_INTERVAL_MS) {
    static RadarSample last_sample = {0, 0, 0.f, 0.f, 0.f, 0};
    // 累积字段：本轮没读到就沿用上次值，但状态位只反映本轮
    RadarSample s = last_sample;
    s.seq          = ++g_seq;
    s.timestamp_ms = now;
    s.status       = 0;
    if (r.got_breath) { s.breath_rate = r.breath; s.status |= EVADAR_ST_BREATH; }
    if (r.got_heart)  { s.heart_rate  = r.heart;  s.status |= EVADAR_ST_HEART; }
    if (r.got_dist)   { s.distance_cm = r.dist;   s.status |= EVADAR_ST_DISTANCE; }
    if (r.got_presence && r.presence_value)  s.status |= EVADAR_ST_PRESENCE;

    evadar::ble_notify_sample(s);
    last_sample = s;
    g_last_notify_ms = now;
  }

  // ---- Health Notify ----
  if (now - g_last_health_ms >= EVADAR_HEALTH_INTERVAL_MS) {
    evadar::HealthSnapshot h{};
    h.uptime_ms = now;
    h.notify_total = g_seq;  // 近似
    h.notify_dropped = 0;    // 由 ble_radar_server 内部统计，这里简单填 0
    h.radar_no_data_ms = (g_last_radar_data_ms == 0)
                            ? now
                            : (now - g_last_radar_data_ms);
    h.client_connected = evadar::ble_is_connected();
    h.fw_version = EVADAR_FIRMWARE_VERSION;
    evadar::ble_publish_health(h);
    g_last_health_ms = now;
  }

  evadar::ble_loop_tick();
}
