#pragma once
#include <Arduino.h>

// 一帧雷达样本（内部数据结构）
struct RadarSample {
  uint32_t seq;
  uint32_t timestamp_ms;
  float breath_rate;   // BPM
  float heart_rate;    // BPM
  float distance_cm;
  uint8_t status;      // 见 project_config.h EVADAR_ST_*
};

// 序列化为紧凑 JSON，写入 out（不含末尾换行），返回字节数
// 形如：{"v":1,"seq":12,"t":34567,"br":16.2,"hr":78.4,"d":52.1,"st":7}
size_t radar_payload_to_json(const RadarSample& s, char* out, size_t out_size);
