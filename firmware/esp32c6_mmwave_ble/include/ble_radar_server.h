#pragma once
#include <Arduino.h>
#include "radar_payload.h"

// 简单封装 NimBLE GATT Server：
// - Service: EVADAR_BLE_SERVICE_UUID
// - Notify Char: EVADAR_BLE_NOTIFY_UUID  -- 雷达 JSON 数据
// - Config Char: EVADAR_BLE_CONFIG_UUID  -- 接收主机命令字符串（保留）
// - Health Char: EVADAR_BLE_HEALTH_UUID  -- 周期发布固件健康 JSON
namespace evadar {

struct HealthSnapshot {
  uint32_t uptime_ms;
  uint32_t notify_total;
  uint32_t notify_dropped;     // 因未连接被丢弃
  uint32_t radar_no_data_ms;   // 距离上次成功 update 的毫秒
  bool client_connected;
  const char* fw_version;
};

void ble_begin();
bool ble_is_connected();
void ble_notify_sample(const RadarSample& s);
void ble_publish_health(const HealthSnapshot& h);

// 主循环里调用，处理周期性任务（重连广播、超时打印）。
void ble_loop_tick();

}  // namespace evadar
