#pragma once
// EV-ADS ESP32-C6 BLE 雷达节点 - 编译期参数

// ---------- BLE ----------
#define EVADAR_BLE_DEVICE_NAME      "EVADAR-C6"
#define EVADAR_BLE_SERVICE_UUID     "0000ad01-0000-1000-8000-00805f9b34fb"
#define EVADAR_BLE_NOTIFY_UUID      "0000ad02-0000-1000-8000-00805f9b34fb"
#define EVADAR_BLE_CONFIG_UUID      "0000ad03-0000-1000-8000-00805f9b34fb"
#define EVADAR_BLE_HEALTH_UUID      "0000ad04-0000-1000-8000-00805f9b34fb"

// ---------- 节流 ----------
// 雷达 update 频率约 10Hz，对外 BLE Notify 也维持 5-10Hz
#define EVADAR_NOTIFY_MIN_INTERVAL_MS 100
#define EVADAR_HEALTH_INTERVAL_MS     2000

// ---------- 状态位 ----------
#define EVADAR_ST_BREATH    0x01
#define EVADAR_ST_HEART     0x02
#define EVADAR_ST_DISTANCE  0x04
#define EVADAR_ST_PRESENCE  0x08

// ---------- 业务阈值 ----------
#define EVADAR_PRESENCE_MAX_CM     300.0f
#define EVADAR_HR_LOW_RED_BPM      75.0f
