#include "ble_radar_server.h"
#include "project_config.h"

#include <NimBLEDevice.h>
#include <string.h>

namespace evadar {

namespace {
NimBLEServer* g_server = nullptr;
NimBLECharacteristic* g_notify_char = nullptr;
NimBLECharacteristic* g_config_char = nullptr;
NimBLECharacteristic* g_health_char = nullptr;
bool g_connected = false;
uint32_t g_notify_total = 0;
uint32_t g_notify_dropped = 0;

class ServerCallbacks : public NimBLEServerCallbacks {
  void onConnect(NimBLEServer* server, NimBLEConnInfo& info) override {
    g_connected = true;
    Serial.print("[BLE] connected: ");
    Serial.println(info.getAddress().toString().c_str());
    // 缩小连接间隔以提高 5-10Hz Notify 的可靠性
    server->updateConnParams(info.getConnHandle(), 12, 24, 0, 200);
  }
  void onDisconnect(NimBLEServer* server, NimBLEConnInfo& /*info*/,
                    int reason) override {
    g_connected = false;
    Serial.print("[BLE] disconnected, reason=");
    Serial.print(reason);
    Serial.println(", restart advertising");
    NimBLEDevice::startAdvertising();
  }
};

class ConfigCallbacks : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic* chr, NimBLEConnInfo& /*info*/) override {
    std::string v = chr->getValue();
    Serial.print("[BLE] config write: ");
    Serial.println(v.c_str());
    // 保留：未来支持 {"cmd":"ping"} / {"cmd":"led","on":1}
  }
};
}  // namespace

void ble_begin() {
  NimBLEDevice::init(EVADAR_BLE_DEVICE_NAME);
  NimBLEDevice::setPower(ESP_PWR_LVL_P9);
  NimBLEDevice::setMTU(185);

  g_server = NimBLEDevice::createServer();
  g_server->setCallbacks(new ServerCallbacks());

  NimBLEService* svc = g_server->createService(EVADAR_BLE_SERVICE_UUID);

  g_notify_char = svc->createCharacteristic(
      EVADAR_BLE_NOTIFY_UUID,
      NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);
  g_notify_char->setValue("{\"v\":1,\"seq\":0}");

  g_config_char = svc->createCharacteristic(
      EVADAR_BLE_CONFIG_UUID,
      NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE);
  g_config_char->setCallbacks(new ConfigCallbacks());

  g_health_char = svc->createCharacteristic(
      EVADAR_BLE_HEALTH_UUID,
      NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);
  g_health_char->setValue("{\"v\":1,\"state\":\"boot\"}");

  svc->start();

  NimBLEAdvertising* adv = NimBLEDevice::getAdvertising();
  adv->addServiceUUID(EVADAR_BLE_SERVICE_UUID);
  adv->setName(EVADAR_BLE_DEVICE_NAME);
  adv->enableScanResponse(true);
  adv->start();

  Serial.print("[BLE] advertising as ");
  Serial.println(EVADAR_BLE_DEVICE_NAME);
}

bool ble_is_connected() { return g_connected; }

void ble_notify_sample(const RadarSample& s) {
  if (g_notify_char == nullptr) return;
  char buf[160];
  size_t n = radar_payload_to_json(s, buf, sizeof(buf));
  if (n == 0) return;

  // 即使无客户端也写入 value，以便 read 调试
  g_notify_char->setValue(reinterpret_cast<const uint8_t*>(buf), n);

  if (!g_connected) {
    g_notify_dropped++;
    return;
  }
  g_notify_char->notify();
  g_notify_total++;
}

void ble_publish_health(const HealthSnapshot& h) {
  if (g_health_char == nullptr) return;
  char buf[200];
  int n = snprintf(
      buf, sizeof(buf),
      "{\"v\":1,\"up\":%lu,\"nt\":%lu,\"nd\":%lu,\"rs\":%lu,\"cn\":%u,\"fw\":\"%s\"}",
      (unsigned long)h.uptime_ms,
      (unsigned long)h.notify_total,
      (unsigned long)h.notify_dropped,
      (unsigned long)h.radar_no_data_ms,
      (unsigned)(h.client_connected ? 1 : 0),
      h.fw_version ? h.fw_version : "?");
  if (n <= 0) return;
  g_health_char->setValue(reinterpret_cast<const uint8_t*>(buf), (size_t)n);
  if (g_connected) g_health_char->notify();
}

void ble_loop_tick() {
  // NimBLE 库自身已用 FreeRTOS 任务驱动，这里保留钩子，方便后续扩展。
}

}  // namespace evadar
