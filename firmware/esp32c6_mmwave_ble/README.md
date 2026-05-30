# ESP32-C6 毫米波 BLE 固件（EVADAR-C6）

基于 `MMRADAR_test` 升级，对外暴露 `EVADAR-C6` BLE GATT 服务，向 RK3588 / Mac 发送
Seeed MR60BHA2 的呼吸率、心率、距离与状态位。

## 1. 硬件
- Seeed Studio XIAO ESP32-C6
- Seeed MR60BHA2 60GHz 毫米波传感器（UART 接 ESP32-C6 RX/TX）
- SSD1306 0.91" OLED（D0=SCL, D10=SDA, 软 I2C）
- 1×WS2812 NeoPixel（D1）

## 2. 编译 & 烧录
```bash
cd ev_ads/firmware/esp32c6_mmwave_ble
pio run
pio run -t upload
pio device monitor -b 115200
```

烧录成功后串口会输出：
```
EV-ADS mmWave BLE firmware v0.1.0
[mmWave] driver started
[BLE] advertising as EVADAR-C6
```

## 3. 与原 MMRADAR_test 的差异
| 项目 | 旧（MMRADAR） | 新（EVADAR-C6） |
|------|---------------|-----------------|
| 设备名 | `MMRADAR` | `EVADAR-C6` |
| Service UUID | NUS `6E400001-...` | `0000ad01-0000-1000-8000-00805f9b34fb` |
| Notify | NUS TX | `0000ad02-...`，紧凑 JSON |
| Config | 无 | `0000ad03-...`，主机写控制命令 |
| Health | 无 | `0000ad04-...`，2s 一次健康 JSON |
| 载荷 | 多行文本 (`heart_rate: 78.40\n`) | `{"v":1,"seq":..,"t":..,"br":..,"hr":..,"d":..,"st":..}` |
| 节流 | 每次更新都 notify | 100ms 一帧 (5-10 Hz) |
| OLED / NeoPixel / 串口 | 保留 | 保留 |

## 4. 协议
详见 [`../../docs/ble_protocol.md`](../../docs/ble_protocol.md)。

## 5. 调试说明

该项目副本已删除 Python BLE 工具。临时调试可用手机 BLE 调试软件查看
`EVADAR-C6` 的 GATT notify；板端 C++ 运行链路使用 `mmwave_vital_node`。
真实 BLE 接收需要在 RK3588 上补接 BlueZ D-Bus 后端，当前 C++ 节点已保留
`mode:=ble` 入口并在未接通时发布 `DISCONNECTED` 健康状态。
