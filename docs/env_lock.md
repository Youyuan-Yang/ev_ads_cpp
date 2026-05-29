# 环境与版本锁定

## 1. 主控

- 硬件：Rockchip RK3588，建议 8GB 内存。
- 系统：Ubuntu 22.04 LTS arm64。
- 内核：建议 5.10.x BSP，需包含 `media`、`videobuf2`、`bluetooth`、`i2c-dev`。

## 2. ROS 2 与 C++

| 组件 | 版本建议 | 用途 |
|---|---|---|
| ROS 2 | Humble | rclcpp、消息、launch |
| g++ | 11+ | C++17 构建 |
| cmake | 3.16+ | 构建 |
| libopencv-dev | 4.x | 摄像头、JPEG、OpenCV DNN ONNX fallback |
| sqlite3 / libsqlite3-dev | 3.x | 事件库 SQLite/WAL 存储 |
| rosbag2 | Humble | 可选录包 |

说明：ROS 2/colcon 的系统工具会依赖 `python3-*` 包，但该项目副本不再保留 Python 运行代码。

## 3. ESP32-C6 固件

- PlatformIO Core：6.1+
- framework：Arduino
- board：`seeed_xiao_esp32c6`
- BLE 设备名：`EVADAR-C6`
- Service UUID：`0000ad01-0000-1000-8000-00805f9b34fb`
- Notify Char：`0000ad02-0000-1000-8000-00805f9b34fb`
- Config Char：`0000ad03-0000-1000-8000-00805f9b34fb`
- Health Char：`0000ad04-0000-1000-8000-00805f9b34fb`

## 4. 模型

| 模型 | 当前状态 | 目标路径 |
|---|---|---|
| 后置 YOLO11n | 已放入 ONNX，SHA256 `4e16b0662b3d9ceff65bc7ff79fca909f62673dc9e08aa74ee4a5e5e1511cf5d` | `models/onnx/rear_yolo.onnx` |
| 驾驶员 YuNet | 已放入 ONNX，SHA256 `8f2383e4dd3cfbb4553ea8718107fc0423210dc964f9f4280604804ed2552fa4` | `models/onnx/driver_face_yunet.onnx` |
| 驾驶员 DMS YOLO | 已放入 ONNX，SHA256 `cca6dffd84d0596e8ca620dd7ba91dd2624b29fde740bc76d420a5f74e908187` | `models/onnx/driver_dms_yolo.onnx` |
| 前置道路危险 | 未训练 | `models/onnx/front_road_hazard.onnx` |
| RKNN 版本 | 未转换 | `models/rknn/*.rknn` |

每个模型必须记录 hash、输入尺寸、类别表、阈值、测试集指标和许可证。

## 5. 关键话题

| 话题 | 类型 | 频率 |
|---|---|---|
| `/sensor/mmwave/vital` | `ev_ads_interfaces/msg/MmWaveVital` | 5-10 Hz |
| `/camera/{front,rear,driver}/image_raw/compressed` | `sensor_msgs/CompressedImage` | 25-30 Hz |
| `/vehicle/motion` | `ev_ads_interfaces/msg/VehicleMotion` | 100 Hz |
| `/perception/front_risk` | `ev_ads_interfaces/msg/FrontRisk` | 10 Hz |
| `/perception/blind_spot` | `ev_ads_interfaces/msg/BlindSpotState` | 10 Hz |
| `/perception/driver_state` | `ev_ads_interfaces/msg/DriverState` | 10 Hz |
| `/decision/risk_state` | `ev_ads_interfaces/msg/RiskState` | 20 Hz |
| `/decision/warning_cmd` | `ev_ads_interfaces/msg/WarningCommand` | 事件触发 |
| `/decision/brake_cmd` | `ev_ads_interfaces/msg/BrakeCommand` | 默认禁用 |
