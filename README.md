# EV-ADS RK3588 C++ 版本

该项目副本面向 8GB RK3588 部署，除测试外不再保留项目内 Python 运行代码。ROS 2/colcon 本身仍会使用系统 Python，这是 ROS 工具链依赖，不属于该项目运行逻辑。

## 1. 当前状态

- `ros2_ws/src` 下只保留一个 ROS2 包：`ev_ads_runtime_cpp`。
- 摄像头、IMU、前向感知、后向感知、驾驶员监测、融合决策、HMI、事件记录、毫米波占位入口均为 C++。
- 启动和运行参数统一为 ROS2 XML launch：`ev_ads_runtime.launch.xml`，不再混用 YAML/TOML。
- 所有项目自有配置文件集中放在根目录 `config/`，ROS2 包只负责安装和运行。
- `models/onnx/rear_yolo.onnx` 已放入，用于后置鱼眼第一版靠近识别；后置节点已支持 OpenCV fisheye 去畸变，参数写在 XML launch 中。
- 驾驶员监测已放入 YuNet 人脸 ONNX 与 SafeDrive DMS YOLO ONNX，节点按人脸检测 + 闭眼/哈欠/手机行为检测组合运行。
- 前置坑洼、路面病害、障碍物和“鬼探头”危险模型仍需专项训练。

## 2. 目录结构

```text
ev_ads_cpp/
├── config/
│   ├── ev_ads_runtime.launch.xml     唯一运行配置入口
│   └── scenarios/                    XML 场景配置
├── ros2_ws/src/
│   └── ev_ads_runtime_cpp/           C++ 节点 + ROS 2 消息 + 安装规则
├── models/
│   ├── onnx/                    ONNX 模型放置目录
│   └── rknn/                    RKNN 模型放置目录
├── firmware/                    ESP32-C6 毫米波固件
├── deploy/                      RK3588 部署脚本
└── docs/                        中文文档
```

## 3. 启动

无硬件演示：

```bash
cd /opt/ev_ads/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch ev_ads_runtime_cpp ev_ads_runtime.launch.xml use_fakes:=true
```

真实硬件：

```bash
ros2 launch ev_ads_runtime_cpp ev_ads_runtime.launch.xml \
  use_fakes:=false \
  perception_mode:=scripted \
  imu_driver:=i2c \
  mmwave_mode:=ble
```

模型模式：

```bash
ros2 launch ev_ads_runtime_cpp ev_ads_runtime.launch.xml \
  use_fakes:=false \
  perception_mode:=model \
  rear_model_path:=/opt/ev_ads/models/onnx/rear_yolo.onnx \
  driver_model_path:=/opt/ev_ads/models/onnx/driver_dms_yolo.onnx \
  driver_face_model_path:=/opt/ev_ads/models/onnx/driver_face_yunet.onnx
```

## 4. 主要节点

| 节点 | 说明 |
|---|---|
| `camera_capture_node` | 三路摄像头采集或 fake 图像 |
| `imu_motion_node` | fake/UART/I2C IMU，输出车体姿态与运动事件 |
| `front_risk_node` | 前向风险接口，当前为 scripted/idle/model 占位 |
| `rear_blind_spot_node` | 后置鱼眼靠近识别，支持 YOLO ONNX + OpenCV DNN |
| `driver_attention_node` | 驾驶员疲劳/分心监测，支持 YuNet 人脸 + DMS YOLO ONNX + OpenCV DNN |
| `risk_fusion_node` | 多模态风险融合、告警与制动门控 |
| `mmwave_vital_node` | fake/jsonl/ble 毫米波入口，BLE 后端待接 BlueZ D-Bus |
| `terminal_hmi_node` | 终端 HMI |
| `event_recorder_node` | SQLite/WAL 事件记录，保留 JSONL 兼容后端 |

## 5. 文档

- 交付报告：`docs/ev_ads_cpp_completion_report.md`
- 模型说明：`models/README.md`
- 模型选型：`docs/model_selection_and_onnx_plan.md`
- 部署说明：`docs/deploy_rk3588.md`
- 测试计划：`docs/test_plan.md`
- 优化建议：`docs/optimization_suggestions.md`

## 6. Mac 本机测试

核心算法、事件存储、模型加载和项目配置可在 Mac 上不依赖 ROS 直接测试。统一从项目根目录执行：

```bash
cmake -S . -B build/mac
cmake --build build/mac
ctest --test-dir build/mac --output-on-failure
```

当前根级测试包含：

- `risk_math_and_fusion`：枚举边界、风险数学、融合 L3 门控。
- `event_store`：SQLite/WAL 批量写入和 JSON 工具。
- `model_loading`：YuNet、DMS YOLO、后置 YOLO 的 OpenCV 加载与空白图推理。
- `project_checks`：XML-only 配置、模型 hash、launch 参数、SQLite 配置、非测试 Python 清理和文档口径。
