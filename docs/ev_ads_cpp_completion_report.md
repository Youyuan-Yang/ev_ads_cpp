# EV-ADS C++ 版本交付报告

生成日期：2026-05-29  
项目路径：`/Users/yuanyi/Documents/RK3588_Project/ev_ads_cpp`

## 1. 交付结论

该项目已收敛为 C++ 运行版本：除测试外，项目内不再保留 Python 运行包、Python launch、Python 工具或桌面应用。原版 Python 项目的修改仍保留在 `/Users/yuanyi/Documents/RK3588_Project/ev_ads`，本副本面向 RK3588 C++ 部署。

8GB RK3588 可以跑通 fake/scripted/真实传感器基础链路。后置和驾驶员监测已具备 ONNX + OpenCV DNN 功能验证路径；量产性能建议再转 RKNN/NPU。

## 2. 当前 ROS 包

| 包 | 语言 | 说明 |
|---|---|---|
| `ev_ads_runtime_cpp` | C++17/IDL/CMake | 摄像头、IMU、感知、DMS、融合、HMI、事件记录、毫米波入口和 ROS 2 消息 |

已删除旧 Python 运行包和拆散的 ROS2 包：`ev_ads_fakes`、`ev_ads_hmi`、`ev_ads_recorder`、`ev_ads_sensor_mmwave_ble`、`ev_ads_bringup`、`ev_ads_interfaces`，以及 `apps/`、`tools/`。

## 3. C++ 架构

```text
ev_ads_cpp/
├── config/
│   ├── ev_ads_runtime.launch.xml
│   └── scenarios/
│       ├── front_pedestrian_emergency.xml
│       ├── driver_drowsy.xml
│       └── ...
└── ros2_ws/src/ev_ads_runtime_cpp/
    ├── msg/
    │   ├── FrontRisk.msg
    │   ├── RiskState.msg
    │   └── ...
    ├── include/ev_ads_runtime_cpp/
│   ├── risk_math.hpp
│   ├── domain_types.hpp
│   ├── topics.hpp
│   ├── runtime_config.hpp
│   ├── risk_fusion_core.hpp
│   ├── event_store.hpp
│   └── onnx_yolo_detector.hpp
    └── src/
    ├── camera_capture_node.cpp
    ├── imu_motion_node.cpp
    ├── front_risk_node.cpp
    ├── rear_blind_spot_node.cpp
    ├── driver_attention_node.cpp
    ├── risk_fusion_node.cpp
    ├── mmwave_vital_node.cpp
    ├── terminal_hmi_node.cpp
    ├── event_recorder_node.cpp
    └── onnx_yolo_detector.cpp
```

数据流：

```text
三路摄像头 + IMU + 毫米波
        ↓
前向风险 / 后向靠近 / 驾驶员状态 / 车辆运动 / 生命体征
        ↓
risk_fusion_node 多模态融合
        ↓
RiskState + WarningCommand + BrakeCommand
        ↓
HMI 显示 + SQLite/WAL 事件日志
```

## 4. 多模态融合模型

融合节点采用 v2 模型：

```text
R = clamp(α * R_weighted + β * R_prob + R_synergy, 0, 1)
```

- `R_weighted`：按传感器健康状态进行可靠度加权。
- `R_prob`：Noisy-OR 聚合多源风险证据。
- `R_synergy`：前向+DMS、前向+IMU、后向+IMU、DMS+毫米波等协同项。
- 前向极短 TTC 可设置风险下限，避免危险被平均掉。
- L3 必须由前向 TTC 主导，毫米波不能单独触发 L2/L3。
- stale/disconnected 传感器默认置零，避免旧数据继续推高风险。

## 5. 模型状态

`models/onnx/rear_yolo.onnx` 已放入，用于后置鱼眼第一版靠近识别：

- 官方权重：`https://github.com/ultralytics/assets/releases/download/v8.3.0/yolo11n.pt`
- 导出命令：`yolo export model=/private/tmp/yolo11n.pt format=onnx imgsz=640 opset=12 simplify=False nms=False`
- 文件大小：约 10 MB
- SHA256：`4e16b0662b3d9ceff65bc7ff79fca909f62673dc9e08aa74ee4a5e5e1511cf5d`
- Homebrew OpenCV 4.13.0 已验证可加载，输出层为 `output0`
- 项目检测器空白图推理成功，检测数量为 0
- 本机原有 `yolo26n.onnx` 已备份为 `models/onnx/rear_yolo_local_yolo26n.onnx`

驾驶员监测采用组合模型，均已放入：

| 文件 | 用途 | 来源 | 许可证 | SHA256 |
|---|---|---|---|---|
| `models/onnx/driver_face_yunet.onnx` | 人脸检测、脸框偏移、脸消失判断 | OpenCV Zoo YuNet | Apache 2.0 | `8f2383e4dd3cfbb4553ea8718107fc0423210dc964f9f4280604804ed2552fa4` |
| `models/onnx/driver_dms_yolo.onnx` | 闭眼、半闭眼、张嘴、手机、吸烟等 DMS 证据 | SafeDrive `yolo_safedrive.pt` 导出 | MIT | `cca6dffd84d0596e8ca620dd7ba91dd2624b29fde740bc76d420a5f74e908187` |

驾驶员节点已改为 YuNet + DMS YOLO 组合推理：

- YuNet 负责人脸可见、脸框位置和短时漏检缓冲。
- SafeDrive DMS YOLO 负责 `eye_open/eye_half/eye_closed/mouth_open/mouth_closed/phone/cigarette/seatbelt_on/seatbelt_off`。
- 默认类别映射写入根目录 `config/ev_ads_runtime.launch.xml` 的 `driver_attention` 参数。
- 人脸连续消失超过 `face_absence_warning_ms` 后按分心/离岗风险处理。

仍未补齐：

- 前置坑洼、路面病害、障碍物和复杂“鬼探头”预警需要专项训练与时序模型，不能靠一个通用模型冒充完成。
- RKNN/NPU 版本尚未转换。

模型放置规范：

```text
models/onnx/rear_yolo.onnx
models/onnx/driver_face_yunet.onnx
models/onnx/driver_dms_yolo.onnx
models/onnx/front_road_hazard.onnx
models/rknn/rear_yolo.rknn
models/rknn/driver_face_yunet.rknn
models/rknn/driver_dms_yolo.rknn
models/rknn/front_road_hazard.rknn
```

## 6. 启动方式

无硬件演示：

```bash
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

## 7. 关键操作记录

- 新增 `mmwave_vital_node`：替代 fake mmWave，并保留 `jsonl/ble` 入口。
- 新增 `terminal_hmi_node`：替代 Python 终端 HMI。
- 新增 `event_recorder_node`：替代 Python 事件日志；当前默认 SQLite/WAL 批量写入，`jsonl` 仅作为兼容后端。
- 新增 `domain_types.hpp`：统一 `Health`、`WarningLevel`、`FaceVisibility`、`ObjectClass`、`ZoneState`、`MotionFlag` 等 enum class，ROS `uint8` 只在消息边界转换。
- 新增 `topics.hpp` 与 `runtime_config.hpp`：集中话题名和节点配置，减少节点内硬编码。
- 新增 `risk_fusion_core.hpp`：将融合算法从 ROS 节点抽离为可单测核心类，`risk_fusion_node` 只负责 ROS 消息桥接。
- 新增 `event_store.hpp/cpp`：封装 SQLite/JSONL 事件存储，默认 WAL + 批量提交，降低 JSONL 高频 flush 性能风险。
- 重构根目录级 `CMakeLists.txt`：现在根 CMake 是 EV-ADS runtime 总入口，默认通过 `ros2_workspace_build` target 调用 colcon 构建真实 `ev_ads_runtime_cpp` ROS2 包，并提供 `run_ev_ads_fake`、`run_ev_ads_hardware` target；GoogleTest 仅作为 `EV_ADS_BUILD_TESTS` 可选测试链路。
- 新增 `test/`：根 CMake 在启用测试时通过 `FetchContent` 自动下载 GoogleTest，Mac 上从项目根目录统一编译测试 `domain_types/risk_math`、`FusionCore`、`EventStore`、ONNX 模型加载和项目配置。
- 将旧 Python launch 改为 XML launch。
- 将运行参数统一收敛到根目录 `config/ev_ads_runtime.launch.xml`，删除项目自有 YAML/TOML 配置，场景文件统一放入 `config/scenarios/`。
- 删除非测试 Python 运行包、`apps/`、`tools/`。
- 下载官方 Ultralytics `yolo11n.pt`，在 `/private/tmp` 临时环境导出 ONNX，并放入 `models/onnx/rear_yolo.onnx`。
- 备份本机原有 `yolo26n.onnx` 到 `models/onnx/rear_yolo_local_yolo26n.onnx`。
- 修正 `onnx_yolo_detector.cpp`：兼容 `x1,y1,x2,y2,score,class_id` 这种端到端 ONNX 输出格式。
- 新增后置鱼眼去畸变：`rear_blind_spot_node` 支持 OpenCV fisheye `K/D` 参数，参数统一写入 `ev_ads_runtime.launch.xml`。
- 经用户确认后，下载 OpenCV Zoo YuNet 人脸 ONNX 到临时目录，验证 `FaceDetectorYN` 可创建后复制到 `models/onnx/driver_face_yunet.onnx`。
- 经用户确认后，从 SafeDrive 模型仓库下载 `yolo_safedrive.pt`。原始 Hugging Face 域名在本机网络超时，因此使用 `hf-mirror.com` 获取同名文件；随后在 `/private/tmp` 临时环境导出 ONNX 并复制到 `models/onnx/driver_dms_yolo.onnx`。
- 修改 `driver_attention_node`：新增 YuNet 人脸检测、DMS YOLO 类别映射、半闭眼证据、人脸连续消失缓冲和组合风险输出。
- 将 SafeDrive 类别 ID、YuNet 阈值和人脸消失风险参数写入 `ev_ads_runtime.launch.xml`。
- 记录模型来源、hash 和本机 OpenCV 加载测试结果。
- 将原 `assert + main` 测试改为 GoogleTest `TEST(...)` 写法，并通过 `gtest_discover_tests` 接入 CTest。

## 8. 自检结果

本机可完成的自检：

- 非构建目录下已无 `.py` 文件。
- ROS 包只剩 `ev_ads_runtime_cpp` 一个包。
- 启动配置只剩根目录 `config/ev_ads_runtime.launch.xml`。
- 文档口径已改为 C++ 版本。
- 后置 ONNX 已通过本机 OpenCV DNN 加载测试。
- YuNet 人脸 ONNX 已通过本机 OpenCV `FaceDetectorYN` 创建和空白图检测测试。
- DMS YOLO ONNX 已通过本机 OpenCV DNN 加载测试，项目检测器空白图推理成功，检测数量为 0。
- XML launch 和 XML 场景文件已通过解析检查。
- 项目自有运行配置已统一为 XML，不再保留 YAML/TOML 配置文件。
- Mac 本机根目录级 CMake/CTest 已通过：
  - `DomainTypeConversion.*`、`RiskMath.*`、`FusionCore.*`
  - `EventStore.*`、`EventStoreJson.*`
  - `ModelLoading.*`
  - `project_checks`

统一命令：

```bash
cmake -S . -B build/mac -DEV_ADS_BUILD_ROS2_NATIVE=OFF -DEV_ADS_BUILD_TESTS=ON
cmake --build build/mac
ctest --test-dir build/mac --output-on-failure
```

本机限制：

- 当前 Mac 环境没有 ROS 2/colcon，不能完整构建工作区。
- 当前 Mac 环境没有 ROS 2/colcon，不能启动完整 ROS 链路。
- RKNN 转换和 NPU 性能验证必须在合适的 Linux/RKNN 工具链中完成。
