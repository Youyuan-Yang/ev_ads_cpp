# RK3588 部署手册

生成日期：2026-05-29

## 1. 目标

该项目在 RK3588 上运行 C++ 链路：

- 三路摄像头、IMU、前向感知、后向感知、驾驶员监测、融合决策。
- C++ 终端 HMI 与 SQLite/WAL 事件记录，JSONL 仅保留兼容后端。
- C++ 毫米波入口，当前 fake/jsonl 可用，BLE 后端待接 BlueZ D-Bus。

## 2. 系统依赖

```bash
sudo apt update
sudo apt install -y \
  ros-humble-ros-base \
  ros-humble-cv-bridge \
  ros-humble-rosbag2 \
  ros-humble-rosbag2-transport \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-vcstool \
  libopencv-dev \
  build-essential \
  cmake \
  sqlite3 \
  libsqlite3-dev \
  i2c-tools \
  v4l-utils \
  bluez \
  libbluetooth-dev
```

说明：这里的 `python3-*` 是 ROS 2 构建工具依赖，不是该项目的 Python 运行包。

## 3. 构建

```bash
cd /opt/ev_ads/ros2_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

## 4. 配置

| 内容 | 文件 |
|---|---|
| 摄像头 | `ros2_ws/src/ev_ads_runtime_cpp/config/cameras.yaml` |
| IMU | `ros2_ws/src/ev_ads_runtime_cpp/config/imu.yaml` |
| 融合 | `ros2_ws/src/ev_ads_runtime_cpp/config/fusion_*.yaml` |
| ONNX 模型 | `models/onnx/` |
| RKNN 模型 | `models/rknn/` |

## 5. 启动

无硬件：

```bash
ros2 launch ev_ads_bringup ev_ads_cpp_runtime.launch.xml use_fakes:=true
```

真实硬件：

```bash
ros2 launch ev_ads_bringup ev_ads_cpp_runtime.launch.xml \
  use_fakes:=false \
  perception_mode:=scripted \
  imu_driver:=i2c \
  mmwave_mode:=ble
```

模型模式：

```bash
ros2 launch ev_ads_bringup ev_ads_cpp_runtime.launch.xml \
  use_fakes:=false \
  perception_mode:=model \
  rear_model_path:=/opt/ev_ads/models/onnx/rear_yolo.onnx \
  driver_model_path:=/opt/ev_ads/models/onnx/driver_dms_yolo.onnx \
  driver_face_model_path:=/opt/ev_ads/models/onnx/driver_face_yunet.onnx \
  event_storage_backend:=sqlite \
  event_log_path:=/tmp/ev_ads/events.sqlite
```

真实制动默认不要放入开机自启。仅低速台架验证时手动加：

```bash
enable_real_brake:=true
```

## 6. 板端验收

- 三路 `/camera/*/image_raw/compressed` 有频率。
- `/vehicle/motion` 有频率。
- `/sensor/mmwave/vital` 有频率；BLE 未接通时应显示 `DISCONNECTED`，不能误报为有效生命体征。
- `/perception/front_risk`、`/perception/blind_spot`、`/perception/driver_state` 有输出。
- `/decision/risk_state`、`/decision/warning_cmd` 稳定输出。
- 模型路径为空或模型加载失败时，节点必须 health 降级。
- CPU、温度、内存连续 30 分钟稳定。
