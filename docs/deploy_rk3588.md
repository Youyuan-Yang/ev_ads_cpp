# RK3588 部署与运行手册

生成日期：2026-05-31

适用对象：Rockchip RK3588，Ubuntu 22.04 arm64，ROS 2 Humble  
项目根目录：`/home/elf/Documents/ev_ads_cpp`  
运行版本：C++ ROS2 单包版本，包名 `ev_ads_runtime_cpp`

---

## 0. 总览：第一次部署按这个顺序做

```text
§1  硬件和接线确认
§2  系统、网络、时间同步
§3  安装 ROS2 和系统依赖
§4  权限、蓝牙、I2C、摄像头检查
§5  模型文件和运行配置确认
§6  构建 C++ ROS2 工作区
§7  启动 fake / 真实硬件 / 模型模式
§8  systemd 开机自启
§9  话题、日志、性能检查
§10 常见报错和修复
§11 升级和清理重建
§12 部署完成清单
```

---

## 1. 硬件和接线确认

| 设备 | 必须 | 接口 | 备注 |
|---|---:|---|---|
| RK3588 开发板，8GB 内存 | 是 | - | 建议主动散热 |
| Ubuntu 22.04 arm64 系统盘 | 是 | eMMC / SD | 至少 32GB |
| 前置摄像头 | 是 | USB / UVC | 车辆、行人、障碍物、坑洼、鬼探头预警 |
| 后置鱼眼摄像头 | 是 | USB / UVC | 靠近识别和靠近速度预警 |
| 驾驶员摄像头 | 是 | USB / UVC | 疲劳、分心、人脸可见性 |
| IMU | 是 | I2C / UART | 注意 3.3V 电平和共地 |
| USB Hub | 推荐 | USB | 三路摄像头建议独立供电 |
| 蓝牙 | 可选 | 板载或 USB dongle | C++ BLE 后端仍是预留入口 |
| 蜂鸣器 / 执行器 | 可选 | GPIO / 控制器 | 真实制动默认禁用 |

通电前检查：

1. IMU 电压和逻辑电平，5V TTL 不要直连 RK3588 GPIO。
2. 摄像头尽量使用带独立供电的 USB Hub。
3. 所有外设共地。
4. 真实制动执行器没有完成低速台架验证前，不要打开 `enable_real_brake`。

相关文档：

- `docs/camera_params.md`
- `docs/wiring_imu.md`
- `docs/env_lock.md`

---

## 2. 系统、网络和时间同步

推荐系统：

- Ubuntu 22.04 LTS arm64
- ROS 2 Humble
- 内核需要支持 `v4l2`、`i2c-dev`、`btusb`

设置主机名：

```bash
sudo hostnamectl set-hostname ev-ads-edge
```

开启 SSH：

```bash
sudo apt update
sudo apt install -y openssh-server
sudo systemctl enable --now ssh
ip a
```

时间同步很重要，RTC 不准会导致 ROS topic 时间戳和日志顺序混乱：

```bash
sudo apt install -y chrony ntpdate
sudo systemctl enable --now chrony
timedatectl status
```

如果 `timedatectl` 显示时间明显不对，先联网再强制同步：

```bash
sudo systemctl stop chrony
sudo chronyd -q "server ntp.aliyun.com iburst maxsamples 1"
sudo systemctl start chrony
date
```

---

## 3. 安装 ROS2 和系统依赖

进入项目目录：

```bash
cd /home/elf/Documents/ev_ads_cpp
```

推荐先跑项目脚本：

```bash
cd /home/elf/Documents/ev_ads_cpp/deploy
bash install_deps.sh
```

如果要手动安装，按下面做：

```bash
sudo apt update
sudo apt install -y software-properties-common curl gnupg lsb-release
sudo add-apt-repository -y universe
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | \
  sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt install -y \
  ros-humble-ros-base \
  ros-humble-cv-bridge \
  ros-humble-rosbag2 \
  ros-humble-rosbag2-transport \
  ros-humble-ament-cmake \
  ros-humble-rosidl-default-generators \
  ros-humble-rosidl-default-runtime \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-vcstool \
  build-essential \
  cmake \
  git \
  libopencv-dev \
  sqlite3 \
  libsqlite3-dev \
  i2c-tools \
  v4l-utils \
  usbutils \
  bluez \
  bluez-tools \
  libbluetooth-dev \
  network-manager \
  chrony \
  ntpdate
```

初始化 rosdep：

```bash
sudo rosdep init 2>/dev/null || true
rosdep update
```

让新终端自动加载 ROS2：

```bash
grep -q "source /opt/ros/humble/setup.bash" ~/.bashrc || \
  echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source /opt/ros/humble/setup.bash
echo $ROS_DISTRO
```

期望输出：

```text
humble
```

说明：该项目运行代码是 C++，但 ROS2 的 `colcon/rosdep` 工具本身依赖系统 Python，这是 ROS 工具链依赖，不是项目运行逻辑。

---

## 4. 权限、蓝牙、I2C 和摄像头

把当前用户加入设备组：

```bash
sudo usermod -aG dialout,video,i2c,bluetooth,plugdev $USER
sudo reboot
```

重启后确认：

```bash
groups
```

输出中应包含：

```text
dialout video i2c bluetooth plugdev
```

蓝牙服务：

```bash
sudo systemctl enable --now bluetooth
systemctl status bluetooth --no-pager
bluetoothctl scan on
```

I2C 检查：

```bash
ls /dev/i2c-*
i2cdetect -y 1
```

摄像头稳定路径：

```bash
v4l2-ctl --list-devices
ls -l /dev/v4l/by-id
v4l2-ctl --device=/dev/v4l/by-id/你的摄像头 --list-formats-ext
```

把实际摄像头路径填到启动参数里，或直接改：

```text
config/ev_ads_runtime.launch.xml
```

当前默认参数名：

| 参数 | 用途 |
|---|---|
| `front_camera_device` | 前置摄像头 |
| `rear_camera_device` | 后置鱼眼摄像头 |
| `driver_camera_device` | 驾驶员摄像头 |

---

## 5. 模型文件和配置确认

模型应在：

```text
/home/elf/Documents/ev_ads_cpp/models/onnx/rear_yolo.onnx
/home/elf/Documents/ev_ads_cpp/models/onnx/driver_face_yunet.onnx
/home/elf/Documents/ev_ads_cpp/models/onnx/driver_dms_yolo.onnx
```

检查文件是否存在：

```bash
cd /home/elf/Documents/ev_ads_cpp
ls -lh models/onnx
sha256sum models/onnx/*.onnx
```

当前 launch 配置入口只有一个：

```text
config/ev_ads_runtime.launch.xml
```

不要再新增 YAML/TOML 运行配置，也不要把业务配置放回 `ros2_ws/src`。

重要限制：

- 后置模型是 COCO/YOLO 通用目标检测，只是第一版靠近识别基础，不等同于鱼眼专训模型。
- 驾驶员监测是 YuNet 人脸 + DMS YOLO 组合模型，仍需按实际摄像头位置调阈值。
- 前置坑洼、路面病害、鬼探头预警仍需要专项训练和实车数据。
- `mmwave_mode:=ble` 目前是 C++ BLE 后端预留入口，未接通 BlueZ D-Bus 前不能当成可用生命体征输入。

---

## 6. 构建 C++ ROS2 工作区

推荐从项目根目录统一构建：

```bash
cd /home/elf/Documents/ev_ads_cpp
source /opt/ros/humble/setup.bash
cmake -S . -B build/rk3588 \
  -DEV_ADS_BUILD_ROS2_NATIVE=OFF \
  -DEV_ADS_BUILD_TESTS=OFF \
  -DCMAKE_BUILD_TYPE=Release
cmake --build build/rk3588 --target ros2_workspace_build
source ros2_ws/install/setup.bash
```

也可以直接使用 ROS2 工作区命令：

```bash
cd /home/elf/Documents/ev_ads_cpp/ros2_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

构建后检查包是否能被 ROS2 找到：

```bash
ros2 pkg prefix ev_ads_runtime_cpp
ros2 pkg executables ev_ads_runtime_cpp
```

期望至少能看到这些可执行文件：

```text
camera_capture_node
imu_motion_node
front_risk_node
rear_blind_spot_node
driver_attention_node
risk_fusion_node
terminal_hmi_node
event_recorder_node
mmwave_vital_node
```

检查消息接口：

```bash
ros2 interface show ev_ads_runtime_cpp/msg/RiskState
ros2 interface show ev_ads_runtime_cpp/msg/BlindSpotState
ros2 interface show ev_ads_runtime_cpp/msg/DriverState
```

检查安装后的 launch 文件不是旧版本：

```bash
grep -n 'type="double"\|：' "$(ros2 pkg prefix ev_ads_runtime_cpp)/share/ev_ads_runtime_cpp/launch/ev_ads_runtime.launch.xml"
```

期望没有输出。

---

## 7. 启动和日常运行

每个新终端先执行：

```bash
cd /home/elf/Documents/ev_ads_cpp
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
```

### 7.1 无硬件演示

推荐先跑这个，确认程序链路本身能起来：

```bash
ros2 launch ev_ads_runtime_cpp ev_ads_runtime.launch.xml \
  use_fakes:=true \
  perception_mode:=scripted
```

也可以从根 CMake target 启动：

```bash
cd /home/elf/Documents/ev_ads_cpp
cmake --build build/rk3588 --target run_ev_ads_fake
```

### 7.2 真实摄像头 + scripted 感知

这个模式适合先验证摄像头、IMU、话题和融合，不强行依赖模型推理：

```bash
ros2 launch ev_ads_runtime_cpp ev_ads_runtime.launch.xml \
  use_fakes:=false \
  perception_mode:=scripted \
  imu_driver:=i2c \
  mmwave_mode:=fake \
  front_camera_device:=/dev/v4l/by-id/你的前置摄像头 \
  rear_camera_device:=/dev/v4l/by-id/你的后置鱼眼 \
  driver_camera_device:=/dev/v4l/by-id/你的驾驶员摄像头
```

### 7.3 模型模式

构建安装后，默认模型路径会指向包内安装的 ONNX。也可以显式指定源码目录里的模型：

```bash
ros2 launch ev_ads_runtime_cpp ev_ads_runtime.launch.xml \
  use_fakes:=false \
  perception_mode:=model \
  rear_model_path:=/home/elf/Documents/ev_ads_cpp/models/onnx/rear_yolo.onnx \
  driver_model_path:=/home/elf/Documents/ev_ads_cpp/models/onnx/driver_dms_yolo.onnx \
  driver_face_model_path:=/home/elf/Documents/ev_ads_cpp/models/onnx/driver_face_yunet.onnx \
  event_storage_backend:=sqlite \
  event_log_path:=/tmp/ev_ads/events.sqlite
```

### 7.4 真实硬件 target

```bash
cd /home/elf/Documents/ev_ads_cpp
cmake --build build/rk3588 --target run_ev_ads_hardware
```

该 target 默认使用：

```text
use_fakes:=false perception_mode:=model
```

如果摄像头 by-id、IMU 或模型还没确认，先不要直接跑这个 target，先跑 §7.1 和 §7.2。

### 7.5 真实制动

默认禁止真实制动。只在低速台架，并确认所有安全门后手动加：

```bash
ros2 launch ev_ads_runtime_cpp ev_ads_runtime.launch.xml \
  use_fakes:=false \
  perception_mode:=model \
  enable_real_brake:=true
```

### 7.6 停止

前台运行时按 `Ctrl-C`。systemd 运行时：

```bash
sudo systemctl stop ev-ads.service
```

---

## 8. systemd 开机自启

项目已经提供脚本：

```text
deploy/install_deps.sh
deploy/ev_ads_boot.sh
deploy/systemd/ev-ads.service
```

安装或重新注册服务：

```bash
cd /home/elf/Documents/ev_ads_cpp/deploy
bash install_deps.sh
```

脚本询问是否注册 systemd 时选择 `y`。

查看服务：

```bash
sudo systemctl daemon-reload
sudo systemctl enable ev-ads.service
sudo systemctl start ev-ads.service
systemctl status ev-ads.service --no-pager
tail -f /var/log/ev_ads.log
```

关闭自启动：

```bash
sudo systemctl disable ev-ads.service
sudo systemctl stop ev-ads.service
```

注意：`deploy/ev_ads_boot.sh` 会自动定位项目根目录，所以项目放在 `/home/elf/Documents/ev_ads_cpp` 时无需改脚本路径。

---

## 9. 话题、日志和性能检查

### 9.1 话题清单

```bash
ros2 topic list
```

关键话题：

```text
/camera/front/image_raw/compressed
/camera/rear/image_raw/compressed
/camera/driver/image_raw/compressed
/vehicle/motion
/sensor/mmwave/vital
/perception/front_risk
/perception/blind_spot
/perception/driver_state
/decision/risk_state
/decision/warning_cmd
/decision/brake_cmd
```

### 9.2 频率检查

```bash
ros2 topic hz /camera/front/image_raw/compressed
ros2 topic hz /camera/rear/image_raw/compressed
ros2 topic hz /camera/driver/image_raw/compressed
ros2 topic hz /vehicle/motion
ros2 topic hz /perception/front_risk
ros2 topic hz /decision/risk_state
```

参考期望：

| 话题 | 期望 |
|---|---:|
| `/camera/*/image_raw/compressed` | 20 Hz 以上 |
| `/vehicle/motion` | 80 Hz 以上 |
| `/perception/*` | 8 Hz 以上 |
| `/decision/risk_state` | 15 Hz 以上 |

### 9.3 状态查看

```bash
ros2 topic echo /decision/risk_state --once
ros2 topic echo /decision/warning_cmd --once
ros2 topic echo /decision/brake_cmd --once
```

### 9.4 SQLite 事件库

默认路径：

```text
/tmp/ev_ads/events.sqlite
```

查看最近事件：

```bash
sqlite3 /tmp/ev_ads/events.sqlite \
  "select id,t,type,payload from events order by id desc limit 10;"
```

### 9.5 系统负载

```bash
htop
free -h
vcgencmd measure_temp 2>/dev/null || cat /sys/class/thermal/thermal_zone*/temp
```

如果 CPU 长期过高，先降摄像头分辨率或 FPS，再考虑把 ONNX 转 RKNN。

---

## 10. 常见报错和修复

### 10.1 `ros2 launch` 报 `invalid type identifier 'double'`

原因：安装目录里还是旧的 `ev_ads_runtime.launch.xml`。ROS2 Humble XML launch 浮点类型要写 `type="float"`，不能写 `type="double"`。

修复：

```bash
cd /home/elf/Documents/ev_ads_cpp
cmake --build build/rk3588 --target ros2_workspace_build
source ros2_ws/install/setup.bash
grep -n 'type="double"\|：' "$(ros2 pkg prefix ev_ads_runtime_cpp)/share/ev_ads_runtime_cpp/launch/ev_ads_runtime.launch.xml"
```

最后一条命令应无输出。

### 10.2 `CMakeFiles/ros2_workspace_build` 里出现 `/bin/bash -lc set\ -e`

原因：旧构建目录缓存了错误 Makefile。

修复：

```bash
cd /home/elf/Documents/ev_ads_cpp
rm -rf build/rk3588
cmake -S . -B build/rk3588 \
  -DEV_ADS_BUILD_ROS2_NATIVE=OFF \
  -DEV_ADS_BUILD_TESTS=OFF \
  -DCMAKE_BUILD_TYPE=Release
cmake --build build/rk3588 --target ros2_workspace_build
```

### 10.3 `ros2` 命令找不到包或节点

先确认 source：

```bash
source /opt/ros/humble/setup.bash
source /home/elf/Documents/ev_ads_cpp/ros2_ws/install/setup.bash
ros2 pkg prefix ev_ads_runtime_cpp
ros2 pkg executables ev_ads_runtime_cpp
```

如果仍找不到：

```bash
ros2 daemon stop
ros2 daemon start
```

### 10.4 colcon build 失败

常见处理：

| 现象 | 处理 |
|---|---|
| 找不到 `ament_cmake` | `sudo apt install ros-humble-ament-cmake` |
| 找不到 `rosidl_default_generators` | `sudo apt install ros-humble-rosidl-default-generators` |
| 找不到 OpenCV | `sudo apt install libopencv-dev ros-humble-cv-bridge` |
| 找不到 SQLite | `sudo apt install sqlite3 libsqlite3-dev` |
| rosdep 网络失败 | 先手动 apt 安装上面依赖，再 `rosdep install ... -r -y` |

清理后重建：

```bash
cd /home/elf/Documents/ev_ads_cpp/ros2_ws
rm -rf build install log
source /opt/ros/humble/setup.bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

### 10.5 摄像头打不开

```bash
groups
ls -l /dev/v4l/by-id
v4l2-ctl --list-devices
v4l2-ctl --device=/dev/v4l/by-id/你的摄像头 --stream-mmap --stream-count=30
```

处理：

- 用户不在 `video` 组：执行 §4 并重启。
- `/dev/videoN` 漂移：使用 `/dev/v4l/by-id/...`。
- 三路同时帧率低：换独立供电 USB Hub，降低分辨率或 FPS。
- 摄像头不支持 MJPG：用 `v4l2-ctl --list-formats-ext` 查格式，再调整 launch 参数。

### 10.6 IMU 没输出

```bash
groups
ls /dev/i2c-*
i2cdetect -y 1
ros2 topic echo /vehicle/motion --once
```

处理：

- 用户不在 `i2c` 或 `dialout` 组：执行 §4 并重启。
- I2C 地址不是 `104`：修改 `config/ev_ads_runtime.launch.xml` 的 `addr`。
- 安装方向不一致：参考 `docs/wiring_imu.md` 调整 `mount_rpy_deg`。

### 10.7 模型加载失败

检查文件：

```bash
cd /home/elf/Documents/ev_ads_cpp
ls -lh models/onnx
file models/onnx/*.onnx
```

检查安装：

```bash
ls -lh "$(ros2 pkg prefix ev_ads_runtime_cpp)/share/ev_ads_runtime_cpp/models/onnx"
```

如果安装目录缺模型，重新构建：

```bash
cmake --build build/rk3588 --target ros2_workspace_build
source ros2_ws/install/setup.bash
```

### 10.8 systemd 起不来

```bash
journalctl -u ev-ads.service -n 200 --no-pager
tail -n 200 /var/log/ev_ads.log
systemctl status ev-ads.service --no-pager
```

常见原因：

- 没构建 `ros2_ws/install/setup.bash`。
- systemd 服务里的用户不是 `elf`。
- 用户组权限修改后没有重启。
- 摄像头路径写错，真实硬件模式启动失败。

### 10.9 `mmwave_mode:=ble` 没有真实数据

当前 C++ 节点保留 `ble` 入口，但 BlueZ D-Bus 后端仍待实现。板端联调前请用：

```bash
mmwave_mode:=fake
```

不要把 BLE 暂未实现导致的 `DISCONNECTED` 当成融合算法故障。

---

## 11. 升级、清理和回滚

更新代码后重建：

```bash
cd /home/elf/Documents/ev_ads_cpp
source /opt/ros/humble/setup.bash
cmake --build build/rk3588 --target ros2_workspace_build
source ros2_ws/install/setup.bash
sudo systemctl restart ev-ads.service
```

如果 CMake 或 launch 安装规则改过，建议清理：

```bash
cd /home/elf/Documents/ev_ads_cpp
rm -rf build/rk3588 ros2_ws/build ros2_ws/install ros2_ws/log
cmake -S . -B build/rk3588 \
  -DEV_ADS_BUILD_ROS2_NATIVE=OFF \
  -DEV_ADS_BUILD_TESTS=OFF \
  -DCMAKE_BUILD_TYPE=Release
cmake --build build/rk3588 --target ros2_workspace_build
source ros2_ws/install/setup.bash
```

临时停服务再手动调试：

```bash
sudo systemctl stop ev-ads.service
cd /home/elf/Documents/ev_ads_cpp
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
ros2 launch ev_ads_runtime_cpp ev_ads_runtime.launch.xml use_fakes:=true perception_mode:=scripted
```

---

## 12. 部署完成清单

- RK3588 项目目录是 `/home/elf/Documents/ev_ads_cpp`。
- `echo $ROS_DISTRO` 输出 `humble`。
- 当前用户在 `dialout/video/i2c/bluetooth/plugdev` 组。
- `ros2 pkg prefix ev_ads_runtime_cpp` 能输出安装路径。
- `ros2 pkg executables ev_ads_runtime_cpp` 能列出 9 个 C++ 节点。
- `grep -n 'type="double"\|：' "$(ros2 pkg prefix ev_ads_runtime_cpp)/share/ev_ads_runtime_cpp/launch/ev_ads_runtime.launch.xml"` 无输出。
- `models/onnx` 中 3 个 ONNX 文件存在。
- `use_fakes:=true perception_mode:=scripted` 可以启动。
- 三路摄像头真实路径已替换为 `/dev/v4l/by-id/...`。
- `/decision/risk_state`、`/decision/warning_cmd` 有输出。
- SQLite 事件库 `/tmp/ev_ads/events.sqlite` 能写入。
- 真实制动仍保持禁用，除非低速台架验证完成。

---

## 13. 相关文档

| 文档 | 内容 |
|---|---|
| `README.md` | 项目总览 |
| `docs/test_plan.md` | 测试计划 |
| `docs/camera_params.md` | 摄像头和鱼眼参数 |
| `docs/wiring_imu.md` | IMU 接线和安装姿态 |
| `docs/model_selection_and_onnx_plan.md` | ONNX 模型来源和限制 |
| `models/README.md` | 模型文件说明 |
| `deploy/README.md` | 一次性安装脚本和 systemd |
