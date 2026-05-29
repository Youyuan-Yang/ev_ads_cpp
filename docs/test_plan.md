# EV-ADS C++ 版本测试计划

## 1. 本机静态检查

确认项目内没有非测试 Python 文件：

```bash
find /Users/yuanyi/Documents/RK3588_Project/ev_ads_cpp \
  -path '*/build/*' -prune -o \
  -path '*/install/*' -prune -o \
  -path '*/log/*' -prune -o \
  -name '*.py' -type f -print
```

确认 ROS 包收敛：

```bash
find ros2_ws/src -maxdepth 2 -name package.xml -print
```

期望只看到：

```text
ev_ads_bringup
ev_ads_interfaces
ev_ads_runtime_cpp
```

## 2. RK3588 构建测试

```bash
cd /opt/ev_ads/ros2_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
colcon test --event-handlers console_direct+
```

## 2.1 Mac 核心测试

该目录不依赖 ROS，可在 Mac 上直接验证核心算法和事件存储：

```bash
cmake -S test -B test/build
cmake --build test/build
ctest --test-dir test/build --output-on-failure
```

## 3. 无硬件冒烟测试

```bash
source install/setup.bash
ros2 launch ev_ads_bringup ev_ads_cpp_runtime.launch.xml use_fakes:=true
```

检查：

```bash
ros2 topic hz /camera/front/image_raw/compressed
ros2 topic hz /camera/rear/image_raw/compressed
ros2 topic hz /camera/driver/image_raw/compressed
ros2 topic hz /vehicle/motion
ros2 topic hz /sensor/mmwave/vital
ros2 topic echo /decision/risk_state
```

## 4. 真实硬件测试

```bash
ros2 launch ev_ads_bringup ev_ads_cpp_runtime.launch.xml \
  use_fakes:=false \
  perception_mode:=scripted \
  imu_driver:=i2c \
  mmwave_mode:=ble
```

验收：

- 三路摄像头 topic 有频率。
- IMU 频率接近配置值。
- 毫米波 BLE 未实现前必须 health 降级，不能输出假有效状态。
- 摄像头断开后，对应感知 health 降级。
- CPU、温度、内存 30 分钟稳定。

## 5. 模型测试

```bash
ros2 launch ev_ads_bringup ev_ads_cpp_runtime.launch.xml \
  use_fakes:=false \
  perception_mode:=model \
  rear_model_path:=/opt/ev_ads/models/onnx/rear_yolo.onnx \
  driver_model_path:=/opt/ev_ads/models/onnx/driver_dms_yolo.onnx \
  driver_face_model_path:=/opt/ev_ads/models/onnx/driver_face_yunet.onnx
```

验收：

- 模型路径为空或错误时 health 为 ERROR，不继续输出旧风险。
- 后置目标接近时 `zone_*` 从 `present` 变为 `approaching`。
- DMS 类别 ID 正确后，闭眼、半闭眼、打哈欠、手机应反映到 `DriverState`；人脸连续消失应在缓冲时间后反映为驾驶员风险。
- 前置模型上线前，`front_node_cpp` 不应把通用模型误当成坑洼/鬼探头模型。

## 6. 日志检查

默认事件库：

```text
/tmp/ev_ads/events.sqlite
```

检查最近事件：

```bash
sqlite3 /tmp/ev_ads/events.sqlite "select t,type,payload from events order by id desc limit 20;"
```

兼容 JSONL：

```bash
ros2 launch ev_ads_bringup ev_ads_cpp_runtime.launch.xml \
  event_storage_backend:=jsonl \
  event_log_path:=/tmp/ev_ads/events.jsonl
```
