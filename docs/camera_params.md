# C++ Runtime 摄像头参数

运行配置入口：`config/ev_ads_runtime.launch.xml`

## 1. 稳定设备路径

项目按 RK3588 上实际识别到的三路 UVC 摄像头配置了 udev 固定别名：

| 用途 | dmesg 名称 | VID:PID | 固定路径 |
|---|---|---|---|
| 前置摄像头 | `ocal4` | `1bcf:28c5` | `/dev/ev_ads/front_camera` |
| 后置鱼眼摄像头 | `A68-1600W` | `1bcf:2281` | `/dev/ev_ads/rear_fisheye` |
| 驾驶员人脸摄像头 | `WebCamera` | `32e6:9221` | `/dev/ev_ads/driver_face` |

安装或刷新规则：

```bash
cd /home/elf/Documents/ev_ads_cpp
sudo cp deploy/udev/99-ev-ads-cameras.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger --subsystem-match=video4linux
ls -l /dev/ev_ads
```

也可以查看系统原始路径：

```bash
ls -l /dev/v4l/by-id/
v4l2-ctl --list-devices
```

启动时默认使用 `/dev/ev_ads/*`。如需临时覆盖：

```bash
ros2 launch ev_ads_runtime_cpp ev_ads_runtime.launch.xml \
  use_fakes:=false \
  front_camera_device:=/dev/ev_ads/front_camera \
  rear_camera_device:=/dev/ev_ads/rear_fisheye \
  driver_camera_device:=/dev/ev_ads/driver_face
```

## 2. 推荐参数

| 摄像头 | 推荐分辨率 | FPS | 格式 | 用途 |
|---|---:|---:|---|---|
| front | 1280x720 或 1920x1080 | 30 | MJPG | 前向车辆/行人/坑洼/鬼探头 |
| rear | 1280x720 或 1920x1080 | 25-30 | MJPG | 后向鱼眼靠近预警 |
| driver | 1280x720 | 25 | MJPG | 疲劳/分心监测 |

C++ `camera_capture_node` 默认发布 `/camera/<name>/image_raw/compressed` 和 `/camera/<name>/health`。

## 3. 验证

```bash
ros2 launch ev_ads_runtime_cpp ev_ads_runtime.launch.xml use_fakes:=false perception_mode:=idle
ros2 topic hz /camera/front/image_raw/compressed
ros2 topic echo /camera/front/health
v4l2-ctl --device=/dev/ev_ads/front_camera --stream-mmap --stream-count=30
v4l2-ctl --device=/dev/ev_ads/rear_fisheye --stream-mmap --stream-count=30
v4l2-ctl --device=/dev/ev_ads/driver_face --stream-mmap --stream-count=30
```

## 4. 后置鱼眼标定

后置节点支持两类标定：鱼眼去畸变标定和检测框估距标定。

鱼眼去畸变：

- 配置入口：`config/ev_ads_runtime.launch.xml`
- `fisheye_undistort: true` 开启校正。
- `fisheye_k: [fx, fy, cx, cy]` 填入相机内参。
- `fisheye_d: [k1, k2, k3, k4]` 填入 OpenCV fisheye 畸变参数。
- `fisheye_balance` 越大保留边缘越多，黑边和拉伸也会更多。
- 未做棋盘格/圆点板标定前，不要用占位参数打开校正。

OpenCV 标定建议：

```text
采集 20-40 张不同角度棋盘格图像
使用 cv::fisheye::calibrate 求 K/D
把 K/D 写入 `config/ev_ads_runtime.launch.xml` 中 `rear_blind_spot` 的 `fisheye_k/fisheye_d`
打开 fisheye_undistort 后检查画面边缘直线是否明显改善
```

检测框估距：

- 调整 `rear_blind_spot_node` 参数 `distance_focal_px`。
- 记录 1m、2m、3m、5m、8m 目标框高度。
- 验证 `/perception/blind_spot` 中 `*_distance` 和 `*_closing`。

注意：开启去畸变后，必须重新标定 `distance_focal_px`，因为目标框高度会变化。后续更准确版本应引入 ROI、目标底边落点、跟踪器或 RKNN 深度模型。
