# C++ Runtime 摄像头参数

运行配置入口：`config/ev_ads_runtime.launch.xml`

## 1. by-id 路径

在 RK3588 上查看摄像头稳定路径：

```bash
ls -l /dev/v4l/by-id/
```

把实际路径写到 launch 参数，或启动时覆盖：

```bash
ros2 launch ev_ads_runtime_cpp ev_ads_runtime.launch.xml \
  use_fakes:=false \
  front_camera_device:=/dev/v4l/by-id/... \
  rear_camera_device:=/dev/v4l/by-id/... \
  driver_camera_device:=/dev/v4l/by-id/...
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
