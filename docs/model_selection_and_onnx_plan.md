# 模型选型与 ONNX 接入方案

生成日期：2026-05-29  
项目：`/Users/yuanyi/Documents/RK3588_Project/ev_ads_cpp`

## 1. 总结

- 后置鱼眼：可以先用轻量 COCO YOLO ONNX 落地靠近识别。
- 驾驶员摄像头：已采用 YuNet 人脸检测 + SafeDrive DMS YOLO 组合，二者均以 ONNX 方式接入 OpenCV。
- 前置摄像头：坑洼、路面破损、障碍物和“鬼探头”需要专项训练与时序风险模型。
- 当前已放入后置 `rear_yolo.onnx`、驾驶员 `driver_face_yunet.onnx`、驾驶员 `driver_dms_yolo.onnx`；前置危险模型仍待训练。

## 2. 后置鱼眼

第一版已使用官方 Ultralytics YOLO11n 导出的 COCO 预训练轻量模型：

```text
models/onnx/rear_yolo.onnx
```

关注类别：

```text
0 person
1 bicycle
2 car
3 motorcycle
5 bus
7 truck
```

运行节点：`rear_node_cpp`。  
核心逻辑：目标检测 → 左/中/右三区 → 框宽/框高估距 → 连续帧距离变化估算靠近速度 → 输出 `BlindSpotState`。

启动：

```bash
ros2 launch ev_ads_bringup ev_ads_cpp_runtime.launch.xml \
  use_fakes:=false \
  perception_mode:=model \
  rear_model_path:=/opt/ev_ads/models/onnx/rear_yolo.onnx
```

后置参数：

| 参数 | 默认值 | 说明 |
|---|---:|---|
| `model_input_width` / `model_input_height` | `640 / 640` | ONNX 输入尺寸 |
| `model_confidence_threshold` | `0.35` | 检测置信度阈值 |
| `model_nms_threshold` | `0.45` | NMS 阈值 |
| `model_class_ids` | `[0,1,2,3,5,7]` | 关注类别 |
| `distance_focal_px` | `700.0` | 估距等效焦距，必须实车标定 |
| `present_max_m` | `10.0` | 近距离关注阈值 |
| `approaching_speed_mps` | `1.0` | 靠近速度阈值 |

鱼眼处理：

- 后置节点已支持 OpenCV fisheye 去畸变。
- 配置文件：`ros2_ws/src/ev_ads_runtime_cpp/config/rear_fisheye.yaml`。
- 参数：`fisheye_undistort`、`fisheye_k=[fx,fy,cx,cy]`、`fisheye_d=[k1,k2,k3,k4]`、`fisheye_balance`、`fisheye_fov_scale`。
- 未填真实标定参数时不要打开去畸变；错误内参会让检测和估距都变差。

## 3. 驾驶员监测

目标文件：

```text
models/onnx/driver_face_yunet.onnx
models/onnx/driver_dms_yolo.onnx
```

组合方案：

```text
YuNet 人脸检测：face box + 5 点关键点，用于脸是否可见、脸框偏移和粗略注意力。
SafeDrive DMS YOLO：eye_open / eye_half / eye_closed / mouth_open / mouth_closed / phone / cigarette / seatbelt_on / seatbelt_off。
```

运行节点：`driver_monitor_node_cpp`。  
输出：`face_visible`、`eye_closure_ratio`、`head_pitch`、`head_yaw`、`distraction_ratio`、`fatigue_score`。

默认类别参数：

| 参数 | 默认值 |
|---|---:|
| `face_class_ids` | `[]` |
| `open_eye_class_ids` | `[0]` |
| `half_eye_class_ids` | `[1]` |
| `closed_eye_class_ids` | `[2]` |
| `yawn_class_ids` | `[3]` |
| `phone_class_ids` | `[5]` |
| `distracted_class_ids` | `[6]` |
| `fatigue_class_ids` | `[]` |

启动：

```bash
ros2 launch ev_ads_bringup ev_ads_cpp_runtime.launch.xml \
  use_fakes:=false \
  perception_mode:=model \
  driver_face_model_path:=/opt/ev_ads/models/onnx/driver_face_yunet.onnx \
  driver_model_path:=/opt/ev_ads/models/onnx/driver_dms_yolo.onnx
```

来源与校验：

| 文件 | 来源 | 许可证 | SHA256 |
|---|---|---|---|
| `driver_face_yunet.onnx` | OpenCV Zoo YuNet | Apache 2.0 | `8f2383e4dd3cfbb4553ea8718107fc0423210dc964f9f4280604804ed2552fa4` |
| `driver_dms_yolo.onnx` | SafeDrive `yolo_safedrive.pt` 导出 | MIT | `cca6dffd84d0596e8ca620dd7ba91dd2624b29fde740bc76d420a5f74e908187` |

注意：单帧 YOLO 不能直接等同于疲劳结论。正式版本需要加入 30-60 秒 PERCLOS 窗口、打哈欠频率、头姿持续偏离、人脸持续消失和夜间补光场景测试。

## 4. 前置危险模型

前置目标：

- 地面坑洼、破损、积水、井盖、减速带。
- 车辆、行人、自行车、电动车、路障。
- 路侧遮挡后突然出现的横穿目标。

建议模型分层：

```text
检测模型：车辆 / 行人 / 障碍物 / 坑洼 / 路面病害
跟踪模型：目标 ID / 速度 / 横向运动 / 遮挡来源
风险模型：TTC / 横向侵入时间 / 遮挡区概率 / 车速与车道位置
```

“鬼探头”不是单帧检测类别，而是时序危险事件。判定条件应至少包含遮挡区、突然出现、横向速度、进入本车路径、TTC 快速下降。

## 5. 当前接入状态

已完成：

- `yolo_onnx.hpp/cpp`：OpenCV DNN 加载 ONNX。
- `rear_node_cpp`：后置 ONNX 推理接口。
- `driver_monitor_node_cpp`：YuNet + DMS YOLO ONNX 组合推理接口。
- `ev_ads_cpp_runtime.launch.xml`：模型路径启动参数。
- `models/README.md`：模型放置规范。
- `models/onnx/rear_yolo.onnx`：由官方 `yolo11n.pt` 导出并完成本机 OpenCV DNN 加载测试。
- `yolo_onnx.cpp`：已兼容端到端输出格式。
- `rear_node_cpp`：已加入鱼眼去畸变入口。

未完成：

- `front_road_hazard.onnx` 未训练，需采集两轮车视角数据。
- RKNN/NPU 后端未接入，当前 ONNX 仅是功能 fallback。
