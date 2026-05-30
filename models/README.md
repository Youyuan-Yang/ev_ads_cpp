# 模型目录说明

该目录用于放置 RK3588 运行链路需要的模型文件。当前已放入后置鱼眼第一版 ONNX、驾驶员 YuNet 人脸模型和驾驶员 DMS YOLO 模型；前置道路危险和 RKNN 模型仍待补齐。

## 1. 目录约定

```text
models/
├── onnx/
│   ├── rear_yolo.onnx          已放入
│   ├── driver_face_yunet.onnx  已放入
│   ├── driver_dms_yolo.onnx    已放入
│   └── front_road_hazard.onnx
└── rknn/
    ├── rear_yolo.rknn
    ├── driver_face_yunet.rknn
    ├── driver_dms_yolo.rknn
    └── front_road_hazard.rknn
```

## 2. 当前模型状态

- 后置鱼眼默认模型：`models/onnx/rear_yolo.onnx`，由官方 Ultralytics `yolo11n.pt` 导出。
- 官方权重来源：`https://github.com/ultralytics/assets/releases/download/v8.3.0/yolo11n.pt`。
- 导出命令：`yolo export model=/private/tmp/yolo11n.pt format=onnx imgsz=640 opset=12 simplify=False nms=False`。
- `rear_yolo.onnx` 文件大小：约 10 MB。
- `rear_yolo.onnx` SHA256：`4e16b0662b3d9ceff65bc7ff79fca909f62673dc9e08aa74ee4a5e5e1511cf5d`。
- 备份模型：`models/onnx/rear_yolo_local_yolo26n.onnx`，来源为 `/Users/yuanyi/Documents/Y_L_M_Project/RideDetector/yolo26n.onnx`。
- 备份模型 SHA256：`0faa933b20bd457fffa7bb8284fd4ddf55e91c2a768c0af7083a409e70557da9`。
- Homebrew OpenCV 4.13.0 可成功加载默认模型，输出层为 `output0`。
- 项目检测器已验证空白图推理结果为 0 个检测。
- 项目 `onnx_yolo_detector.cpp` 兼容标准 YOLO 输出，也兼容备份模型的 `x1,y1,x2,y2,score,class_id` 输出格式。
- 驾驶员人脸模型：`models/onnx/driver_face_yunet.onnx`，来源为 OpenCV Zoo YuNet，Apache 2.0，用于人脸可见、脸框位置和粗略注意力判断。
- `driver_face_yunet.onnx` SHA256：`8f2383e4dd3cfbb4553ea8718107fc0423210dc964f9f4280604804ed2552fa4`。
- 驾驶员 DMS 模型：`models/onnx/driver_dms_yolo.onnx`，来源为 SafeDrive `yolo_safedrive.pt` 导出，MIT 许可证，类别为 `eye_open/eye_half/eye_closed/mouth_open/mouth_closed/phone/cigarette/seatbelt_on/seatbelt_off`。
- `driver_dms_yolo.onnx` SHA256：`cca6dffd84d0596e8ca620dd7ba91dd2624b29fde740bc76d420a5f74e908187`。
- DMS 导出命令：`yolo export model=/private/tmp/yolo_safedrive.pt format=onnx imgsz=640 opset=12 simplify=False nms=False`。
- 两个驾驶员模型均已通过本机 Homebrew OpenCV 4.13.0 加载测试；DMS ONNX 也已通过项目检测器空白图推理测试。
- 前置摄像头需求最复杂，坑洼、路面病害、障碍物和“鬼探头”需要专项训练与时序危险模型，不能直接用通用 COCO 模型替代。
- 随便放社区模型会造成“能加载但预警不可信”，所以前置危险模型仍不放假权重。

## 2.1 COCO 与 ONNX 区别

- COCO 是数据集和类别体系，常用于训练/评估通用目标检测模型，典型类别包括人、车、自行车、手机等。
- ONNX 是模型文件格式，用于保存已经训练好的网络结构和权重，方便 OpenCV DNN、ONNX Runtime、RKNN 工具链加载推理。
- 一个模型可以用 COCO 数据训练，再导出为 ONNX；但 COCO 本身不是模型，ONNX 本身也不代表某个类别表。
- COCO 没有闭眼、半闭眼、打哈欠、疲劳等驾驶员状态类别，因此 DMS 不能只用普通 COCO 模型替代。

## 3. 后置鱼眼模型

目标文件：

```text
models/onnx/rear_yolo.onnx
```

建议类别：

```text
0 person
1 bicycle
2 car
3 motorcycle
5 bus
7 truck
```

C++ 节点会按目标框高度估算距离，再按连续帧距离变化估算靠近速度。由于后置摄像头是鱼眼镜头，`rear_blind_spot_node` 已加入 OpenCV fisheye 去畸变入口。真实使用时必须先用棋盘格/圆点板标定 `fisheye_k` 和 `fisheye_d`，再打开 `fisheye_undistort`。

## 4. 驾驶员监测模型

目标文件：

```text
models/onnx/driver_face_yunet.onnx
models/onnx/driver_dms_yolo.onnx
```

当前采用组合方案：

- YuNet：做人脸检测、脸框中心偏移、连续人脸丢失判断。
- SafeDrive DMS YOLO：检测眼睛状态、张嘴/哈欠、手机和吸烟等分心行为。

SafeDrive DMS 类别 ID：

```text
0 eye_open
1 eye_half
2 eye_closed
3 mouth_open
4 mouth_closed
5 phone
6 cigarette
7 seatbelt_on
8 seatbelt_off
```

项目默认参数在根目录 `config/ev_ads_runtime.launch.xml` 的 `driver_attention` 节点中：

```text
open_eye_class_ids: [0]
half_eye_class_ids: [1]
closed_eye_class_ids: [2]
yawn_class_ids: [3]
phone_class_ids: [5]
distracted_class_ids: [6]
```

单帧 YOLO 只能提供证据，最终疲劳判断应叠加 PERCLOS 滑动窗口、打哈欠频率、头姿持续偏离、人脸持续消失和置信度衰减。当前 C++ 节点已加入人脸短时漏检缓冲，连续消失超过 `face_absence_warning_ms` 后按分心风险处理。

## 5. 前置危险模型

目标文件：

```text
models/onnx/front_road_hazard.onnx
```

前置模型建议拆成两层：

- 检测层：车辆、行人、自行车、障碍物、坑洼、路面破损。
- 时序风险层：跟踪、TTC、横向速度、路侧遮挡区、突然横穿概率。

“鬼探头”不是单帧类别，而是遮挡区内目标突然进入车道的时序危险事件，必须单独建数据集训练与验证。

## 6. 上板建议

1. 用真实后置画面验证 `rear_yolo.onnx` 的类别与置信度。
2. 标定后置鱼眼 `fisheye_k`、`fisheye_d`。
3. 开启 `fisheye_undistort` 后重新标定 `distance_focal_px`。
4. 用真实驾驶员摄像头验证 `driver_face_yunet.onnx` 与 `driver_dms_yolo.onnx`，重点检查闭眼、半闭眼、哈欠、手机和夜间补光场景。
5. 前置模型训练完成后再接入 `front_risk_node`。
6. ONNX 功能稳定后转 RKNN，并把启动参数切到 RKNN 后端。
7. 每个模型必须记录文件 hash、输入尺寸、类别表、阈值、测试集指标和许可证。
