# EV-ADS C++ 版本优化建议

生成日期：2026-05-29

## 1. 当前状态

该项目已删除非测试 Python 运行代码，运行链路集中在 `ev_ads_runtime_cpp`：

- C++ 三路摄像头。
- C++ IMU。
- C++ 前向风险入口。
- C++ 后置 YOLO ONNX fallback。
- C++ 驾驶员 YuNet + DMS YOLO ONNX fallback。
- C++ 多模态融合 v2。
- C++ HMI、SQLite/WAL 事件记录和毫米波入口。
- `types.hpp` 统一 enum class，ROS 消息中的 `uint8` 只作为边界编码。
- `topics.hpp` 与 `runtime_config.hpp` 集中话题名和配置对象，减少节点内硬编码。
- `fusion_core.hpp` 和 `event_store.hpp/cpp` 已能在 Mac 上脱离 ROS 单测。

## 2. P0：板端验证

1. 在 RK3588 上完成 `colcon build`。
2. 以 `use_fakes:=true` 验证 C++ fake 链路。
3. 以 `use_fakes:=false perception_mode:=scripted` 验证真实摄像头和 IMU。
4. 毫米波 BLE 后端未接入前，应明确显示 `DISCONNECTED`。
5. 记录 CPU、温度、内存、topic hz 和 SQLite 事件库。

## 3. P1：模型落地

后置：

- 已补入 `models/onnx/rear_yolo.onnx`。
- 用真实鱼眼画面验证检测类别和边缘误检。
- 完成 OpenCV fisheye 标定，填入 `config/rear_fisheye.yaml`。
- 验证左/中/右三区距离和靠近速度。
- 开启去畸变后重新标定 `distance_focal_px`。
- ONNX 稳定后转换 RKNN。

DMS：

- 已补入 `models/onnx/driver_face_yunet.onnx` 和 `models/onnx/driver_dms_yolo.onnx`。
- 当前类别 ID 已按 SafeDrive 写入 `config/driver_monitor.yaml`：睁眼 `[0]`、半闭眼 `[1]`、闭眼 `[2]`、张嘴 `[3]`、手机 `[5]`、吸烟/分心 `[6]`。
- 用真实驾驶员摄像头核对人脸、闭眼、半闭眼、打哈欠、手机和夜间补光场景。
- 增加 30-60 秒 PERCLOS 滑动窗口。
- 夜间、头盔、口罩、补光场景必须单独测试。

前置：

- 车辆、行人、骑行者可用 COCO 初始化。
- 坑洼、路面病害需要专项训练。
- “鬼探头”需要检测、跟踪、遮挡区、横向速度和 TTC 的时序模型。

## 4. P2：工程化

- 已新增 Mac CMake 测试目录 `test/`，覆盖 `common.hpp`、`FusionCore` 和 `EventStore`；后续再接入 gtest 与 `yolo_onnx` 输出解析用例。
- 增加 diagnostics topic，报告延迟、频率、health。
- 增加模型版本记录：文件 hash、输入尺寸、类别表、阈值、测试集结果、许可证。
- 高频事件默认写 SQLite/WAL 并批量提交；JSONL 仅用于兼容、调试和小流量回放。
- 长稳测试至少 2 小时，记录温度、丢帧、误报和漏报。

## 5. P3：性能优化

- ONNX 仅作功能 fallback，最终推理应使用 RKNN/NPU。
- 摄像头节点减少 JPEG 压缩开销，可改共享内存或 raw image intra-process。
- 后置与 DMS 模型应控制输入尺寸、帧率和 NMS 阈值，避免 CPU 被 DNN 占满。
- fusion 节点保留低计算量，重点优化感知模型与图像链路。

## 6. P4：安全优化

- 真实制动默认关闭，只允许低速台架验证。
- L3 必须有前向 TTC 或等价强证据，不能由 DMS/毫米波单独触发。
- 模型加载失败、摄像头断连、IMU 失效时必须降级，不能沿用旧风险。
- 路测前必须建立固定场景回归集，覆盖白天、夜间、雨天、逆光、震动和遮挡。
