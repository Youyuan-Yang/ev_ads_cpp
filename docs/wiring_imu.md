# IMU 接线说明

> RK3588 GPIO 电平为 **3.3 V**。**严禁** 把 5 V TTL 直接接入。
> 所有接线必须在断电状态完成，确认极性后再上电。

## 1. 三种总线选择

| 接口 | 模块判别 | 优先级 |
|------|----------|--------|
| I2C  | 模块标 `SDA / SCL` | 高（首选） |
| UART | 模块标 `TX / RX`  | 中（适合带 MCU 的模块） |
| SPI  | 模块标 `SCLK / MOSI / MISO / CS` | 低（高速但接线多） |

## 2. 通电前检查清单

- [ ] 模块 VCC 电平：3.3 V 还是 5 V？（确认贴丝印或资料）
- [ ] 模块 IO 电平是否兼容 3.3 V？（5 V IO 必须串电阻或加电平转换）
- [ ] 公共地 GND 已就绪
- [ ] I2C：两根 4.7 kΩ 上拉电阻（模块内置时可省）
- [ ] UART：交叉接线（MCU TX → RK RX，MCU RX → RK TX）

## 3. 推荐杜邦线颜色

| 颜色 | 含义 |
|------|------|
| 红 | VCC |
| 黑 | GND |
| 黄 | TX |
| 绿 | RX |
| 蓝 | SDA |
| 白 | SCL |

## 4. I2C 接线示例（MPU6050 / ICM-20948）

| RK3588 40-pin Header | 模块 |
|----------------------|------|
| Pin 1 (3.3 V)        | VCC |
| Pin 6 (GND)          | GND |
| Pin 3 (I2C1_SDA)     | SDA |
| Pin 5 (I2C1_SCL)     | SCL |

确认总线编号：`ls /dev/i2c-*`；扫描设备 `i2cdetect -y 1`。

## 5. UART 接线示例

| RK3588 | 模块 |
|--------|------|
| 3.3 V  | VCC |
| GND    | GND |
| UART_TX| RX  |
| UART_RX| TX  |

确认串口：`ls /dev/serial/by-id`。在 `config/ev_ads_runtime.launch.xml` 的 `imu_motion` 节点中：

```xml
<param name="driver" value="uart"/>
<param name="port" value="/dev/serial/by-id/usb-xxx"/>
<param name="baud" value="921600"/>
```

## 6. 安装方向 & mounting_transform

车体约定：X 车头、Y 左、Z 上。把 IMU 朝向车体的相对欧拉角（degrees）写入：

```xml
<param name="mount_rpy_deg" value="[180.0, 0.0, 90.0]"/>
```

C++ runtime 中 mounting transform 已集成在 `imu_motion_node`。后续应补 gtest 覆盖 identity / yaw 90 / roll 180 / pair 一致性。

## 7. 校准

启动后保持车辆静止 2 秒（`bias_seconds: 2.0`），固件会自动估计：
- 加速度零偏（减 0,0,9.80665）
- 陀螺零偏

车辆运行中 **不会** 再重新校准；如需手动重置，重启节点即可。

## 8. 故障排查

| 现象 | 排查 |
|------|------|
| 一直 `HEALTH_DISCONNECTED` | 确认 `/dev/i2c-N` 或串口、地址、波特率 |
| 加速度模长 != 9.8 m/s² | 量程错（修改 `accel_lsb_per_g`） |
| roll 反向 | `mount_rpy_deg` 中 roll +/- 180 |
| yaw_rate 与转向相反 | `mount_rpy_deg` 中 yaw +/- 180 |
| `motion_flags=lean` 一直亮 | `lean_deg` 阈值调高 |
