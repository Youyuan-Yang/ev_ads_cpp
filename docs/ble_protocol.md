# EV-ADS BLE 协议（ESP32-C6 ↔ RK3588）

## 1. 设备发现
- **Local Name** / **Advertising Name**：`EVADAR-C6`
- 主 Service UUID 写入 Adv，便于扫描时直接过滤。
- Connection Interval：固件请求 12–24 (15–30 ms)，supervision timeout 200 (2 s)。

## 2. GATT 结构

| 名称 | UUID | 属性 | 说明 |
|------|------|------|------|
| Vital Service | `0000ad01-0000-1000-8000-00805f9b34fb` | — | 雷达根服务 |
| Notify Char   | `0000ad02-0000-1000-8000-00805f9b34fb` | READ + NOTIFY | 雷达 JSON 数据，5–10 Hz |
| Config Char   | `0000ad03-0000-1000-8000-00805f9b34fb` | READ + WRITE  | 主机下发命令（可选） |
| Health Char   | `0000ad04-0000-1000-8000-00805f9b34fb` | READ + NOTIFY | 固件健康 JSON，2 s |

> 注：UUID 与 `firmware/esp32c6_mmwave_ble/include/project_config.h` 严格一致，修改时**同步更新两侧**。

## 3. Notify 数据格式（v1 调试版）

紧凑 JSON，每帧一条 ASCII 字符串，**不带换行**：

```json
{"v":1,"seq":12,"t":34567,"br":16.20,"hr":78.40,"d":52.10,"st":7}
```

| 字段 | 类型 | 含义 |
|------|------|------|
| `v`  | int  | 协议版本，当前 `1` |
| `seq`| u32  | ESP32-C6 自增帧号；用于 RK3588 端推断丢帧 |
| `t`  | u32  | ESP32-C6 `millis()`；用于估计端到端延迟 |
| `br` | float | 呼吸率 BPM；`st & 0x01` 为 1 时有效 |
| `hr` | float | 心率 BPM；`st & 0x02` 为 1 时有效 |
| `d`  | float | 距离 cm；`st & 0x04` 为 1 时有效 |
| `st` | u8   | 状态位 bitmask |

### 状态位 `st`
| bit | 名称       | 含义 |
|-----|------------|------|
| 0   | `BREATH`   | 本帧呼吸率有效 |
| 1   | `HEART`    | 本帧心率有效 |
| 2   | `DISTANCE` | 本帧距离有效 |
| 3   | `PRESENCE` | 本帧人体被检测到 |
| 4-7 | 保留       | 必须填 0 |

未带有效位的字段保留上一帧值，便于解析端简化逻辑，但 **必须** 配合 `st` 判断是否信任。

## 4. Health 数据格式

```json
{"v":1,"up":12345,"nt":120,"nd":3,"rs":0,"cn":1,"fw":"0.1.0"}
```

| 字段 | 类型 | 含义 |
|------|------|------|
| `up` | u32 | 固件 uptime ms |
| `nt` | u32 | 累计 Notify 帧数 |
| `nd` | u32 | 因未连接被丢弃的帧数 |
| `rs` | u32 | 距离上一次雷达成功 update 的毫秒数 |
| `cn` | u8  | 当前是否有 BLE 客户端 (0/1) |
| `fw` | str | 固件版本 |

## 5. Config 写入命令（保留扩展位）

主机向 Config Char 写入 UTF-8 字符串，固件按行解析：
```json
{"cmd":"ping"}
{"cmd":"notify_rate","hz":10}
{"cmd":"led","r":0,"g":125,"b":0}
```
当前固件只打印不执行，**预留协议槽位**给未来版本。

## 6. 节流策略
- 雷达驱动 `mmWave.update(100)` 阻塞 ≤100 ms。
- 主循环每 `EVADAR_NOTIFY_MIN_INTERVAL_MS`(100 ms) 触发一次 Notify，
  无新数据时沿用上一帧值，但 `st` 中相应有效位置 0。
- 实测速率约 **8–10 Hz**。

## 7. 错误恢复
- BLE 断开：固件立即重新广播。
- 雷达 UART 无数据：`rs` 增大，串口打印 `[mmWave] waiting for data...`，但 Notify 仍按节流发送（用于探活）。
- 主机端建议 `stale_ms > 500` 标记 STALE，> 3000 标记 DISCONNECTED。

## 8. 版本演进
| version | 状态 | 变更 |
|---------|------|------|
| `v=1`  | 当前  | 紧凑 JSON 调试版 |
| `v=2`  | 计划  | 二进制 12 字节定长帧；保留 JSON 作为调试模式 |
