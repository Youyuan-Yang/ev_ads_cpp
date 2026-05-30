# ESP32-C6 + Seeed MR60BHA2 通过 BLE GATT 上报的生命体征 + 距离
# 频率：5-10 Hz
std_msgs/Header header
uint32  seq                 # ESP32-C6 自增序号
float32 breath_rate         # 呼吸率，每分钟次数
float32 heart_rate          # 心率，每分钟次数
float32 distance            # 距离，厘米
float32 confidence          # 0~1 综合可信度（由状态位和数据新鲜度推断）
uint8   status              # 位0=呼吸有效 位1=心率有效 位2=距离有效 位3=有人存在
uint8   health              # 0=正常 1=数据过旧 2=错误 3=断开
uint32  drop_count          # 累计丢帧数（按 seq 推断）
uint32  parse_error_count   # 累计解析错误
uint32  stale_ms            # 距离上一次有效数据的毫秒
