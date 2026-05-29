# 融合后的整体风险等级
std_msgs/Header header
uint8    level                     # 0=L0 1=L1 2=L2 3=L3
float32  score                     # 0~1 融合分
string   primary_reason            # 例 "front_ttc_low" / "rear_right_approaching"
string[] reasons                   # 全部触发原因
string[] allowed_actions           # 例 ["warn_voice","warn_vibration"]
uint8    sensor_health_summary     # 位掩码：位0=前摄 位1=后摄 位2=驾驶员摄像头 位3=IMU 位4=毫米波
