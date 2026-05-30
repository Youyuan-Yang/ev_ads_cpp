# 人机提示指令
std_msgs/Header header
uint8   level              # 0=L0 1=L1 2=L2 3=L3
string  voice_text         # 例 "注意前方行人"
uint8   vibration_pattern  # 0=关闭 1=单次 2=双次 3=连续
uint32  beep_freq_hz       # 0=静音
uint32  duration_ms
string  screen_icon        # 例 "front_ped" / "rear_right" / "drowsy"
