# 前向感知结果
std_msgs/Header header
uint8   primary_class      # 0=无 1=行人 2=车辆 3=自行车 4=动物 5=路面障碍物 6=坑洼
float32 ttc                # s，估计的剩余碰撞时间；inf 表示无威胁
float32 distance           # m
float32 closing_speed      # m/s，正 = 接近
float32 lateral_offset     # m，正 = 右侧
float32 risk_score         # 0~1
float32 confidence         # 0~1
uint8   health             # 0=正常 1=数据过旧 2=错误 3=断开
