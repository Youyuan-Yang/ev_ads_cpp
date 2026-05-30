# 车体运动状态（来自 IMU 和安装姿态变换）
std_msgs/Header header
float32 roll              # rad，右倾为正
float32 pitch             # rad，仰头为正
float32 yaw_rate          # rad/s，左转为正
float32 accel_norm        # m/s^2，加速度模长
float32 jerk              # m/s^3
float32 confidence        # 0~1
uint32  motion_flags      # 位0=急刹 位1=明显倾斜 位2=颠簸 位3=疑似倒车/摔倒
uint8   health            # 0=正常 1=数据过旧 2=错误 3=断开
