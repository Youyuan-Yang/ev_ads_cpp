# 驾驶员监测输出
std_msgs/Header header
uint8   face_visible       # 0=未见人脸 1=可见 2=未知
float32 eye_closure_ratio  # 0~1，PERCLOS 近似值
float32 head_pitch         # rad，低头为正
float32 head_yaw           # rad，左转为正
float32 distraction_ratio  # 0~1，注意力偏离比例
float32 fatigue_score      # 0~1
float32 confidence
uint8   health
