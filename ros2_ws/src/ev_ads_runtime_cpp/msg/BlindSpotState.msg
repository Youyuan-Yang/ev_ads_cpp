# 后向鱼眼三分区盲区
std_msgs/Header header
uint8   zone_left          # 0=空 1=有目标 2=正在靠近
uint8   zone_center
uint8   zone_right
float32 left_distance      # 米，未知填 -1
float32 center_distance
float32 right_distance
float32 left_closing       # 米/秒
float32 center_closing
float32 right_closing
float32 risk_score         # 0~1，三区聚合
uint8   health
