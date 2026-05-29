# 制动接口（默认禁用，仅低速台架/演示）
std_msgs/Header header
bool   enable              # 必须 true 才执行
float32 demand             # 0~1，制动力度需求
uint32 max_duration_ms     # 单次最大持续
string source              # 来源说明，例 "fusion_l3_front"
uint8  safety_gates_passed # 位掩码：位0=前摄正常 位1=IMU正常 位2=配置允许
