# 标准机械臂关节状态消息 (弧度单位)
# 所有关节角度使用弧度作为单位

std_msgs/Header header

# 六个主要关节角度 (弧度)
float32 joint1
float32 joint2
float32 joint3
float32 joint4
float32 joint5
float32 joint6
int32 but1
int32 but2

# 夹爪角度 (弧度)
float32 gripper

# 可选的运动控制参数
float32 time  # 运动时间(秒)，默认为0表示立即执行