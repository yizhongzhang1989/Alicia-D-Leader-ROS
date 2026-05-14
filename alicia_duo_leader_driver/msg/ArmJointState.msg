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

# Overheat state, decoded from the upper bits of the leader's run-status
# byte (offset 14 of the CMD=0x06/FUNC=0x00 response).
#   0 = normal
#   1 = overheat            (raw byte ~ 0xE1: servos still active but hot)
#   2 = overheat_protect    (raw byte ~ 0xE2: torque cut by firmware)
# Only the upper bits are inspected; the lock bit (bit 0) is ignored
# so but1/but2 remain valid simultaneously.
uint8 overheat

# 夹爪角度 (弧度)
float32 gripper

# 可选的运动控制参数
float32 time  # 运动时间(秒)，默认为0表示立即执行