# msg/GaitStatus.msg
std_msgs/Header header

string current_state      # 当前状态: IDLE, STARTING, WALKING, STOPPING
float32 gait_phase        # 当前步态相位 (0.0 到 1.0)
float32 step_height       # 当前实时采用的抬腿高度 (m)
float32 gait_period       # 当前实时采用的步频 (s)
bool is_stable            # 机器人是否处于平衡状态