# 主控上报的综合状态（RX P1，12 个 float，48 字节）
# 字段顺序必须与固件 Port 1 上报顺序一致，见《上位机与主控通讯协议》5.2 节
float32 x_mm                 # 世界坐标 X（mm）
float32 y_mm                 # 世界坐标 Y（mm）
float32 theta_rad            # 车体航向（rad）
float32 vx_mm_s              # Vel * cos(theta)（mm/s）
float32 vy_mm_s              # Vel * sin(theta)（mm/s）
float32 omega_rad_s          # 底盘角速度（rad/s）
float32 chassis_fsm_state    # 底盘 FSM 状态（固件枚举值）
float32 up_step_fsm_state    # 升降 FSM 状态（固件枚举值）
float32 grab_bar_fsm_state   # 武器抓取 FSM 状态（固件枚举值）
float32 master_fsm_state     # 顶层 FSM 状态（发动作前看它是否 MS_IDLE）
float32 get_blocks_fsm_state # 接块 FSM 状态（固件枚举值）
float32 game_started         # 比赛是否启动
