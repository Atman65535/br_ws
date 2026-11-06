# 定位误差（TX P2，3 个 float，12 字节）
# 主控内部会把平移误差 ×1000 转 mm，所以这里直接发米和弧度
float32 error_x_m        # X 方向误差（m）
float32 error_y_m        # Y 方向误差（m）
float32 error_theta_rad  # 航向误差（rad）
