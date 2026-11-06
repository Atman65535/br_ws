# 上位机 ↔ 主控 串口通信（ROS2）V1.0

基于《上位机与主控通讯协议》实现，**配置驱动**：队友写一个 `.msg` + 在 `interfaces/CMakeLists.txt` 登记 + 在 `config/ports.yaml` 加一段，串口、组帧、拆帧、订阅、发布全部自动。

想加新端口时 可参考[`接入指南.md`](接入指南.md) 

## 目录结构

```
communicate_ros2/
├── python/                    
│   ├── communicate_py/
│   │   ├── packet.py          # 协议层
│   │   ├── engine.py          # 反射引擎
│   │   └── communicate_node.py
│   ├── config/ports.yaml      # 声明表（改这里）
│   ├── launch/communicate.launch.py
│   ├── setup.py / setup.cfg / package.xml
│   └── resource/communicate_py
├── interfaces/                # 共享接口（3 个 .msg）
│   └── msg/
│       ├── RobotStatus.msg    # RX P1：主控综合状态（12 个 float）
│       ├── LocError.msg       # TX P2：定位误差（3 个 float）
│       └── LidarPos.msg       # TX P3：雷达位置（2 个 float）
└── cpp/                       #旧回调写法，本版不维护，暂不同步配置驱动
```

## 功能说明

### 主要功能

1. **串口通信**：115200 波特率与 STM32 主控收发，断线自动重连 + 断流告警。
2. **数据包处理**：0xFF 帧协议组帧/拆帧，半帧、多帧、粘连都能切分，带诊断计数（校验失败/重同步/垃圾字节）。
3. **配置驱动**：队友只写 `.msg` + 在 `interfaces/CMakeLists.txt` 登记 + 在 `ports.yaml` 加一段，订阅/发布自动装配，不碰串口和组帧代码。

## 构建

```bash
# 把 python 和 interfaces 拷进工作空间（示例）
mkdir -p ~/ros2_ws/src
cp -r python interfaces ~/ros2_ws/src/

cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

## 运行

```bash
# 串口设备在 config/ports.yaml 的 serial.device 里配置，直接跑：
ros2 run communicate_py communicate_node

# 或用 launch（可传 ports_yaml 覆盖声明表路径）：
ros2 launch communicate_py communicate.launch.py

# 临时覆盖串口（不改 yaml）时：
ros2 run communicate_py communicate_node --ros-args -p device:=/dev/ttyUSB1 -p baudrate:=9600
```

## 参考测试（先测通信链路）

```bash
# 1. 发 TX P2/P3 假坐标（电控在主控端确认收到即"通"）
ros2 topic pub /robot/loc_error robot_interfaces/msg/LocError \
  "{error_x_m: 0.5, error_y_m: 0.3, error_theta_rad: 0.0}" --once

ros2 topic pub /robot/lidar_pos robot_interfaces/msg/LidarPos \
  "{x_m: 0.5, y_m: 0.3}" --once

# 2. 收 RX P1 综合状态（应看到 ~20Hz 的状态帧）
ros2 topic echo /robot/status
```

## 注意事项

- 串口权限：`sudo usermod -aG dialout $USER` 后重新登录；或临时 `sudo chmod 666 /dev/ttyUSB0`。
- 串口设备名可能变（USB 重排），用前 `ls /dev/ttyUSB*` 确认，建议用 udev 规则固定成别名。
- 发 P5~P8 自动动作：框架已带空闲门控（yaml 标 `idle_check: true` 即发前查 `master_fsm_state == MS_IDLE`），
  但注意到`MS_IDLE` 枚举值电控给的协议中不清晰，后续会与电控进一步确认。
- Port 11 复位全部 FSM 是高风险命令：框架默认不装配（yaml 标 `dangerous: true` 需显式 `-p allow_dangerous:=true`）。
- `float` 直接按小端打包，依赖上位机是小端（x86_64 / ARM 均满足）。
