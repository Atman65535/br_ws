"""通信节点（Python / rclpy）—— 配置驱动。

读 config/ports.yaml，自动为每个 TX 端口建订阅、每个 RX 端口建发布，
串口收发在单个定时器里做。队友只改 ports.yaml + 写 .msg，不碰本文件。

针对协议第 9 节做的三道防护：
  第 7 条  诊断计数：校验失败 / 串口重同步 / 垃圾字节，定时打印 + 断流告警
  第 5 条  动作端口空闲门控：P5~P8 标 idle_check 后，发前查 master_fsm_state==MS_IDLE
  第 6 条  高危端口总闸：P11 标 dangerous 后默认不装配，要显式 allow_dangerous

层次：
  packet.py  —— 协议层（纯字节组帧/拆帧，带诊断计数）
  engine.py  —— msg <-> payload 反射互转
  本文件      —— 读表 + 装配 + 串口收发 + 防护
"""
import os
import time

import rclpy
import serial
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node

from communicate_py.engine import msg_to_payload, payload_to_msg, resolve_msg_type
from communicate_py.packet import pack_packet, PacketParser

DEFAULT_PTYPE = 1
DEFAULT_LEVEL = 3
DEFAULT_QOS = 10
DEFAULT_POLL = 0.01
DEFAULT_DIAG_PERIOD = 5.0
DEFAULT_STALE_TIMEOUT = 2.0
DEFAULT_RECONNECT_PERIOD = 1.0


class CommunicateNode(Node):
    def __init__(self):
        super().__init__('communicate')

        self.declare_parameter('ports_yaml', '')       # 空 = 用包默认 config/ports.yaml
        self.declare_parameter('device', '')           # 空 = 用 yaml 的 serial.device
        self.declare_parameter('baudrate', 0)          # 0  = 用 yaml 的 serial.baudrate
        self.declare_parameter('allow_dangerous', False)  # 高危端口(P11)总闸，默认关

        # 状态 / 统计
        self._master_fsm = None      # 缓存的顶层 FSM 状态（动作端口发前查空闲用）
        self._last_rx_time = None    # 上次收到有效帧的时刻（断流告警用）
        self._last_warn = 0.0        # 门控告警节流
        self._last_open_attempt = 0.0  # 上次尝试开串口的时刻（重连节流用）

        cfg = self._load_config()
        self._setup_serial(cfg.get('serial', {}))
        self._setup_idle_gate(cfg.get('idle_check'))
        self._setup_ports(cfg)

        self.create_timer(self._poll_period, self._timer_cb)
        self.create_timer(self._diag_period, self._diagnostics_cb)

        self.get_logger().info(
            f'通信节点就绪：TX {len(self._tx)} 路，RX {len(self._rx)} 路，'
            f'串口 {self._device} @ {self._baud}')
        if self._idle_value is None:
            self.get_logger().warning(
                'idle_check.idle_value 未配置（MS_IDLE 未知）：动作端口(idle_check=true)一律拒发，'
                '跟电控确认 MS_IDLE 枚举值后填上')

    # ---- 配置 ----
    def _load_config(self):
        path = self.get_parameter('ports_yaml').value
        if not path:
            path = os.path.join(
                get_package_share_directory('communicate_py'), 'config', 'ports.yaml')
        self.get_logger().info(f'加载声明表：{path}')
        with open(path, 'r', encoding='utf-8') as f:
            return yaml.safe_load(f) or {}

    # ---- 串口 ----
    def _setup_serial(self, serial_cfg):
        device = self.get_parameter('device').value or serial_cfg.get('device', '/dev/ttyUSB0')
        baud = self.get_parameter('baudrate').value or serial_cfg.get('baudrate', 115200)
        self._poll_period = float(serial_cfg.get('poll_period', DEFAULT_POLL))
        self._diag_period = float(serial_cfg.get('diagnostics_period', DEFAULT_DIAG_PERIOD))
        self._stale_timeout = float(serial_cfg.get('stale_timeout', DEFAULT_STALE_TIMEOUT))
        self._reconnect_period = float(serial_cfg.get('reconnect_period', DEFAULT_RECONNECT_PERIOD))
        self._device, self._baud = device, int(baud)
        self._ser = None
        self._parser = PacketParser()
        self._open_serial()

    def _open_serial(self):
        # 重连节流：设备没插/权限不对时每个 poll 周期都会调进来，不节流会刷屏。
        now = time.time()
        if now - self._last_open_attempt < self._reconnect_period:
            return
        self._last_open_attempt = now
        try:
            self._ser = serial.Serial(self._device, self._baud, timeout=1.0)
            self.get_logger().info(f'串口已打开：{self._device} @ {self._baud}')
        except (serial.SerialException, OSError) as e:
            self.get_logger().warning(f'打开串口失败 {self._device}：{e}')
            self._close_serial()

    def _close_serial(self):
        if self._ser is not None:
            try:
                self._ser.close()
            except (serial.SerialException, OSError):
                pass
            self._ser = None

    def _write(self, port, payload, ptype, level):
        if self._ser is None:
            self._open_serial()
            if self._ser is None:
                return
        try:
            self._ser.write(pack_packet(port, payload, ptype=ptype, level=level))
        except (serial.SerialException, OSError) as e:
            self.get_logger().warning(f'串口写入失败：{e}')
            self._close_serial()

    # ---- 动作端口空闲门控（协议第 9 节第 5 条）----
    def _setup_idle_gate(self, idle_cfg):
        self._idle_rx_port = None
        self._idle_field = None
        self._idle_value = None
        if idle_cfg:
            self._idle_rx_port = int(idle_cfg['rx_port'])
            self._idle_field = idle_cfg['field']
            self._idle_value = idle_cfg.get('idle_value')  # None = MS_IDLE 未确认

    def _idle_ok(self):
        if self._idle_value is None:
            self._throttled_warn('动作端口被拒：MS_IDLE 未配置（ports.yaml idle_check.idle_value）')
            return False
        if self._master_fsm is None:
            self._throttled_warn('动作端口被拒：尚未收到主控状态帧（无 master_fsm_state）')
            return False
        if int(round(float(self._master_fsm))) != int(self._idle_value):
            self._throttled_warn(
                f'动作端口被拒：master_fsm={self._master_fsm} != MS_IDLE={self._idle_value}')
            return False
        return True

    def _throttled_warn(self, text):
        now = time.time()
        if now - self._last_warn >= 1.0:
            self.get_logger().warning(text)
            self._last_warn = now

    # ---- 端口装配 ----
    def _setup_ports(self, cfg):
        self._tx = []
        self._rx = {}
        allow_dangerous = self.get_parameter('allow_dangerous').value

        for entry in cfg.get('tx', []):
            port = int(entry['port'])
            ptype = int(entry.get('ptype', DEFAULT_PTYPE))
            level = int(entry.get('level', DEFAULT_LEVEL))
            qos = int(entry.get('qos', DEFAULT_QOS))
            dangerous = bool(entry.get('dangerous', False))
            idle_check = bool(entry.get('idle_check', False))
            MsgClass = resolve_msg_type(entry['type'])

            if dangerous and not allow_dangerous:
                self.get_logger().warning(
                    f'TX port={port} 是高危端口(dangerous=true)，未开 allow_dangerous，跳过装配')
                continue

            self.create_subscription(
                MsgClass, entry['topic'],
                self._make_tx_callback(port, ptype, level, idle_check), qos)
            self._tx.append((port, ptype, level, idle_check))
            extra = ' +查空闲' if idle_check else ''
            self.get_logger().info(f'TX 装配：port={port} <- {entry["topic"]}{extra}')

        for entry in cfg.get('rx', []):
            port = int(entry['port'])
            qos = int(entry.get('qos', DEFAULT_QOS))
            MsgClass = resolve_msg_type(entry['type'])
            pub = self.create_publisher(MsgClass, entry['topic'], qos)
            self._rx[port] = (pub, MsgClass)
            self.get_logger().info(f'RX 装配：port={port} -> {entry["topic"]}')

    def _make_tx_callback(self, port, ptype, level, idle_check):
        def cb(msg):
            if idle_check and not self._idle_ok():
                return
            self._write(port, msg_to_payload(msg), ptype, level)
        return cb

    # ---- 串口收发 ----
    def _timer_cb(self):
        if self._ser is None:
            self._open_serial()
            return
        try:
            waiting = self._ser.in_waiting
        except (serial.SerialException, OSError):
            self._close_serial()
            return
        if waiting == 0:
            return
        try:
            data = self._ser.read(waiting)
        except (serial.SerialException, OSError):
            self._close_serial()
            return
        frames = self._parser.feed(data)
        if frames:
            self._last_rx_time = self.get_clock().now()
        for ptype, level, port, pid, payload in frames:
            self._dispatch_rx(ptype, port, payload)

    def _dispatch_rx(self, ptype, port, payload):
        if ptype != 1:
            # Type0/Type2（可靠 + ACK）暂不处理，需要重传时在这里加
            self.get_logger().debug(f'忽略 type={ptype} 帧（port={port}）')
            return
        entry = self._rx.get(port)
        if entry is None:
            self.get_logger().warning(f'收到未声明的 RX port={port}（len={len(payload)}），丢弃')
            return
        pub, MsgClass = entry
        try:
            msg = payload_to_msg(payload, MsgClass)
        except (ValueError, TypeError) as e:
            self.get_logger().error(f'RX port={port} 拆包失败：{e}')
            return
        # 缓存顶层 FSM 状态（动作端口发前查空闲用）
        if port == self._idle_rx_port and self._idle_field is not None:
            if hasattr(msg, self._idle_field):
                self._master_fsm = getattr(msg, self._idle_field)
            else:
                self.get_logger().error(
                    f'idle_check.field {self._idle_field!r} 不在 RX port={port} 的 .msg 里')
        pub.publish(msg)

    # ---- 诊断（协议第 9 节第 7 条）----
    def _diagnostics_cb(self):
        p = self._parser
        state = '已打开' if self._ser is not None else '未连接'
        stale = ''
        if self._last_rx_time is not None:
            dt = (self.get_clock().now() - self._last_rx_time).nanoseconds / 1e9
            if dt > self._stale_timeout:
                stale = f' | ⚠️ {dt:.1f}s 没收到主控数据（查串口/波特率/接线）'
        self.get_logger().info(
            f'诊断：串口{state} | 收帧 {p.good_frames} | 校验失败 {p.checksum_fail} | '
            f'重同步 {p.resync} | 垃圾字节 {p.garbage_bytes}{stale}')


def main(args=None):
    rclpy.init(args=args)
    node = CommunicateNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
