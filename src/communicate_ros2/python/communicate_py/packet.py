"""组帧 / 拆帧核心，对应《上位机与主控通讯协议》的外层 Packet 协议。

帧格式：0xFF | LEN_H | LEN_L | META | ID | PAYLOAD | CHECKSUM，帧长 = payload + 6
  - LEN 大端；payload 里的 float 小端（IEEE754）
  - META = (type << 6) | (level << 4) | port
  - checksum = sum(payload) & 0xFF（只累加 payload）
  - 普通命令用 Type=1, Level=3, ID=0

PacketParser 带诊断计数（对应协议第 9 节第 7 条）：校验失败 / 串口重同步 / 垃圾字节，
现场联调时靠这几个数判断是串口线/波特率问题还是协议问题。
"""
import struct

HEAD = 0xFF
MAX_PAYLOAD = 200
FIX = 6


def pack_packet(port, payload, ptype=1, level=3, pid=0):
    """port + payload -> 完整帧（默认 Type=1 / Level=3 / ID=0，无确认，主用）。"""
    if not isinstance(payload, (bytes, bytearray)):
        payload = bytes(payload)
    n = len(payload)
    if n > MAX_PAYLOAD:
        raise ValueError(f"payload too long: {n} > {MAX_PAYLOAD}")
    if not 0 <= port <= 15:
        raise ValueError(f"port must be in [0, 15], got {port}")
    if not 0 <= ptype <= 2:
        raise ValueError(f"ptype must be in [0, 2], got {ptype}")
    if not 0 <= level <= 3:
        raise ValueError(f"level must be in [0, 3], got {level}")
    meta = (ptype << 6) | (level << 4) | (port & 0x0F)
    checksum = sum(payload) & 0xFF
    return bytes([HEAD, (n >> 8) & 0xFF, n & 0xFF, meta, pid & 0xFF]) + payload + bytes([checksum])


def pack_floats(*values):
    """把若干 float 按小端打包成 payload。"""
    return struct.pack("<" + "f" * len(values), *values)


def unpack_floats(payload):
    """把小端 float payload 解成元组。"""
    return struct.unpack("<" + "f" * (len(payload) // 4), payload)


class PacketParser:
    """增量拆帧器：串口 read 不能假设一次就是一帧，必须用持久缓冲区。

    用法：
        parser = PacketParser()
        for ptype, level, port, pid, payload in parser.feed(data):
            ...

    诊断计数（可读属性，随 feed 累积）：
        good_frames   成功解析的帧数
        checksum_fail 校验失败帧数
        bad_length    长度 > 200 的脏帧数
        resync        串口重同步次数（= bad_length + checksum_fail，每次丢一个 0xFF 重新找帧头）
        garbage_bytes 丢弃的字节总数（帧头前的杂散字节 + 坏帧/超长帧丢弃的尾巴；波特率不对时暴涨）
    """

    def __init__(self):
        self._buf = bytearray()
        self.good_frames = 0
        self.checksum_fail = 0
        self.bad_length = 0
        self.garbage_bytes = 0

    @property
    def resync(self):
        return self.bad_length + self.checksum_fail

    def feed(self, data):
        self._buf.extend(data)
        out = []
        while True:
            idx = self._buf.find(HEAD)
            if idx < 0:                 # 没有帧头了，剩余全是垃圾字节
                self.garbage_bytes += len(self._buf)
                self._buf.clear()
                break
            if idx > 0:                 # 帧头前的垃圾字节丢弃
                self.garbage_bytes += idx
                del self._buf[:idx]
            if len(self._buf) < 5:      # 连完整帧头都没有，等更多数据
                break

            n = (self._buf[1] << 8) | self._buf[2]
            if n > MAX_PAYLOAD:         # 长度非法：丢这个 0xFF，重新同步
                self.bad_length += 1
                del self._buf[0]
                continue

            total = n + FIX
            if len(self._buf) < total:  # 帧没收齐，等更多数据
                break

            payload = bytes(self._buf[5:5 + n])
            meta = self._buf[3]
            ptype = meta >> 6
            level = (meta >> 4) & 0x03
            port = meta & 0x0F
            pid = self._buf[4]

            if (sum(payload) & 0xFF) != self._buf[total - 1]:
                self.checksum_fail += 1
                del self._buf[0]        # 校验失败：丢帧头重新同步
                continue

            self.good_frames += 1
            out.append((ptype, level, port, pid, payload))
            del self._buf[:total]
        return out
