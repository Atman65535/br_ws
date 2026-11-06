"""msg <-> payload 反射互转 —— 通信框架的核心。

靠 ROS2 消息自带的 get_fields_and_field_types()（按字段声明顺序返回 {字段名: 类型}）
自动把任意 msg 打包成协议 payload、或从 payload 拆回 msg，从而做到"队友只声明 .msg
的字段顺序，不用手写 struct.pack / struct.unpack"。

约定：协议 payload 里的 float 一律是 4 字节 float32（小端 IEEE754）、整数是 int32。
      所以 float64 显式报错——宁可当场炸出来，也别发出长度不对的错帧。
"""
import importlib
import struct

# ROS2 运行时字段类型名 -> struct 格式（协议全是小端）
# 注意：get_fields_and_field_types() 返回的是运行时类型名（float/double/boolean），
#       不是 .msg 里写的 float32/float64/bool，两者别混。
FMT = {
    'float': '<f',      # .msg 里的 float32
    'double': '<d',     # .msg 里的 float64
    'int8': '<b',
    'uint8': '<B',
    'int16': '<h',
    'uint16': '<H',
    'int32': '<i',
    'uint32': '<I',
    'int64': '<q',
    'uint64': '<Q',
    'boolean': '<B',    # .msg 里的 bool
}


def _fields(field_types):
    """校验字段类型可映射，并显式拒绝 float64。返回原 dict（有序）。"""
    for name, typ in field_types.items():
        if typ not in FMT:
            raise TypeError(f"字段 {name} 类型 {typ} 不支持，支持：{sorted(FMT)}")
        if typ == 'double':
            raise TypeError(f"字段 {name} 用了 float64，协议 float 必须是 float32")
    return field_types


def msg_to_payload(msg) -> bytes:
    """任意 msg 实例 -> payload 字节（字段声明顺序 = 打包顺序）。"""
    out = bytearray()
    for name, typ in _fields(msg.get_fields_and_field_types()).items():
        val = getattr(msg, name)
        if typ == 'boolean':
            val = 1 if val else 0
        out += struct.pack(FMT[typ], val)
    return bytes(out)


def payload_size(MsgClass) -> int:
    """MsgClass 对应 payload 的期望字节数（拆帧前校验用）。"""
    return sum(struct.calcsize(FMT[t]) for t in MsgClass.get_fields_and_field_types().values())


def payload_to_msg(payload, MsgClass):
    """payload 字节 -> MsgClass 实例。先校验长度，再按字段声明顺序拆。"""
    fields = _fields(MsgClass.get_fields_and_field_types())
    expected = sum(struct.calcsize(FMT[t]) for t in fields.values())
    if len(payload) != expected:
        raise ValueError(
            f"{MsgClass.__name__} 期望 payload {expected} 字节，实际 {len(payload)}")
    msg = MsgClass()
    off = 0
    for name, typ in fields.items():
        size = struct.calcsize(FMT[typ])
        val = struct.unpack_from(FMT[typ], payload, off)[0]
        if typ == 'boolean':
            val = bool(val)
        setattr(msg, name, val)
        off += size
    return msg


def resolve_msg_type(type_str):
    """把 ROS2 消息类型串 'pkg/msg/Type' 解析成 Python 消息类。"""
    parts = type_str.split('/')
    if len(parts) != 3 or parts[1] != 'msg':
        raise ValueError(f"消息类型应为 pkg/msg/Type，收到：{type_str!r}")
    pkg, _, cls = parts
    module = importlib.import_module(f'{pkg}.msg')
    return getattr(module, cls)
