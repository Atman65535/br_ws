// 组帧 / 拆帧核心，对应《上位机与主控通讯协议》的外层 Packet 协议。
// 帧格式：0xFF | LEN_H | LEN_L | META | ID | PAYLOAD | CHECKSUM，帧长 = payload + 6
//   - LEN 大端；payload 里的 float 小端（IEEE754）
//   - META = (type << 6) | (level << 4) | port
//   - checksum = sum(payload) & 0xFF（只累加 payload）
//   - 普通命令用 Type=1, Level=3, ID=0
#ifndef COMMUNICATE_CPP__PACKET_HPP_
#define COMMUNICATE_CPP__PACKET_HPP_

#include <cstdint>
#include <vector>
#include <functional>
#include <algorithm>
#include <cstring>

namespace br_packet
{

constexpr uint8_t  HEAD         = 0xFF;
constexpr uint16_t MAX_PAYLOAD  = 200;
constexpr uint16_t FIX          = 6;

// 解析出的一帧
struct Packet
{
  uint8_t type;
  uint8_t level;
  uint8_t port;
  uint8_t id;
  std::vector<uint8_t> payload;
};

// 组帧：port + payload -> 完整帧。默认 Type=1 / Level=3 / ID=0（无确认，主用）。
inline std::vector<uint8_t> pack_packet(
  uint8_t port, const std::vector<uint8_t> & payload,
  uint8_t type = 1, uint8_t level = 3, uint8_t id = 0)
{
  uint16_t n = static_cast<uint16_t>(payload.size());
  if (n > MAX_PAYLOAD) {
    // 与文档一致：payload 上限 200 字节
    return {};
  }

  uint8_t checksum = 0;
  for (uint8_t b : payload) {
    checksum += b;
  }

  std::vector<uint8_t> frame(n + FIX);
  frame[0] = HEAD;
  frame[1] = static_cast<uint8_t>((n >> 8) & 0xFF);
  frame[2] = static_cast<uint8_t>(n & 0xFF);
  frame[3] = static_cast<uint8_t>((type << 6) | (level << 4) | (port & 0x0F));
  frame[4] = id;
  std::memcpy(frame.data() + 5, payload.data(), n);
  frame[n + 5] = checksum;
  return frame;
}

// 增量拆帧器：串口 read 不能假设一次就是一帧，必须用持久缓冲区。
class PacketParser
{
public:
  using Callback = std::function<void(const Packet &)>;

  void set_callback(Callback cb) { callback_ = std::move(cb); }

  void feed(const uint8_t * data, size_t len)
  {
    buffer_.insert(buffer_.end(), data, data + len);

    while (true) {
      auto head = std::find(buffer_.begin(), buffer_.end(), HEAD);
      if (head == buffer_.end()) {         // 没有帧头了，丢弃残留，等下一批
        buffer_.clear();
        break;
      }
      if (head != buffer_.begin()) {       // 帧头前的垃圾字节丢弃
        buffer_.erase(buffer_.begin(), head);
      }
      if (buffer_.size() < 5) {            // 连完整帧头都没有，等更多数据
        break;
      }

      uint16_t n = static_cast<uint16_t>((buffer_[1] << 8) | buffer_[2]);
      if (n > MAX_PAYLOAD) {               // 长度非法：丢这个 0xFF，重新同步
        buffer_.erase(buffer_.begin());
        continue;
      }

      size_t total = n + FIX;
      if (buffer_.size() < total) {        // 帧没收齐，等更多数据
        break;
      }

      uint8_t checksum = 0;
      for (uint16_t i = 0; i < n; ++i) {
        checksum += buffer_[5 + i];
      }
      if (checksum != buffer_[total - 1]) {  // 校验失败：丢帧头重新同步
        buffer_.erase(buffer_.begin());
        continue;
      }

      Packet p;
      p.type  = buffer_[3] >> 6;
      p.level = (buffer_[3] >> 4) & 0x03;
      p.port  = buffer_[3] & 0x0F;
      p.id    = buffer_[4];
      p.payload.assign(buffer_.begin() + 5, buffer_.begin() + 5 + n);

      if (callback_) {
        callback_(p);
      }
      buffer_.erase(buffer_.begin(), buffer_.begin() + total);
    }
  }

private:
  std::vector<uint8_t> buffer_;
  Callback callback_;
};

}  // namespace br_packet

#endif  // COMMUNICATE_CPP__PACKET_HPP_
