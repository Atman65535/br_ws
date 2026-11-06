// 通信节点（C++ / rclcpp）
//   - 收主控 RX P1 综合状态  -> 发布 robot/status
//   - 订阅 robot/loc_error    -> 发 TX P2 定位误差
//   - 订阅 robot/lidar_pos    -> 发 TX P3 雷达位置
//
// 依赖：rclcpp、robot_interfaces、serial（libserial-dev，头文件 <serial/serial.h>）
// 注意：float 直接 memcpy 成字节，依赖上位机是小端（x86_64 / ARM 都是），
//       与协议要求的 payload 小端一致。
#include <rclcpp/rclcpp.hpp>
#include <serial/serial.h>

#include <cstring>
#include <memory>
#include <string>
#include <vector>

#include "robot_interfaces/msg/robot_status.hpp"
#include "robot_interfaces/msg/loc_error.hpp"
#include "robot_interfaces/msg/lidar_pos.hpp"

#include "communicate_cpp/packet.hpp"

using robot_interfaces::msg::RobotStatus;
using robot_interfaces::msg::LocError;
using robot_interfaces::msg::LidarPos;

namespace
{

// 把 float 数组按小端拷进 payload 字节
void append_float(std::vector<uint8_t> & out, float v)
{
  const auto * p = reinterpret_cast<const uint8_t *>(&v);
  out.insert(out.end(), p, p + sizeof(float));
}

}  // namespace

class CommunicateNode : public rclcpp::Node
{
public:
  CommunicateNode()
  : Node("communicate")
  {
    device_ = this->declare_parameter<std::string>("device", "/dev/ttyUSB0");
    baud_ = this->declare_parameter<int>("baudrate", 115200);

    status_pub_ = this->create_publisher<RobotStatus>("robot/status", 10);
    loc_sub_ = this->create_subscription<LocError>(
      "robot/loc_error", 10, [this](const LocError::SharedPtr m) { on_loc(m); });
    lidar_sub_ = this->create_subscription<LidarPos>(
      "robot/lidar_pos", 10, [this](const LidarPos::SharedPtr m) { on_lidar(m); });

    parser_.set_callback([this](const br_packet::Packet & p) { on_packet(p); });

    open_serial();
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(10), [this]() { timer_cb(); });
  }

private:
  void open_serial()
  {
    try {
      serial_.setPort(device_);
      serial_.setBaudrate(baud_);
      serial_.setTimeout(1000, 1000, 0, 0, 0);
      serial_.open();
      RCLCPP_INFO(this->get_logger(), "Serial opened: %s @ %d", device_.c_str(), baud_);
    } catch (const serial::IOException & e) {
      RCLCPP_WARN(this->get_logger(), "Failed to open %s: %s", device_.c_str(), e.what());
    }
  }

  // TX P2：定位误差 [error_x_m, error_y_m, error_theta_rad]
  void on_loc(const LocError::SharedPtr m)
  {
    std::vector<uint8_t> payload;
    append_float(payload, m->error_x_m);
    append_float(payload, m->error_y_m);
    append_float(payload, m->error_theta_rad);
    write_frame(2, payload);
  }

  // TX P3：雷达位置 [x_m, y_m]
  void on_lidar(const LidarPos::SharedPtr m)
  {
    std::vector<uint8_t> payload;
    append_float(payload, m->x_m);
    append_float(payload, m->y_m);
    write_frame(3, payload);
  }

  void write_frame(uint8_t port, const std::vector<uint8_t> & payload)
  {
    if (!serial_.isOpen()) {
      open_serial();
      if (!serial_.isOpen()) {
        return;
      }
    }
    auto frame = br_packet::pack_packet(port, payload);
    try {
      serial_.write(frame);
    } catch (const serial::IOException & e) {
      RCLCPP_WARN(this->get_logger(), "Write failed: %s", e.what());
      serial_.close();
    }
  }

  // 定时轮询串口：有字节就喂给拆帧器
  void timer_cb()
  {
    if (!serial_.isOpen()) {
      open_serial();
      return;
    }
    size_t avail = serial_.available();
    if (avail == 0) {
      return;
    }
    std::vector<uint8_t> buf(avail);
    serial_.read(buf.data(), avail);
    parser_.feed(buf.data(), buf.size());
  }

  // 收到完整一帧，按 port 分发。这里只处理 RX P1 综合状态。
  void on_packet(const br_packet::Packet & p)
  {
    if (p.type == 1 && p.port == 1 && p.payload.size() == 48) {
      float v[12];
      std::memcpy(v, p.payload.data(), sizeof(v));

      RobotStatus msg;
      msg.x_mm                 = v[0];
      msg.y_mm                 = v[1];
      msg.theta_rad            = v[2];
      msg.vx_mm_s              = v[3];
      msg.vy_mm_s              = v[4];
      msg.omega_rad_s          = v[5];
      msg.chassis_fsm_state    = v[6];
      msg.up_step_fsm_state    = v[7];
      msg.grab_bar_fsm_state   = v[8];
      msg.master_fsm_state     = v[9];
      msg.get_blocks_fsm_state = v[10];
      msg.game_started         = v[11];
      status_pub_->publish(msg);
    }
  }

  serial::Serial serial_;
  std::string device_;
  int baud_;

  br_packet::PacketParser parser_;

  rclcpp::Publisher<RobotStatus>::SharedPtr status_pub_;
  rclcpp::Subscription<LocError>::SharedPtr loc_sub_;
  rclcpp::Subscription<LidarPos>::SharedPtr lidar_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CommunicateNode>());
  rclcpp::shutdown();
  return 0;
}
