/// @file serial_bridge_node.cpp
/// @brief 串口桥接 — 将信标三态通过串口发给下位机
///
/// 协议: HEAD(0xFF 0xFE) + state(1B) + TAIL(0xAA 0xDD)
///   state: 0=UNKNOWN  1=WAIT  2=GO
///
/// 用法:
///   ros2 run color_detect serial_bridge_node --ros-args \
///     -p port:=/dev/ttyACM0 -p baudrate:=115200

#include <serial/serial.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <sstream>
#include <vector>
#include <iomanip>
#include <memory>

static const uint8_t HEAD[2] = {0xFF, 0xFE};
static const uint8_t TAIL[2] = {0xAA, 0xDD};

class SerialBridgeNode : public rclcpp::Node {
public:
  SerialBridgeNode() : Node("serial_bridge_node") {
    declare_parameter<std::string>("port", "/dev/ttyACM0");
    declare_parameter<int>("baudrate", 115200);

    auto port = get_parameter("port").as_string();
    auto baud = get_parameter("baudrate").as_int();

    try {
      ser_.setPort(port);
      ser_.setBaudrate(baud);
      ser_.setTimeout(serial::Timeout::max(), 1000, 0, 1000, 0);
      ser_.open();
    } catch (const std::exception& e) {
      RCLCPP_FATAL(get_logger(), "Failed open %s: %s", port.c_str(), e.what());
      throw;
    }
    RCLCPP_INFO(get_logger(), "Serial %s @ %d opened.", port.c_str(), baud);

    sub_ = create_subscription<std_msgs::msg::String>(
        "/color_detect/state", 10,
        std::bind(&SerialBridgeNode::cb, this, std::placeholders::_1));
  }

  ~SerialBridgeNode() { if (ser_.isOpen()) ser_.close(); }

private:
  /// 从 JSON `{"s":1,...}` 中取出 s 字段值
  int extract_state(const std::string& json) {
    // 找 '"s":' 后的数字
    auto p = json.find("\"s\":");
    if (p == std::string::npos) return -1;
    p += 4;
    while (p < json.size() && (json[p] == ' ' || json[p] == '\t')) p++;
    if (p >= json.size()) return -1;
    return json[p] - '0';  // 0/1/2 都在一位内
  }

  void cb(const std_msgs::msg::String::SharedPtr msg) {
    if (!ser_.isOpen()) return;

    int s = extract_state(msg->data);
    if (s < 0 || s > 2) s = 0;  // 兜底

    std::vector<uint8_t> pkt;
    pkt.insert(pkt.end(), HEAD, HEAD + 2);
    pkt.push_back(static_cast<uint8_t>(s));
    pkt.insert(pkt.end(), TAIL, TAIL + 2);

    try {
      ser_.write(pkt.data(), pkt.size());
    } catch (const std::exception& e) {
      RCLCPP_ERROR(get_logger(), "Write failed: %s", e.what());
      return;
    }

    static int cnt = 0;
    if (++cnt % 10 == 0) {
      std::stringstream dbg;
      dbg << std::hex << std::uppercase;
      for (auto b : pkt) dbg << std::setw(2) << std::setfill('0') << (int)b << " ";
      RCLCPP_INFO(get_logger(), "TX %s → %d [%s]",
                  dbg.str().c_str(), s,
                  s == 0 ? "UNKNOWN" : s == 1 ? "WAIT" : "GO");
    }
  }

  serial::Serial ser_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SerialBridgeNode>());
  rclcpp::shutdown();
  return 0;
}
