/// @file serial_bridge_node.cpp
/// @brief 串口桥接 — 将信标状态和粗定位结果发给下位机
///
/// 协议:
///   HEAD(0xFF 0xFE)
///   + state(1B)          0=UNKNOWN  1=WAIT  2=GO
///   + x/y/z/yaw(4B float)
///   + TAIL(0xAA 0xDD)
///
/// 用法:
///   ros2 run color_detect serial_bridge_node --ros-args \
///     -p port:=/dev/ttyACM0 -p baudrate:=115200

#include <serial/serial.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <cmath>
#include <cstring>
#include <iomanip>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

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
  bool extract_number(const std::string& json, const std::string& key, float& value) const {
    const std::string mark = "\"" + key + "\":";
    auto pos = json.find(mark);
    if (pos == std::string::npos) return false;
    pos += mark.size();

    try {
      size_t parsed = 0;
      value = std::stof(json.substr(pos), &parsed);
      return parsed > 0 && std::isfinite(value);
    } catch (const std::exception&) {
      return false;
    }
  }

  bool build_payload(const std::string& json, std::vector<uint8_t>& payload) const {
    float state_f = 0.0f;
    float pose_valid_f = 0.0f;
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
    float yaw = 0.0f;

    bool valid = extract_number(json, "s", state_f);
    valid = extract_number(json, "pv", pose_valid_f) && valid;
    valid = extract_number(json, "x", x) && valid;
    valid = extract_number(json, "y", y) && valid;
    valid = extract_number(json, "z", z) && valid;
    valid = extract_number(json, "yaw", yaw) && valid;

    int state = static_cast<int>(state_f);
    const bool tx_valid = valid && state >= 0 && state <= 2 && pose_valid_f >= 0.5f;
    if (!tx_valid) {
      state = 0;
      x = 0.0f;
      y = 0.0f;
      z = 0.0f;
      yaw = 0.0f;
    }

    payload.clear();
    payload.push_back(static_cast<uint8_t>(state));

    auto append_float = [&](float value) {
      uint8_t bytes[sizeof(float)];
      std::memcpy(bytes, &value, sizeof(float));
      payload.insert(payload.end(), bytes, bytes + sizeof(float));
    };

    append_float(x);
    append_float(y);
    append_float(z);
    append_float(yaw);
    return tx_valid;
  }

  void cb(const std_msgs::msg::String::SharedPtr msg) {
    if (!ser_.isOpen()) return;

    std::vector<uint8_t> payload;
    const bool valid = build_payload(msg->data, payload);

    std::vector<uint8_t> pkt;
    pkt.insert(pkt.end(), HEAD, HEAD + 2);
    pkt.insert(pkt.end(), payload.begin(), payload.end());
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
      RCLCPP_INFO(get_logger(), "TX %s valid=%d", dbg.str().c_str(), valid ? 1 : 0);
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
