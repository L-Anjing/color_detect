#ifndef COLOR_DETECT_BEACON_PROTOCOL_HPP_
#define COLOR_DETECT_BEACON_PROTOCOL_HPP_

#include <cstdint>
#include <string>
#include <vector>

#include "color_detect/color_classifier.hpp"
#include "color_detect/beacon_config.hpp"

namespace color_detect {

/// @brief 三态信标状态
///
/// 灯带编码: [SYNC] [DATA]
///   SYNC=配置色 + DATA=WAIT色 → WAIT(1)
///   SYNC=配置色 + DATA=GO色   → GO(2)
///   无同步头 / 无数据          → UNKNOWN(0)
struct BeaconState {
  enum State : uint8_t {
    UNKNOWN = 0,
    WAIT    = 1,
    GO      = 2
  };

  State state = UNKNOWN;
  float confidence = 0.0f;         // [0.0, 1.0]
  uint64_t timestamp_ms = 0;
  float beacon_angle_deg = 0.0f;
  float beacon_center_x = 0.0f;
  float beacon_center_y = 0.0f;
  float beacon_area_px = 0.0f;
  int n_visible_segments = 0;
  int n_white_segments = 0;

  std::string to_string() const;
  std::string to_json() const;     // JSON，兼容 serial 直接取 "s" 字段
  static const char* state_name(State s);
  uint8_t to_serial_byte() const { return static_cast<uint8_t>(state); }
};

/// @brief 2 段信标协议解码器
///
/// 协议: [SYNC_COLOR] [DATA_COLOR]
///   SYNC 色和 DATA 色均可配置（默认 SYNC=YELLOW, WAIT=CYAN, GO=MAGENTA）
class BeaconProtocol {
public:
  explicit BeaconProtocol(const BeaconConfig& config);

  /// @brief 解码颜色序列为三态
  BeaconState decode(const std::vector<ColorClass>& segments,
                     float angle_deg,
                     float center_x,
                     float center_y,
                     float area_px) const;

private:
  ColorClass sync_color_;   // 同步头颜色
  ColorClass wait_color_;   // WAIT 对应的数据色
  ColorClass go_color_;     // GO 对应的数据色
};

}  // namespace color_detect

#endif  // COLOR_DETECT_BEACON_PROTOCOL_HPP_
