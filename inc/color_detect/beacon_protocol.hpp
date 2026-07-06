#ifndef COLOR_DETECT_BEACON_PROTOCOL_HPP_
#define COLOR_DETECT_BEACON_PROTOCOL_HPP_

#include <cstdint>
#include <string>
#include <vector>

#include "color_detect/color_classifier.hpp"
#include "color_detect/beacon_config.hpp"

namespace color_detect {

struct BeaconState {
  enum State : uint8_t {
    GO = 0,
    WAIT = 1,
    STOP = 2,
    UNKNOWN = 3
  };

  State state = UNKNOWN;
  float confidence = 0.0f;
  uint64_t timestamp_ms = 0;
  float beacon_angle_deg = 0.0f;
  float beacon_center_x = 0.0f;
  float beacon_center_y = 0.0f;
  float beacon_length_px = 0.0f;
  float beacon_area_px = 0.0f;
  int n_visible_segments = 0;
  int n_white_segments = 0;

  std::string to_string() const;
  static const char* state_name(State s);
  uint8_t to_serial_byte() const { return static_cast<uint8_t>(state); }
};

class BeaconProtocol {
public:
  explicit BeaconProtocol(const BeaconConfig& config);

  BeaconState decode(const std::vector<ColorClass>& segments,
                     float angle_deg,
                     float center_x,
                     float center_y,
                     float length_px,
                     float area_px) const;

private:
  ColorClass stop_color_;
};

}  // namespace color_detect

#endif  // COLOR_DETECT_BEACON_PROTOCOL_HPP_
