#include "color_detect/beacon_protocol.hpp"

#include <algorithm>
#include <sstream>

namespace color_detect {

namespace {

ColorClass str_to_color(const std::string& value) {
  std::string upper = value;
  std::transform(upper.begin(), upper.end(), upper.begin(), ::toupper);
  if (upper == "WHITE") return ColorClass::WHITE;
  if (upper == "CYAN") return ColorClass::CYAN;
  if (upper == "BLUE") return ColorClass::BLUE;
  if (upper == "YELLOW") return ColorClass::YELLOW;
  if (upper == "MAGENTA") return ColorClass::MAGENTA;
  if (upper == "GREEN") return ColorClass::GREEN;
  if (upper == "RED") return ColorClass::RED;
  return ColorClass::UNKNOWN;
}

bool color_matches(ColorClass actual, ColorClass expected) {
  if (actual == expected) return true;
  if (expected == ColorClass::BLUE && actual == ColorClass::CYAN) return true;
  if (expected == ColorClass::CYAN && actual == ColorClass::BLUE) return true;
  return false;
}

}  // namespace

const char* BeaconState::state_name(State s) {
  switch (s) {
    case UNKNOWN:
      return "UNKNOWN";
    case STOP:
      return "STOP";
    case WAIT:
      return "WAIT";
    case GO:
      return "GO";
    default:
      return "?";
  }
}

std::string BeaconState::to_string() const {
  std::ostringstream oss;
  oss << "BeaconState{state=" << state_name(state)
      << "(" << static_cast<int>(state) << ")"
      << ", conf=" << confidence
      << ", vis=" << n_visible_segments << "}";
  return oss.str();
}

BeaconProtocol::BeaconProtocol(const BeaconConfig& config) {
  stop_color_ = str_to_color(config.color_stop);
}

BeaconState BeaconProtocol::decode(const std::vector<ColorClass>& segments,
                                   float angle_deg,
                                   float center_x,
                                   float center_y,
                                   float length_px,
                                   float area_px) const {
  BeaconState state;
  state.beacon_angle_deg = angle_deg;
  state.beacon_center_x = center_x;
  state.beacon_center_y = center_y;
  state.beacon_length_px = length_px;
  state.beacon_area_px = area_px;

  int visible_segments = 0;
  int wait_segments = 0;
  for (auto color : segments) {
    if (color == ColorClass::UNKNOWN) {
      continue;
    }
    ++visible_segments;
    if (color_matches(color, stop_color_)) {
      ++wait_segments;
    }
  }

  state.n_visible_segments = visible_segments;
  state.n_white_segments = wait_segments;
  state.state = (wait_segments > 0) ? BeaconState::WAIT : BeaconState::GO;
  state.confidence = (wait_segments > 0) ? 0.95f : 0.80f;
  return state;
}

}  // namespace color_detect
