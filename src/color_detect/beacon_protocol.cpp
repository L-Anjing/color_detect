#include "color_detect/beacon_protocol.hpp"
#include <sstream>
#include <algorithm>

namespace color_detect {

/// 字符串 → ColorClass 映射表
static ColorClass str_to_color(const std::string& s) {
  std::string u = s;
  std::transform(u.begin(), u.end(), u.begin(), ::toupper);
  if (u == "WHITE")   return ColorClass::WHITE;
  if (u == "CYAN")    return ColorClass::CYAN;
  if (u == "YELLOW")  return ColorClass::YELLOW;
  if (u == "MAGENTA") return ColorClass::MAGENTA;
  if (u == "GREEN")   return ColorClass::GREEN;
  return ColorClass::UNKNOWN;
}

// ── BeaconState ──

const char* BeaconState::state_name(State s) {
  switch (s) {
    case UNKNOWN: return "UNKNOWN";
    case WAIT:    return "WAIT";
    case GO:      return "GO";
    default:      return "?";
  }
}

std::string BeaconState::to_string() const {
  std::ostringstream oss;
  oss << "BeaconState{state=" << state_name(state)
      << "(" << static_cast<int>(state) << ")"
      << ", conf=" << confidence
      << ", vis=" << n_visible_segments << "/2"
      << ", angle=" << beacon_angle_deg << "deg}";
  return oss.str();
}

std::string BeaconState::to_json() const {
  // 精简 JSON，"s" 字段 serial 可直接取
  std::ostringstream oss;
  oss << "{"
      << "\"s\":" << static_cast<int>(state)
      << ",\"c\":" << confidence
      << ",\"v\":" << n_visible_segments
      << ",\"a\":" << beacon_angle_deg
      << "}";
  return oss.str();
}

// ── BeaconProtocol ──

BeaconProtocol::BeaconProtocol(const BeaconConfig& config) {
  sync_color_ = str_to_color(config.sync_color);
  wait_color_ = str_to_color(config.color_wait);
  go_color_   = str_to_color(config.color_go);
}

BeaconState BeaconProtocol::decode(
    const std::vector<ColorClass>& segments,
    float angle_deg,
    float center_x,
    float center_y,
    float area_px) const {

  BeaconState r;
  r.beacon_angle_deg = angle_deg;
  r.beacon_center_x = center_x;
  r.beacon_center_y = center_y;
  r.beacon_area_px = area_px;

  if (segments.empty()) return r;

  // 统计
  int vis = 0, sync_found = 0;
  ColorClass first_data = ColorClass::UNKNOWN;

  for (auto c : segments) {
    if (c != ColorClass::UNKNOWN) vis++;
    if (c == sync_color_) sync_found++;
    else if (first_data == ColorClass::UNKNOWN) first_data = c;
  }

  r.n_visible_segments = vis;
  r.n_white_segments = sync_found;

  // 必须至少看到同步头 + 1 个非同步段
  if (sync_found < 1 || vis < 2) {
    r.confidence = 0.0f;
    return r;
  }

  r.confidence = (vis >= 2) ? 0.95f : 0.7f;

  // 数据色映射
  if (first_data == wait_color_) {
    r.state = BeaconState::WAIT;
  } else if (first_data == go_color_) {
    r.state = BeaconState::GO;
  } else {
    r.state = BeaconState::UNKNOWN;
    r.confidence = 0.3f;  // 看到同步头但颜色不对
  }

  return r;
}

}  // namespace color_detect
