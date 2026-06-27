#include "color_detect/beacon_fusion.hpp"
#include <cstdint>

namespace color_detect {

void BeaconFusion::update_left(const BeaconState& state) {
  left_ = state;
}

void BeaconFusion::update_right(const BeaconState& state) {
  right_ = state;
}

bool BeaconFusion::has_valid_detection() const {
  return (left_.state != BeaconState::UNKNOWN && left_.confidence > 0.1f) ||
         (right_.state != BeaconState::UNKNOWN && right_.confidence > 0.1f);
}

BeaconState BeaconFusion::fused() const {
  BeaconState result;
  result.timestamp_ms = (left_.timestamp_ms > right_.timestamp_ms)
                            ? left_.timestamp_ms : right_.timestamp_ms;

  auto l = left_.state;
  auto r = right_.state;

  // ── 融合表 ──
  if (l == r) {
    // 一致：AA / BB / UNK-UNK
    result.state = l;
    result.confidence = std::max(left_.confidence, right_.confidence);
    return result;
  }

  // 一边 UNKNOWN → 取另一边
  if (l == BeaconState::UNKNOWN) {
    result.state = r;
    result.confidence = right_.confidence * 0.85f;  // 单目置信度稍降
    return result;
  }
  if (r == BeaconState::UNKNOWN) {
    result.state = l;
    result.confidence = left_.confidence * 0.85f;
    return result;
  }

  // ── 冲突：A vs B → UNKNOWN ──
  result.state = BeaconState::UNKNOWN;
  result.confidence = 0.0f;
  return result;
}

void BeaconFusion::reset() {
  left_ = BeaconState{};
  right_ = BeaconState{};
}

}  // namespace color_detect
