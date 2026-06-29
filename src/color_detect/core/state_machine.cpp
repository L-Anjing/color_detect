#include "color_detect/state_machine.hpp"
#include <algorithm>
#include <map>

namespace color_detect {

BeaconStateMachine::BeaconStateMachine(const BeaconConfig& config)
    : config_(config) {}

void BeaconStateMachine::reset() {
  fsm_state_ = FsmState::INIT;
  window_.clear();
  stable_count_ = 0;
  last_valid_state_ = BeaconState{};
  last_valid_timestamp_ms_ = 0;
  last_any_detection_ms_ = 0;
  pending_new_state_ = BeaconState::UNKNOWN;
  pending_confirm_count_ = 0;
}

BeaconState::State BeaconStateMachine::window_majority() const {
  if (window_.empty()) return BeaconState::UNKNOWN;
  std::map<BeaconState::State, int> counts;
  for (auto s : window_) counts[s]++;
  return std::max_element(counts.begin(), counts.end(),
      [](const auto& a, const auto& b) { return a.second < b.second; })->first;
}

BeaconState BeaconStateMachine::update(const BeaconState& raw, uint64_t now_ms) {
  window_.push_back(raw.state);
  while (window_.size() > static_cast<size_t>(config_.temporal_window))
    window_.pop_front();

  BeaconState out;

  // ── Case 1: 无检测 ──
  if (raw.state == BeaconState::UNKNOWN || raw.confidence < 0.1f) {
    switch (fsm_state_) {
      case FsmState::INIT: return out;
      case FsmState::TRACKING_CONF: fsm_state_ = FsmState::INIT; return out;
      case FsmState::TRACKING_ACTIVE:
      case FsmState::STATE_CHANGING:
        fsm_state_ = FsmState::TRACKING_LOST;
        last_any_detection_ms_ = now_ms;
        out = last_valid_state_; out.timestamp_ms = now_ms;
        return out;
      case FsmState::TRACKING_LOST:
        if (now_ms - last_any_detection_ms_ >
            static_cast<uint64_t>(config_.lost_timeout_ms)) {
          out.state = BeaconState::UNKNOWN; out.confidence = 0;
          out.timestamp_ms = now_ms; return out;
        }
        out = last_valid_state_; out.timestamp_ms = now_ms;
        return out;
    }
  }

  // ── Case 2: 有检测 ──
  last_any_detection_ms_ = now_ms;

  switch (fsm_state_) {
    case FsmState::INIT:
      fsm_state_ = FsmState::TRACKING_CONF;
      stable_count_ = 1; pending_new_state_ = raw.state;
      return out;

    case FsmState::TRACKING_CONF: {
      auto maj = window_majority();
      if (maj == raw.state && maj != BeaconState::UNKNOWN) {
        if (++stable_count_ >= config_.init_confirm_frames) {
          fsm_state_ = FsmState::TRACKING_ACTIVE;
          last_valid_state_ = raw; last_valid_state_.state = maj;
          out = last_valid_state_; out.timestamp_ms = now_ms;
          return out;
        }
      } else {
        stable_count_ = std::max(0, stable_count_ - 1);
        if (stable_count_ <= 0) fsm_state_ = FsmState::INIT;
      }
      return out;
    }

    case FsmState::TRACKING_ACTIVE: {
      auto maj = window_majority();
      if (maj == last_valid_state_.state) {
        last_valid_state_ = raw; last_valid_state_.state = maj;
        out = last_valid_state_; out.timestamp_ms = now_ms;
        return out;
      }
      fsm_state_ = FsmState::STATE_CHANGING;
      pending_new_state_ = maj; pending_confirm_count_ = 1;
      out = last_valid_state_; out.timestamp_ms = now_ms;
      return out;
    }

    case FsmState::STATE_CHANGING: {
      auto maj = window_majority();
      if (maj == pending_new_state_) {
        if (++pending_confirm_count_ >= config_.debounce_frames) {
          fsm_state_ = FsmState::TRACKING_ACTIVE;
          last_valid_state_ = raw; last_valid_state_.state = maj;
          out = last_valid_state_; out.timestamp_ms = now_ms;
          return out;
        }
        out = last_valid_state_; out.timestamp_ms = now_ms;
        return out;
      }
      if (maj == last_valid_state_.state) {
        fsm_state_ = FsmState::TRACKING_ACTIVE; pending_confirm_count_ = 0;
        out = last_valid_state_; out.timestamp_ms = now_ms;
        return out;
      }
      out = last_valid_state_; out.timestamp_ms = now_ms;
      return out;
    }

    case FsmState::TRACKING_LOST:
      fsm_state_ = FsmState::TRACKING_CONF;
      stable_count_ = 1; pending_new_state_ = raw.state;
      return out;
  }
  return out;
}

}  // namespace color_detect
