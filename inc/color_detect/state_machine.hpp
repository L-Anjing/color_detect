#ifndef COLOR_DETECT_STATE_MACHINE_HPP_
#define COLOR_DETECT_STATE_MACHINE_HPP_

#include <cstdint>
#include <deque>
#include <chrono>

#include "color_detect/beacon_protocol.hpp"
#include "color_detect/beacon_config.hpp"

namespace color_detect {

enum class FsmState : uint8_t {
  INIT,
  TRACKING_CONF,
  TRACKING_ACTIVE,
  STATE_CHANGING,
  TRACKING_LOST
};

class BeaconStateMachine {
public:
  explicit BeaconStateMachine(const BeaconConfig& config);

  BeaconState update(const BeaconState& raw, uint64_t now_ms);
  FsmState fsm_state() const { return fsm_state_; }
  void reset();

private:
  BeaconConfig config_;
  FsmState fsm_state_ = FsmState::INIT;
  std::deque<BeaconState::State> window_;
  int stable_count_ = 0;
  BeaconState last_valid_state_;
  uint64_t last_valid_timestamp_ms_ = 0;
  uint64_t last_any_detection_ms_ = 0;
  BeaconState::State pending_new_state_ = BeaconState::UNKNOWN;
  int pending_confirm_count_ = 0;

  BeaconState::State window_majority() const;
};

}  // namespace color_detect

#endif  // COLOR_DETECT_STATE_MACHINE_HPP_
