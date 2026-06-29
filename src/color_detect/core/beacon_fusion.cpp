#include "color_detect/beacon_fusion.hpp"
#include <cstdint>
#include <algorithm>

namespace color_detect {

namespace {

void copy_pose(BeaconState& dst, const BeaconState& src) {
  dst.beacon_angle_deg = src.beacon_angle_deg;
  dst.beacon_center_x = src.beacon_center_x;
  dst.beacon_center_y = src.beacon_center_y;
  dst.beacon_length_px = src.beacon_length_px;
  dst.beacon_area_px = src.beacon_area_px;
  dst.n_visible_segments = src.n_visible_segments;
  dst.n_white_segments = src.n_white_segments;
  dst.pose_valid = src.pose_valid;
  dst.pose_x_m = src.pose_x_m;
  dst.pose_y_m = src.pose_y_m;
  dst.pose_z_m = src.pose_z_m;
  dst.yaw_deg = src.yaw_deg;
}

void fuse_pose(BeaconState& dst, const BeaconState& left, const BeaconState& right) {
  if (left.pose_valid && right.pose_valid) {
    const float wl = std::max(left.confidence, 0.01f);
    const float wr = std::max(right.confidence, 0.01f);
    const float inv = 1.0f / (wl + wr);
    dst.pose_valid = true;
    dst.pose_x_m = (left.pose_x_m * wl + right.pose_x_m * wr) * inv;
    dst.pose_y_m = (left.pose_y_m * wl + right.pose_y_m * wr) * inv;
    dst.pose_z_m = (left.pose_z_m * wl + right.pose_z_m * wr) * inv;
    dst.yaw_deg = (left.yaw_deg * wl + right.yaw_deg * wr) * inv;
    dst.beacon_center_x = (left.beacon_center_x * wl + right.beacon_center_x * wr) * inv;
    dst.beacon_center_y = (left.beacon_center_y * wl + right.beacon_center_y * wr) * inv;
    dst.beacon_length_px = (left.beacon_length_px * wl + right.beacon_length_px * wr) * inv;
    dst.beacon_area_px = (left.beacon_area_px * wl + right.beacon_area_px * wr) * inv;
    dst.beacon_angle_deg = (left.beacon_angle_deg * wl + right.beacon_angle_deg * wr) * inv;
    dst.n_visible_segments = std::max(left.n_visible_segments, right.n_visible_segments);
    dst.n_white_segments = std::max(left.n_white_segments, right.n_white_segments);
    return;
  }
  if (left.pose_valid || left.confidence >= right.confidence) {
    copy_pose(dst, left);
  } else {
    copy_pose(dst, right);
  }
}

}  // namespace

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
    fuse_pose(result, left_, right_);
    return result;
  }

  // 一边 UNKNOWN → 取另一边
  if (l == BeaconState::UNKNOWN) {
    result.state = r;
    result.confidence = right_.confidence * 0.85f;  // 单目置信度稍降
    copy_pose(result, right_);
    return result;
  }
  if (r == BeaconState::UNKNOWN) {
    result.state = l;
    result.confidence = left_.confidence * 0.85f;
    copy_pose(result, left_);
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
