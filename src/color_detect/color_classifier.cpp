#include "color_detect/color_classifier.hpp"
#include <algorithm>
#include <cmath>

namespace color_detect {

ColorClassifier::ColorClassifier(const BeaconConfig& config) : config_(config) {
  build_ranges();
}

void ColorClassifier::build_ranges() {
  const int ht = config_.hue_tolerance;
  const int smin = config_.s_min_for_color;
  const int vmin = config_.v_min_bright;

  ranges_[static_cast<size_t>(ColorClass::WHITE)] = {
    0, 180, 0, config_.s_threshold_white, config_.v_threshold, 255};
  ranges_[static_cast<size_t>(ColorClass::CYAN)] = {
    85 - ht/2, 95 + ht/2, smin, 255, vmin, 255};
  ranges_[static_cast<size_t>(ColorClass::YELLOW)] = {
    27 - ht/2, 32 + ht/2, smin, 255, vmin, 255};
  ranges_[static_cast<size_t>(ColorClass::MAGENTA)] = {
    140 - ht/2, 150 + ht/2, smin, 255, vmin, 255};
  ranges_[static_cast<size_t>(ColorClass::GREEN)] = {
    55 - ht/2, 65 + ht/2, smin, 255, vmin, 255};
}

ColorClass ColorClassifier::classify(const cv::Mat& roi_hsv,
                                     const cv::Mat& roi_mask) const {
  if (roi_hsv.empty()) return ColorClass::UNKNOWN;
  std::array<int, static_cast<size_t>(ColorClass::COUNT)> votes{};
  int total = 0;
  for (int y = 0; y < roi_hsv.rows; ++y) {
    for (int x = 0; x < roi_hsv.cols; ++x) {
      if (!roi_mask.empty() && !roi_mask.at<uint8_t>(y, x)) continue;
      auto& hsv = roi_hsv.at<cv::Vec3b>(y, x);
      if (hsv[2] < config_.v_min_bright) continue;
      total++;
      votes[static_cast<size_t>(classify_pixel(hsv))]++;
    }
  }
  if (total == 0) return ColorClass::UNKNOWN;
  size_t best = static_cast<size_t>(ColorClass::UNKNOWN);
  int best_count = 0;
  for (size_t i = 0; i < votes.size(); ++i) {
    if (votes[i] > best_count) { best_count = votes[i]; best = i; }
  }
  return (static_cast<float>(best_count) / total > 0.4f)
             ? static_cast<ColorClass>(best) : ColorClass::UNKNOWN;
}

ColorClass ColorClassifier::classify_pixel(const cv::Vec3b& hsv) const {
  uint8_t h = hsv[0], s = hsv[1], v = hsv[2];
  if (v < config_.v_min_bright) return ColorClass::UNKNOWN;
  if (s < config_.s_threshold_white)
    return (v >= config_.v_threshold) ? ColorClass::WHITE : ColorClass::UNKNOWN;
  for (size_t i = static_cast<size_t>(ColorClass::CYAN);
       i <= static_cast<size_t>(ColorClass::GREEN); ++i) {
    auto& r = ranges_[i];
    if (h >= r.h_low && h <= r.h_high && s >= r.s_low && s <= r.s_high &&
        v >= r.v_low && v <= r.v_high)
      return static_cast<ColorClass>(i);
  }
  return ColorClass::UNKNOWN;
}

ColorClass ColorClassifier::classify_with_consistency(
    const cv::Mat& roi_hsv, float* consistency) const {
  if (roi_hsv.empty()) return ColorClass::UNKNOWN;
  std::array<int, static_cast<size_t>(ColorClass::COUNT)> votes{};
  int total = 0;
  for (int y = 0; y < roi_hsv.rows; ++y) {
    for (int x = 0; x < roi_hsv.cols; ++x) {
      auto& hsv = roi_hsv.at<cv::Vec3b>(y, x);
      if (hsv[2] < config_.v_min_bright) continue;
      total++;
      votes[static_cast<size_t>(classify_pixel(hsv))]++;
    }
  }
  if (total == 0) return ColorClass::UNKNOWN;
  size_t best = static_cast<size_t>(ColorClass::UNKNOWN);
  int best_count = 0;
  for (size_t i = 0; i < votes.size(); ++i) {
    if (votes[i] > best_count) { best_count = votes[i]; best = i; }
  }
  float ratio = static_cast<float>(best_count) / total;
  if (consistency) *consistency = ratio;
  return (ratio >= config_.color_consistency_thresh)
             ? static_cast<ColorClass>(best) : ColorClass::UNKNOWN;
}

const ColorRange& ColorClassifier::get_range(ColorClass c) const {
  return ranges_[static_cast<size_t>(c)];
}

}  // namespace color_detect
