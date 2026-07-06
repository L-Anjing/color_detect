#include "color_detect/color_classifier.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace color_detect {

namespace {

int clamp_hue(int value) {
  return std::clamp(value, 0, 180);
}

int clamp_sv(int value) {
  return std::clamp(value, 0, 255);
}

bool hue_in_range(int hue, const ColorRange& range) {
  if (range.h_low <= range.h_high) {
    return hue >= range.h_low && hue <= range.h_high;
  }
  return hue >= range.h_low || hue <= range.h_high;
}

bool in_range(const cv::Vec3b& hsv, const ColorRange& range) {
  const int h = hsv[0];
  const int s = hsv[1];
  const int v = hsv[2];
  return hue_in_range(h, range) &&
         s >= range.s_low && s <= range.s_high &&
         v >= range.v_low && v <= range.v_high;
}

float hue_range_center(const ColorRange& range) {
  if (range.h_low <= range.h_high) {
    return 0.5f * static_cast<float>(range.h_low + range.h_high);
  }
  const float span = static_cast<float>((180 - range.h_low) + range.h_high);
  float center = static_cast<float>(range.h_low) + 0.5f * span;
  if (center >= 180.0f) {
    center -= 180.0f;
  }
  return center;
}

float hue_range_span(const ColorRange& range) {
  if (range.h_low <= range.h_high) {
    return static_cast<float>(range.h_high - range.h_low);
  }
  return static_cast<float>((180 - range.h_low) + range.h_high);
}

float circular_hue_distance(float lhs, float rhs) {
  float dist = std::abs(lhs - rhs);
  if (dist > 90.0f) {
    dist = 180.0f - dist;
  }
  return dist;
}

}  // namespace

ColorClassifier::ColorClassifier(const BeaconConfig& config) : config_(config) {
  build_ranges();
}

void ColorClassifier::build_ranges() {
  const int ht = config_.hue_tolerance;
  const int smin = config_.s_min_for_color;
  const int vmin = config_.v_min_bright;

  ranges_[static_cast<size_t>(ColorClass::WHITE)] = {
      0, 180, 0, config_.s_threshold_white, config_.v_threshold, 255};
  ranges_[static_cast<size_t>(ColorClass::BLUE)] = {
      clamp_hue(92 - ht / 2), clamp_hue(130 + ht / 2), smin, 255, vmin, 255};
  ranges_[static_cast<size_t>(ColorClass::CYAN)] = {
      clamp_hue(75 - ht / 2), clamp_hue(95 + ht / 2), smin, 255, vmin, 255};
  ranges_[static_cast<size_t>(ColorClass::YELLOW)] = {
      clamp_hue(config_.yellow_h_low), clamp_hue(config_.yellow_h_high),
      clamp_sv(config_.yellow_s_low), 255, clamp_sv(config_.yellow_v_low), 255};
  ranges_[static_cast<size_t>(ColorClass::MAGENTA)] = {
      clamp_hue(140 - ht / 2), clamp_hue(150 + ht / 2), smin, 255, vmin, 255};
  ranges_[static_cast<size_t>(ColorClass::GREEN)] = {
      clamp_hue(config_.green_h_low), clamp_hue(config_.green_h_high),
      clamp_sv(config_.green_s_low), 255, clamp_sv(config_.green_v_low), 255};
  ranges_[static_cast<size_t>(ColorClass::RED)] = {
      clamp_hue(config_.red_h_low), clamp_hue(config_.red_h_high),
      clamp_sv(config_.red_s_low), 255, clamp_sv(config_.red_v_low), 255};
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
    if (votes[i] > best_count) {
      best_count = votes[i];
      best = i;
    }
  }
  return (static_cast<float>(best_count) / total > 0.4f)
             ? static_cast<ColorClass>(best)
             : ColorClass::UNKNOWN;
}

ColorClass ColorClassifier::classify_pixel(const cv::Vec3b& hsv) const {
  const uint8_t h = hsv[0];
  const uint8_t s = hsv[1];
  const uint8_t v = hsv[2];
  if (v < config_.v_min_bright) return ColorClass::UNKNOWN;
  if (s < config_.s_threshold_white) {
    return (v >= config_.v_threshold) ? ColorClass::WHITE : ColorClass::UNKNOWN;
  }

  ColorClass best = ColorClass::UNKNOWN;
  float best_dist = std::numeric_limits<float>::max();
  float best_span = std::numeric_limits<float>::max();
  for (size_t i = static_cast<size_t>(ColorClass::BLUE);
       i <= static_cast<size_t>(ColorClass::RED); ++i) {
    const auto& range = ranges_[i];
    if (!in_range(hsv, range)) {
      continue;
    }

    const float dist =
        circular_hue_distance(static_cast<float>(h), hue_range_center(range));
    const float span = hue_range_span(range);
    if (dist < best_dist || (std::abs(dist - best_dist) < 1e-3f && span < best_span)) {
      best = static_cast<ColorClass>(i);
      best_dist = dist;
      best_span = span;
    }
  }
  return best;
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
    if (votes[i] > best_count) {
      best_count = votes[i];
      best = i;
    }
  }
  const float ratio = static_cast<float>(best_count) / total;
  if (consistency) *consistency = ratio;
  return (ratio >= config_.color_consistency_thresh)
             ? static_cast<ColorClass>(best)
             : ColorClass::UNKNOWN;
}

const ColorRange& ColorClassifier::get_range(ColorClass c) const {
  return ranges_[static_cast<size_t>(c)];
}

ColorClass ColorClassifier::classify_hue_histogram(
    const cv::Mat& roi_hsv, float* confidence) const {
  if (roi_hsv.empty()) return ColorClass::UNKNOWN;

  const int bins = 36;
  std::array<int, bins> hist{};
  int total = 0;

  for (int y = 0; y < roi_hsv.rows; ++y) {
    for (int x = 0; x < roi_hsv.cols; ++x) {
      auto& p = roi_hsv.at<cv::Vec3b>(y, x);
      const uint8_t h = p[0];
      const uint8_t s = p[1];
      const uint8_t v = p[2];
      if (s < config_.s_min_for_color) continue;
      if (v < config_.v_min_bright) continue;
      int bin = (h * bins) / 180;
      if (bin >= bins) bin = bins - 1;
      hist[bin]++;
      total++;
    }
  }

  if (total == 0) {
    if (confidence) *confidence = 0.0f;
    return ColorClass::UNKNOWN;
  }

  int peak_bin = 0;
  int peak_count = 0;
  for (int i = 0; i < bins; ++i) {
    if (hist[i] > peak_count) {
      peak_count = hist[i];
      peak_bin = i;
    }
  }

  const float peak_hue = (peak_bin + 0.5f) * (180.0f / bins);

  float best_dist = std::numeric_limits<float>::max();
  ColorClass best_color = ColorClass::UNKNOWN;
  for (size_t i = static_cast<size_t>(ColorClass::BLUE);
       i <= static_cast<size_t>(ColorClass::RED); ++i) {
    const auto& range = ranges_[i];
    const float dist = circular_hue_distance(peak_hue, hue_range_center(range));
    if (dist < best_dist) {
      best_dist = dist;
      best_color = static_cast<ColorClass>(i);
    }
  }

  const float conf = static_cast<float>(peak_count) / total;
  if (confidence) *confidence = conf;

  const auto& range = ranges_[static_cast<size_t>(best_color)];
  const float half_range = hue_range_span(range) / 2.0f + 5.0f;
  if (conf >= config_.color_consistency_thresh && best_dist <= half_range) {
    return best_color;
  }

  return ColorClass::UNKNOWN;
}

}  // namespace color_detect
