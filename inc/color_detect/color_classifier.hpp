#ifndef COLOR_DETECT_COLOR_CLASSIFIER_HPP_
#define COLOR_DETECT_COLOR_CLASSIFIER_HPP_

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <array>
#include <cstdint>

#include "color_detect/beacon_config.hpp"

namespace color_detect {

enum class ColorClass : uint8_t {
  UNKNOWN = 0,
  WHITE,
  CYAN,
  YELLOW,
  MAGENTA,
  GREEN,
  RED,
  COUNT
};

struct ColorRange {
  int h_low, h_high;
  int s_low, s_high;
  int v_low, v_high;
};

class ColorClassifier {
public:
  explicit ColorClassifier(const BeaconConfig& config);

  ColorClass classify(const cv::Mat& roi_hsv,
                      const cv::Mat& roi_mask = cv::Mat()) const;
  ColorClass classify_pixel(const cv::Vec3b& hsv) const;
  ColorClass classify_with_consistency(const cv::Mat& roi_hsv,
                                       float* consistency = nullptr) const;
  /// @brief 基于 Hue 直方图峰值分类（比像素投票更抗噪）
  ColorClass classify_hue_histogram(const cv::Mat& roi_hsv,
                                    float* confidence = nullptr) const;
  const ColorRange& get_range(ColorClass c) const;

private:
  BeaconConfig config_;
  std::array<ColorRange, static_cast<size_t>(ColorClass::COUNT)> ranges_;
  void build_ranges();
};

}  // namespace color_detect

#endif  // COLOR_DETECT_COLOR_CLASSIFIER_HPP_
