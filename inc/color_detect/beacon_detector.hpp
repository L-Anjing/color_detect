#ifndef COLOR_DETECT_BEACON_DETECTOR_HPP_
#define COLOR_DETECT_BEACON_DETECTOR_HPP_

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <vector>
#include <memory>

#include "color_detect/beacon_config.hpp"
#include "color_detect/color_classifier.hpp"
#include "color_detect/beacon_protocol.hpp"

namespace color_detect {

struct SegmentHsvStats {
  ColorClass classified_color = ColorClass::UNKNOWN;
  float confidence = 0.0f;
  bool valid = false;
  int pixel_count = 0;
  int h_min = 0;
  int h_max = 0;
  int s_min = 0;
  int s_max = 0;
  int v_min = 0;
  int v_max = 0;
  float h_mean = 0.0f;
  float s_mean = 0.0f;
  float v_mean = 0.0f;
};

struct BeaconCandidate {
  cv::RotatedRect rotated_rect;
  std::vector<cv::Point> contour;
  double area = 0.0;
  double area_ratio = 0.0;
  double aspect_ratio = 0.0;
  double rect_fill_ratio = 0.0;
  double mean_brightness = 0.0;
  double score = 0.0;
  std::vector<ColorClass> segment_colors;
  std::vector<SegmentHsvStats> segment_hsv_stats;
  int color_run_count = 0;
};

class BeaconDetector {
public:
  BeaconDetector();
  explicit BeaconDetector(const BeaconConfig& config);

  BeaconState process_frame(const cv::Mat& bgr_frame);
  cv::Mat debug_overlay() const { return debug_overlay_; }
  std::string latest_hsv_summary() const { return latest_hsv_summary_; }
  void set_config(const BeaconConfig& config);
  void reset();

private:
  BeaconConfig config_;
  ColorClassifier classifier_;
  BeaconProtocol protocol_;
  cv::Mat debug_overlay_;
  std::string latest_hsv_summary_;

  cv::Mat preprocess(const cv::Mat& bgr_frame);
  cv::Mat enhance_roi_hsv(const cv::Mat& roi_hsv) const;
  std::vector<BeaconCandidate> find_candidates(const cv::Mat& bgr_frame,
                                                const cv::Mat& hsv);
  std::vector<ColorClass> segment_candidate(const BeaconCandidate& candidate,
                                             const cv::Mat& hsv,
                                             std::vector<SegmentHsvStats>* segment_hsv_stats);
  int count_color_runs(const BeaconCandidate& candidate,
                       const cv::Mat& hsv) const;
  BeaconCandidate build_candidate(const std::vector<cv::Point>& contour) const;
  bool should_merge_candidates(const BeaconCandidate& lhs,
                               const BeaconCandidate& rhs) const;
  bool is_strip_like(double contour_area,
                     const cv::RotatedRect& rect,
                     double* aspect_ratio,
                     double* rect_fill_ratio) const;
  std::string build_hsv_summary(const BeaconCandidate& candidate) const;
  std::vector<BeaconCandidate> merge_candidates(
      const std::vector<BeaconCandidate>& candidates);
  const BeaconCandidate* select_best(
      const std::vector<BeaconCandidate>& candidates) const;
  void draw_debug(const cv::Mat& frame,
                  const cv::Mat& hsv,
                  const std::vector<BeaconCandidate>& candidates,
                  const BeaconCandidate* best,
                  const BeaconState& state);
};

}  // namespace color_detect

#endif  // COLOR_DETECT_BEACON_DETECTOR_HPP_
