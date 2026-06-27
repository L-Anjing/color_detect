#ifndef COLOR_DETECT_BEACON_DETECTOR_HPP_
#define COLOR_DETECT_BEACON_DETECTOR_HPP_

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <vector>
#include <memory>

#include "color_detect/beacon_config.hpp"
#include "color_detect/color_classifier.hpp"
#include "color_detect/beacon_protocol.hpp"
#include "color_detect/state_machine.hpp"

namespace color_detect {

struct BeaconCandidate {
  cv::RotatedRect rotated_rect;
  std::vector<cv::Point> contour;
  double area = 0.0;
  double aspect_ratio = 0.0;
  double solidity = 0.0;
  std::vector<ColorClass> segment_colors;
};

class BeaconDetector {
public:
  BeaconDetector();
  explicit BeaconDetector(const BeaconConfig& config);

  BeaconState process_frame(const cv::Mat& bgr_frame);
  cv::Mat debug_overlay() const { return debug_overlay_; }
  void set_config(const BeaconConfig& config);
  void reset();

private:
  BeaconConfig config_;
  ColorClassifier classifier_;
  BeaconProtocol protocol_;
  BeaconStateMachine state_machine_;
  cv::Mat debug_overlay_;

  cv::Mat preprocess(const cv::Mat& bgr_frame);
  std::vector<BeaconCandidate> find_candidates(const cv::Mat& bgr_frame,
                                                const cv::Mat& hsv);
  std::vector<ColorClass> segment_candidate(const BeaconCandidate& candidate,
                                             const cv::Mat& hsv);
  std::vector<BeaconCandidate> merge_candidates(
      const std::vector<BeaconCandidate>& candidates);
  const BeaconCandidate* select_best(
      const std::vector<BeaconCandidate>& candidates) const;
  void draw_debug(const cv::Mat& frame,
                  const std::vector<BeaconCandidate>& candidates,
                  const BeaconState& state);
};

}  // namespace color_detect

#endif  // COLOR_DETECT_BEACON_DETECTOR_HPP_
