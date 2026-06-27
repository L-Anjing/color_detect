#include "color_detect/beacon_detector.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <chrono>

namespace color_detect {

BeaconDetector::BeaconDetector()
    : classifier_(config_), protocol_(config_), state_machine_(config_) {}
BeaconDetector::BeaconDetector(const BeaconConfig& config)
    : config_(config), classifier_(config),
      protocol_(config), state_machine_(config) {}

void BeaconDetector::set_config(const BeaconConfig& config) {
  config_ = config;
  classifier_ = ColorClassifier(config_);
  protocol_ = BeaconProtocol(config_);
  state_machine_ = BeaconStateMachine(config_);
}

void BeaconDetector::reset() { state_machine_.reset(); debug_overlay_ = cv::Mat(); }

BeaconState BeaconDetector::process_frame(const cv::Mat& bgr_frame) {
  if (bgr_frame.empty()) return BeaconState{};

  auto now_ms = static_cast<uint64_t>(
      std::chrono::duration_cast<std::chrono::milliseconds>(
          std::chrono::steady_clock::now().time_since_epoch()).count());

  auto hsv = preprocess(bgr_frame);
  auto candidates = find_candidates(bgr_frame, hsv);

  if (candidates.empty()) {
    BeaconState empty; empty.state = BeaconState::UNKNOWN;
    return state_machine_.update(empty, now_ms);
  }

  candidates = merge_candidates(candidates);
  auto* best = select_best(candidates);
  if (!best) {
    BeaconState empty; empty.state = BeaconState::UNKNOWN;
    return state_machine_.update(empty, now_ms);
  }

  BeaconState raw = protocol_.decode(best->segment_colors,
      best->rotated_rect.angle,
      best->rotated_rect.center.x, best->rotated_rect.center.y, best->area);
  raw.timestamp_ms = now_ms;

  auto filtered = state_machine_.update(raw, now_ms);
  if (config_.debug_output) draw_debug(bgr_frame, candidates, filtered);
  return filtered;
}

cv::Mat BeaconDetector::preprocess(const cv::Mat& bgr_frame) {
  cv::Mat frame = bgr_frame;
  if (bgr_frame.cols > config_.resize_width) {
    float s = static_cast<float>(config_.resize_width) / bgr_frame.cols;
    cv::resize(bgr_frame, frame, cv::Size(), s, s, cv::INTER_LINEAR);
  }
  cv::Mat hsv;
  cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);
  if (config_.gaussian_kernel > 0) {
    int k = config_.gaussian_kernel | 1;
    cv::GaussianBlur(hsv, hsv, cv::Size(k, k), 0);
  }
  return hsv;
}

std::vector<BeaconCandidate> BeaconDetector::find_candidates(
    const cv::Mat& bgr_frame, const cv::Mat& hsv) {
  cv::Mat v_channel, mask;
  cv::extractChannel(hsv, v_channel, 2);
  cv::threshold(v_channel, mask, config_.v_threshold, 255, cv::THRESH_BINARY);

  auto ok = cv::getStructuringElement(cv::MORPH_ELLIPSE,
      cv::Size(config_.morph_open_size, config_.morph_open_size));
  auto ck = cv::getStructuringElement(cv::MORPH_ELLIPSE,
      cv::Size(config_.morph_close_size, config_.morph_close_size));
  cv::morphologyEx(mask, mask, cv::MORPH_OPEN, ok);
  cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, ck);

  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

  int farea = bgr_frame.cols * bgr_frame.rows;
  int max_a = static_cast<int>(farea * config_.max_area_ratio);
  std::vector<BeaconCandidate> cands;

  for (auto& c : contours) {
    double a = cv::contourArea(c);
    if (a < config_.min_area_px || a > max_a) continue;
    std::vector<cv::Point> hull;
    cv::convexHull(c, hull);
    if (a / cv::contourArea(hull) < config_.min_solidity) continue;

    auto rr = cv::minAreaRect(c);
    float w = rr.size.width, hh = rr.size.height;
    if (w < 1 || hh < 1) continue;
    float asp = std::max(w, hh) / (std::min(w, hh) + 1e-6f);
    if (asp < config_.min_aspect_ratio) continue;

    BeaconCandidate bc;
    bc.rotated_rect = rr; bc.contour = c;
    bc.area = a; bc.aspect_ratio = asp;
    bc.solidity = a / cv::contourArea(hull);
    bc.segment_colors = segment_candidate(bc, hsv);
    cands.push_back(bc);
  }
  return cands;
}

std::vector<ColorClass> BeaconDetector::segment_candidate(
    const BeaconCandidate& cand, const cv::Mat& hsv) {
  const int N = config_.n_segments;
  std::vector<ColorClass> cols(N, ColorClass::UNKNOWN);
  if (N < 1) return cols;

  auto rr = cand.rotated_rect;
  auto sz = rr.size;
  auto ct = rr.center;
  float ang = rr.angle;

  cv::Point2f axis;
  if (sz.width >= sz.height) {
    axis = {std::cos(ang * CV_PI / 180), std::sin(ang * CV_PI / 180)};
  } else {
    axis = {std::cos((ang + 90) * CV_PI / 180), std::sin((ang + 90) * CV_PI / 180)};
    std::swap(sz.width, sz.height);
  }

  float seg = sz.width / N;
  cv::Point2f perp(-axis.y, axis.x);

  for (int i = 0; i < N; i++) {
    float t = (i + 0.5f) * seg - sz.width / 2;
    auto sc = ct + axis * t;
    cv::Point2f c4[4] = {
      sc + axis * (-seg/2) + perp * (sz.height/2),
      sc + axis * (seg/2) + perp * (sz.height/2),
      sc + axis * (seg/2) + perp * (-sz.height/2),
      sc + axis * (-seg/2) + perp * (-sz.height/2)
    };
    cv::Mat sm = cv::Mat::zeros(hsv.size(), CV_8UC1);
    std::vector<cv::Point> poly;
    for (auto& cp : c4) poly.push_back({(int)std::round(cp.x), (int)std::round(cp.y)});
    cv::fillConvexPoly(sm, poly, 255);
    cv::Mat sh; hsv.copyTo(sh, sm);
    float cons = 0;
    cols[i] = classifier_.classify_with_consistency(sh, &cons);
    if (cons < config_.color_consistency_thresh) cols[i] = ColorClass::UNKNOWN;
  }
  return cols;
}

std::vector<BeaconCandidate> BeaconDetector::merge_candidates(
    const std::vector<BeaconCandidate>& cands) {
  if (cands.size() <= 1) return cands;
  std::vector<BeaconCandidate> merged;
  std::vector<bool> used(cands.size(), false);
  for (size_t i = 0; i < cands.size(); i++) {
    if (used[i]) continue;
    auto com = cands[i];
    for (size_t j = i + 1; j < cands.size(); j++) {
      if (used[j]) continue;
      std::vector<cv::Point2f> inter;
      if (cv::rotatedRectangleIntersection(cands[i].rotated_rect,
            cands[j].rotated_rect, inter) != cv::INTERSECT_NONE) {
        if (cands[j].area > com.area) com = cands[j];
        used[j] = true;
      }
    }
    merged.push_back(com);
    used[i] = true;
  }
  return merged;
}

const BeaconCandidate* BeaconDetector::select_best(
    const std::vector<BeaconCandidate>& cands) const {
  if (cands.empty()) return nullptr;
  const BeaconCandidate* best = nullptr;
  double best_s = -std::numeric_limits<double>::max();
  for (auto& c : cands) {
    int vis = 0;
    for (auto col : c.segment_colors)
      if (col != ColorClass::UNKNOWN) vis++;
    double s = c.area * c.aspect_ratio * (vis / (double)config_.n_segments);
    if (s > best_s) { best_s = s; best = &c; }
  }
  return best;
}

void BeaconDetector::draw_debug(const cv::Mat& frame,
    const std::vector<BeaconCandidate>& cands, const BeaconState& state) {
  debug_overlay_ = frame.clone();
  for (auto& c : cands) {
    cv::Point2f v[4]; c.rotated_rect.points(v);
    for (int i = 0; i < 4; i++)
      cv::line(debug_overlay_, v[i], v[(i+1)%4], cv::Scalar(0,255,0), 2);
  }
  std::string info = std::string(BeaconState::state_name(state.state))
      + " c:" + std::to_string(state.confidence).substr(0,4)
      + " p:" + (state.is_partial ? "Y" : "N");
  cv::putText(debug_overlay_, info, {30,30},
      cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0,0,255), 2);
}

}  // namespace color_detect
