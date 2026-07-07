#include "color_detect/beacon_detector.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <sstream>

namespace color_detect {

namespace {

const char* color_name(ColorClass color) {
  switch (color) {
    case ColorClass::WHITE:
      return "WHITE";
    case ColorClass::BLUE:
      return "BLUE";
    case ColorClass::CYAN:
      return "CYAN";
    case ColorClass::YELLOW:
      return "YELLOW";
    case ColorClass::MAGENTA:
      return "MAGENTA";
    case ColorClass::GREEN:
      return "GREEN";
    case ColorClass::RED:
      return "RED";
    case ColorClass::UNKNOWN:
    default:
      return "UNKNOWN";
  }
}

bool is_reportable_color(ColorClass color) {
  return color == ColorClass::YELLOW ||
         color == ColorClass::RED ||
         color == ColorClass::GREEN;
}

int reportable_color_count(const BeaconCandidate& candidate) {
  int count = 0;
  for (const auto& stats : candidate.segment_hsv_stats) {
    if (is_reportable_color(stats.classified_color)) {
      ++count;
    }
  }
  return count;
}

bool has_stop_segment(const BeaconCandidate& candidate) {
  for (auto color : candidate.segment_colors) {
    if (color == ColorClass::RED) {
      return true;
    }
  }
  return false;
}

ColorClass classify_mean_hsv(float h, float s, float v) {
  if (v < 40.0f) {
    return ColorClass::UNKNOWN;
  }
  if (s < 35.0f) {
    return (v > 90.0f) ? ColorClass::WHITE : ColorClass::UNKNOWN;
  }

  const float hue = std::fmod(h + 180.0f, 180.0f);
  if (hue < 8.0f || hue >= 172.0f) {
    return ColorClass::RED;
  }
  if (hue < 20.0f) {
    return ColorClass::RED;
  }
  if (hue < 35.0f) {
    return ColorClass::YELLOW;
  }
  if (hue < 85.0f) {
    return ColorClass::GREEN;
  }
  if (hue < 100.0f) {
    return ColorClass::CYAN;
  }
  if (hue < 135.0f) {
    return ColorClass::BLUE;
  }
  return ColorClass::MAGENTA;
}

cv::Point2f long_axis(const cv::RotatedRect& rect) {
  auto size = rect.size;
  const float angle_deg =
      (size.width >= size.height) ? rect.angle : (rect.angle + 90.0f);
  if (size.width < size.height) {
    std::swap(size.width, size.height);
  }
  const float angle_rad = angle_deg * static_cast<float>(CV_PI / 180.0);
  return {std::cos(angle_rad), std::sin(angle_rad)};
}

float normalized_angle_diff_deg(float lhs, float rhs) {
  float diff = std::abs(lhs - rhs);
  while (diff >= 180.0f) {
    diff -= 180.0f;
  }
  if (diff > 90.0f) {
    diff = 180.0f - diff;
  }
  return diff;
}

}  // namespace

BeaconDetector::BeaconDetector()
    : classifier_(config_), protocol_(config_) {}

BeaconDetector::BeaconDetector(const BeaconConfig& config)
    : config_(config),
      classifier_(config),
      protocol_(config) {}

void BeaconDetector::set_config(const BeaconConfig& config) {
  config_ = config;
  classifier_ = ColorClassifier(config_);
  protocol_ = BeaconProtocol(config_);
}

void BeaconDetector::reset() {
  debug_overlay_ = cv::Mat();
  latest_hsv_summary_.clear();
}

BeaconState BeaconDetector::process_frame(const cv::Mat& bgr_frame) {
  if (bgr_frame.empty()) {
    return BeaconState{};
  }

  const auto now_ms = static_cast<uint64_t>(
      std::chrono::duration_cast<std::chrono::milliseconds>(
          std::chrono::steady_clock::now().time_since_epoch())
          .count());

  auto hsv = preprocess(bgr_frame);
  auto candidates = merge_candidates(find_candidates(bgr_frame, hsv));

  for (auto& candidate : candidates) {
    candidate.segment_colors =
        segment_candidate(candidate, hsv, &candidate.segment_hsv_stats);
    candidate.color_run_count = count_color_runs(candidate, hsv);
  }

  if (candidates.empty()) {
    latest_hsv_summary_.clear();
    BeaconState raw;
    raw.state = BeaconState::GO;
    raw.confidence = 0.60f;
    raw.timestamp_ms = now_ms;
    if (config_.debug_output) {
      draw_debug(bgr_frame, hsv, candidates, nullptr, raw);
    }
    return raw;
  }

  const BeaconCandidate* hsv_best = nullptr;
  double hsv_best_score = -std::numeric_limits<double>::max();
  for (const auto& candidate : candidates) {
    const int reportable_count = reportable_color_count(candidate);
    if (reportable_count <= 0) {
      continue;
    }
    const double score =
        reportable_count * 1000.0 + candidate.area + candidate.aspect_ratio * 10.0;
    if (score > hsv_best_score) {
      hsv_best_score = score;
      hsv_best = &candidate;
    }
  }
  latest_hsv_summary_ = hsv_best ? build_hsv_summary(*hsv_best) : std::string();

  const auto* best = select_best(candidates);
  const BeaconCandidate* wait_best = nullptr;
  double wait_best_score = -std::numeric_limits<double>::max();
  for (const auto& candidate : candidates) {
    if (!has_stop_segment(candidate)) {
      continue;
    }
    if (candidate.score > wait_best_score) {
      wait_best_score = candidate.score;
      wait_best = &candidate;
    }
  }
  if (wait_best != nullptr) {
    best = wait_best;
  }
  if (!best) {
    BeaconState raw;
    raw.state = BeaconState::GO;
    raw.confidence = 0.70f;
    raw.timestamp_ms = now_ms;
    if (config_.debug_output) {
      draw_debug(bgr_frame, hsv, candidates, nullptr, raw);
    }
    return raw;
  }

  BeaconState raw = protocol_.decode(
      best->segment_colors, best->rotated_rect.angle,
      best->rotated_rect.center.x, best->rotated_rect.center.y,
      std::max(best->rotated_rect.size.width, best->rotated_rect.size.height),
      best->area);

  if (hsv.cols > 0 && hsv.rows > 0) {
    const float scale_x = static_cast<float>(bgr_frame.cols) / hsv.cols;
    const float scale_y = static_cast<float>(bgr_frame.rows) / hsv.rows;
    raw.beacon_center_x *= scale_x;
    raw.beacon_center_y *= scale_y;
    raw.beacon_length_px *= (scale_x + scale_y) * 0.5f;
  }
  raw.timestamp_ms = now_ms;

  if (config_.debug_output) {
    draw_debug(bgr_frame, hsv, candidates,
               raw.state == BeaconState::WAIT ? best : nullptr, raw);
  }
  return raw;
}

cv::Mat BeaconDetector::preprocess(const cv::Mat& bgr_frame) {
  cv::Mat frame = bgr_frame;
  if (bgr_frame.cols > config_.resize_width) {
    const float scale = static_cast<float>(config_.resize_width) / bgr_frame.cols;
    cv::resize(bgr_frame, frame, cv::Size(), scale, scale, cv::INTER_LINEAR);
  }

  cv::Mat hsv;
  cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);

  if (config_.use_clahe) {
    // CLAHE is reserved for ROI color enhancement, not global candidate search.
  }

  return hsv;
}

cv::Mat BeaconDetector::enhance_roi_hsv(const cv::Mat& roi_hsv) const {
  if (roi_hsv.empty()) {
    return roi_hsv;
  }

  cv::Mat enhanced = roi_hsv.clone();
  std::vector<cv::Mat> channels;
  cv::split(enhanced, channels);

  if (config_.saturation_gain > 0.0f &&
      std::abs(config_.saturation_gain - 1.0f) > 1e-3f) {
    channels[1].convertTo(channels[1], -1, config_.saturation_gain, 0);
  }
  if (config_.value_gain > 0.0f &&
      std::abs(config_.value_gain - 1.0f) > 1e-3f) {
    channels[2].convertTo(channels[2], -1, config_.value_gain, 0);
  }

  if (config_.use_clahe) {
    auto clahe = cv::createCLAHE(
        config_.clahe_clip_limit,
        cv::Size(config_.clahe_grid_size, config_.clahe_grid_size));
    clahe->apply(channels[2], channels[2]);
  }

  cv::merge(channels, enhanced);
  if (config_.gaussian_kernel > 0) {
    const int kernel = config_.gaussian_kernel | 1;
    cv::GaussianBlur(enhanced, enhanced, cv::Size(kernel, kernel), 0);
  }
  return enhanced;
}

std::vector<BeaconCandidate> BeaconDetector::find_candidates(
    const cv::Mat& bgr_frame, const cv::Mat& hsv) {
  (void)bgr_frame;

  std::vector<cv::Mat> channels;
  cv::split(hsv, channels);
  cv::Mat v_ch = channels[2];

  cv::Mat blurred;
  cv::GaussianBlur(v_ch, blurred, cv::Size(5, 5), 0);
  double local_max = 0.0;
  cv::minMaxLoc(blurred, nullptr, &local_max);

  cv::Mat otsu_mask;
  cv::threshold(blurred, otsu_mask, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);

  const int dynamic_threshold = std::max(
      10, std::min(config_.candidate_bright_threshold,
                   static_cast<int>(local_max) - config_.candidate_threshold_delta));
  cv::Mat dyn_mask;
  cv::threshold(blurred, dyn_mask, dynamic_threshold, 255, cv::THRESH_BINARY);

  cv::Mat red_mask;
  const int red_candidate_v_low = std::max(1, config_.red_v_floor);
  const int red_candidate_s_low = std::max(20, config_.red_s_low);
  if (config_.red_h_low > config_.red_h_high) {
    cv::Mat red_low;
    cv::Mat red_high;
    cv::inRange(hsv, cv::Scalar(0, red_candidate_s_low, red_candidate_v_low),
                cv::Scalar(config_.red_h_high, 255, 255), red_low);
    cv::inRange(hsv, cv::Scalar(config_.red_h_low, red_candidate_s_low, red_candidate_v_low),
                cv::Scalar(179, 255, 255), red_high);
    red_mask = red_low | red_high;
  } else {
    cv::inRange(hsv, cv::Scalar(config_.red_h_low, red_candidate_s_low, red_candidate_v_low),
                cv::Scalar(config_.red_h_high, 255, 255), red_mask);
  }

  cv::Mat mask = otsu_mask | dyn_mask | red_mask;

  double search_area = static_cast<double>(mask.rows) * mask.cols;
  if (config_.use_roi) {
    const int x0 = std::clamp(static_cast<int>(mask.cols * config_.roi_x_min_ratio), 0,
                              mask.cols);
    const int x1 = std::clamp(static_cast<int>(mask.cols * config_.roi_x_max_ratio), 0,
                              mask.cols);
    const int y0 = std::clamp(static_cast<int>(mask.rows * config_.roi_y_min_ratio), 0,
                              mask.rows);
    const int y1 = std::clamp(static_cast<int>(mask.rows * config_.roi_y_max_ratio), 0,
                              mask.rows);
    cv::Mat roi_mask = cv::Mat::zeros(mask.size(), CV_8UC1);
    if (x1 > x0 && y1 > y0) {
      roi_mask(cv::Rect(x0, y0, x1 - x0, y1 - y0)).setTo(255);
      search_area = static_cast<double>(x1 - x0) * (y1 - y0);
    }
    mask &= roi_mask;
  }
  search_area = std::max(1.0, search_area);

  const auto open_kernel = cv::getStructuringElement(
      cv::MORPH_RECT,
      cv::Size(config_.morph_open_size, config_.morph_open_size));
  const auto close_kernel = cv::getStructuringElement(
      cv::MORPH_RECT,
      cv::Size(config_.morph_close_size, config_.morph_close_size));
  const auto dilate_kernel = cv::getStructuringElement(
      cv::MORPH_RECT,
      cv::Size(config_.dilation_size, config_.dilation_size));

  cv::morphologyEx(mask, mask, cv::MORPH_OPEN, open_kernel);
  cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, close_kernel, cv::Point(-1, -1),
                   std::max(1, config_.morph_close_iterations));
  cv::dilate(mask, mask, dilate_kernel, cv::Point(-1, -1),
             std::max(1, config_.morph_dilate_iterations));

  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

  std::vector<BeaconCandidate> candidates;
  for (const auto& contour : contours) {
    const double area = cv::contourArea(contour);
    if (area < config_.min_contour_area_px) {
      continue;
    }
    const double area_ratio = area / search_area;
    if (area_ratio < config_.candidate_min_area_ratio) {
      continue;
    }

    const auto rect = cv::minAreaRect(contour);
    if (rect.size.width < 1.0f || rect.size.height < 1.0f) {
      continue;
    }
    double aspect_ratio = 0.0;
    double rect_fill_ratio = 0.0;
    if (!is_strip_like(area, rect, &aspect_ratio, &rect_fill_ratio)) {
      continue;
    }
    if (aspect_ratio < config_.candidate_min_aspect_ratio ||
        rect_fill_ratio < config_.candidate_min_fill_ratio) {
      continue;
    }

    const cv::Rect image_rect(0, 0, mask.cols, mask.rows);
    const cv::Rect bbox = cv::boundingRect(contour) & image_rect;
    if (bbox.empty()) {
      continue;
    }

    std::vector<cv::Point> shifted_contour;
    shifted_contour.reserve(contour.size());
    for (const auto& point : contour) {
      shifted_contour.emplace_back(point.x - bbox.x, point.y - bbox.y);
    }
    cv::Mat contour_mask = cv::Mat::zeros(bbox.size(), CV_8UC1);
    cv::drawContours(contour_mask, std::vector<std::vector<cv::Point>>{shifted_contour},
                     -1, 255, cv::FILLED);
    const double mean_brightness = cv::mean(blurred(bbox), contour_mask)[0];

    BeaconCandidate candidate;
    candidate.rotated_rect = rect;
    candidate.contour = contour;
    candidate.area = area;
    candidate.area_ratio = area_ratio;
    candidate.aspect_ratio = aspect_ratio;
    candidate.rect_fill_ratio = rect_fill_ratio;
    candidate.mean_brightness = mean_brightness;
    candidate.score = area_ratio * aspect_ratio * rect_fill_ratio * mean_brightness;
    candidates.push_back(candidate);
  }

  return candidates;
}

std::vector<ColorClass> BeaconDetector::segment_candidate(
    const BeaconCandidate& cand, const cv::Mat& hsv,
    std::vector<SegmentHsvStats>* segment_hsv_stats) {
  const int segment_count = config_.n_segments;
  std::vector<ColorClass> colors(segment_count, ColorClass::UNKNOWN);
  if (segment_hsv_stats) {
    segment_hsv_stats->assign(segment_count, SegmentHsvStats{});
  }
  if (segment_count < 1) {
    return colors;
  }

  auto rect = cand.rotated_rect;
  auto size = rect.size;
  const auto center = rect.center;
  const float angle = rect.angle;

  cv::Point2f axis;
  if (size.width >= size.height) {
    axis = {static_cast<float>(std::cos(angle * CV_PI / 180.0)),
            static_cast<float>(std::sin(angle * CV_PI / 180.0))};
  } else {
    axis = {static_cast<float>(std::cos((angle + 90.0f) * CV_PI / 180.0)),
            static_cast<float>(std::sin((angle + 90.0f) * CV_PI / 180.0))};
    std::swap(size.width, size.height);
  }

  const float segment_width = size.width / segment_count;
  const cv::Point2f perp(-axis.y, axis.x);

  for (int i = 0; i < segment_count; ++i) {
    const float offset = (i + 0.5f) * segment_width - size.width / 2.0f;
    const auto sample_center = center + axis * offset;
    const float sample_height = std::max(6.0f, size.height * config_.halo_sample_scale);

    cv::Point2f corners[4] = {
        sample_center + axis * (-segment_width / 2.0f) + perp * (sample_height / 2.0f),
        sample_center + axis * (segment_width / 2.0f) + perp * (sample_height / 2.0f),
        sample_center + axis * (segment_width / 2.0f) - perp * (sample_height / 2.0f),
        sample_center - axis * (segment_width / 2.0f) - perp * (sample_height / 2.0f)};

    cv::Mat sample_mask = cv::Mat::zeros(hsv.size(), CV_8UC1);
    std::vector<cv::Point> polygon;
    polygon.reserve(4);
    for (const auto& corner : corners) {
      polygon.push_back(
          {static_cast<int>(std::round(corner.x)), static_cast<int>(std::round(corner.y))});
    }
    cv::fillConvexPoly(sample_mask, polygon, 255);

    cv::Mat sample_hsv;
    hsv.copyTo(sample_hsv, sample_mask);
    sample_hsv = enhance_roi_hsv(sample_hsv);

    cv::Mat color_mask;
    cv::inRange(sample_hsv, cv::Scalar(0, 25, 50), cv::Scalar(179, 255, 255), color_mask);
    cv::bitwise_and(color_mask, sample_mask, color_mask);

    std::vector<cv::Mat> sample_channels;
    cv::split(sample_hsv, sample_channels);
    double roi_v_max = 0.0;
    cv::minMaxLoc(sample_channels[2], nullptr, &roi_v_max, nullptr, nullptr, sample_mask);
    const int adaptive_red_v_low = std::clamp(
        static_cast<int>(roi_v_max) - 35,
        std::max(1, config_.red_v_floor),
        std::max(std::max(1, config_.red_v_floor), config_.red_v_low / 2));

    cv::Mat red_mask;
    const int red_candidate_v_low = adaptive_red_v_low;
    const int red_candidate_s_low = std::max(20, config_.red_s_low);
    if (config_.red_h_low > config_.red_h_high) {
      cv::Mat red_low;
      cv::Mat red_high;
      cv::inRange(sample_hsv, cv::Scalar(0, red_candidate_s_low, red_candidate_v_low),
                  cv::Scalar(config_.red_h_high, 255, 255), red_low);
      cv::inRange(sample_hsv,
                  cv::Scalar(config_.red_h_low, red_candidate_s_low, red_candidate_v_low),
                  cv::Scalar(179, 255, 255), red_high);
      red_mask = red_low | red_high;
    } else {
      cv::inRange(sample_hsv,
                  cv::Scalar(config_.red_h_low, red_candidate_s_low, red_candidate_v_low),
                  cv::Scalar(config_.red_h_high, 255, 255), red_mask);
    }
    cv::bitwise_and(red_mask, sample_mask, red_mask);
    const int sample_pixels = std::max(1, cv::countNonZero(sample_mask));
    const int red_pixels = cv::countNonZero(red_mask);
    const float red_ratio = red_pixels / static_cast<float>(sample_pixels);

    cv::Scalar mean_hsv;
    int valid_pixels = cv::countNonZero(color_mask);
    if (valid_pixels > 0) {
      mean_hsv = cv::mean(sample_hsv, color_mask);
    } else {
      mean_hsv = cv::mean(sample_hsv, sample_mask);
    }

    float confidence = 0.0f;
    if (red_pixels >= 3 && red_ratio >= config_.red_min_ratio) {
      colors[i] = ColorClass::RED;
      confidence = std::min(1.0f, red_ratio * 8.0f);
    } else {
      colors[i] = classify_mean_hsv(static_cast<float>(mean_hsv[0]),
                                    static_cast<float>(mean_hsv[1]),
                                    static_cast<float>(mean_hsv[2]));
    }
    if (valid_pixels > 0) {
      confidence = std::max(confidence,
                            std::min(1.0f, valid_pixels /
                                               static_cast<float>(sample_pixels)));
    }

    if (segment_hsv_stats) {
      auto& stats = (*segment_hsv_stats)[i];
      stats.classified_color = colors[i];
      stats.confidence = confidence;

      int h_min = 255;
      int h_max = 0;
      int s_min = 255;
      int s_max = 0;
      int v_min = 255;
      int v_max = 0;
      int pixel_count = 0;
      double h_sum = 0.0;
      double s_sum = 0.0;
      double v_sum = 0.0;

      for (int y = 0; y < sample_hsv.rows; ++y) {
        for (int x = 0; x < sample_hsv.cols; ++x) {
          if (sample_mask.at<uint8_t>(y, x) == 0) {
            continue;
          }
          const auto& pixel = sample_hsv.at<cv::Vec3b>(y, x);
          if (pixel[1] < config_.s_min_for_color || pixel[2] < config_.v_min_bright) {
            continue;
          }
          h_min = std::min(h_min, static_cast<int>(pixel[0]));
          h_max = std::max(h_max, static_cast<int>(pixel[0]));
          s_min = std::min(s_min, static_cast<int>(pixel[1]));
          s_max = std::max(s_max, static_cast<int>(pixel[1]));
          v_min = std::min(v_min, static_cast<int>(pixel[2]));
          v_max = std::max(v_max, static_cast<int>(pixel[2]));
          h_sum += pixel[0];
          s_sum += pixel[1];
          v_sum += pixel[2];
          ++pixel_count;
        }
      }

      stats.pixel_count = pixel_count;
      stats.valid = pixel_count > 0;
      if (stats.valid) {
        stats.h_min = h_min;
        stats.h_max = h_max;
        stats.s_min = s_min;
        stats.s_max = s_max;
        stats.v_min = v_min;
        stats.v_max = v_max;
        stats.h_mean = static_cast<float>(h_sum / pixel_count);
        stats.s_mean = static_cast<float>(s_sum / pixel_count);
        stats.v_mean = static_cast<float>(v_sum / pixel_count);
      }
    }
  }

  return colors;
}

int BeaconDetector::count_color_runs(const BeaconCandidate& cand,
                                     const cv::Mat& hsv) const {
  const int sample_count = std::max(config_.n_segments * 3, 6);
  if (sample_count <= 0) {
    return 0;
  }

  auto rect = cand.rotated_rect;
  auto size = rect.size;
  const auto center = rect.center;
  const float angle = rect.angle;

  cv::Point2f axis;
  if (size.width >= size.height) {
    axis = {static_cast<float>(std::cos(angle * CV_PI / 180.0)),
            static_cast<float>(std::sin(angle * CV_PI / 180.0))};
  } else {
    axis = {static_cast<float>(std::cos((angle + 90.0f) * CV_PI / 180.0)),
            static_cast<float>(std::sin((angle + 90.0f) * CV_PI / 180.0))};
    std::swap(size.width, size.height);
  }

  const float sample_width = size.width / sample_count;
  const cv::Point2f perp(-axis.y, axis.x);
  std::vector<ColorClass> sampled_colors;
  sampled_colors.reserve(sample_count);

  for (int i = 0; i < sample_count; ++i) {
    const float offset = (i + 0.5f) * sample_width - size.width / 2.0f;
    const auto sample_center = center + axis * offset;
    const float sample_height = std::max(6.0f, size.height * config_.halo_sample_scale);

    cv::Point2f corners[4] = {
        sample_center + axis * (-sample_width / 2.0f) + perp * (sample_height / 2.0f),
        sample_center + axis * (sample_width / 2.0f) + perp * (sample_height / 2.0f),
        sample_center + axis * (sample_width / 2.0f) - perp * (sample_height / 2.0f),
        sample_center - axis * (sample_width / 2.0f) - perp * (sample_height / 2.0f)};

    cv::Mat sample_mask = cv::Mat::zeros(hsv.size(), CV_8UC1);
    std::vector<cv::Point> polygon;
    polygon.reserve(4);
    for (const auto& corner : corners) {
      polygon.push_back(
          {static_cast<int>(std::round(corner.x)), static_cast<int>(std::round(corner.y))});
    }
    cv::fillConvexPoly(sample_mask, polygon, 255);

    cv::Mat sample_hsv;
    hsv.copyTo(sample_hsv, sample_mask);
    sample_hsv = enhance_roi_hsv(sample_hsv);

    cv::Mat color_mask;
    cv::inRange(sample_hsv, cv::Scalar(0, 25, 50), cv::Scalar(179, 255, 255), color_mask);
    cv::bitwise_and(color_mask, sample_mask, color_mask);
    cv::Scalar mean_hsv;
    if (cv::countNonZero(color_mask) > 0) {
      mean_hsv = cv::mean(sample_hsv, color_mask);
    } else {
      mean_hsv = cv::mean(sample_hsv, sample_mask);
    }
    ColorClass color = classify_mean_hsv(static_cast<float>(mean_hsv[0]),
                                         static_cast<float>(mean_hsv[1]),
                                         static_cast<float>(mean_hsv[2]));
    sampled_colors.push_back(color);
  }

  int run_count = 0;
  ColorClass prev = ColorClass::UNKNOWN;
  for (ColorClass color : sampled_colors) {
    if (color == ColorClass::UNKNOWN) {
      continue;
    }
    if (prev == ColorClass::UNKNOWN || color != prev) {
      ++run_count;
      prev = color;
    }
  }

  return run_count;
}

BeaconCandidate BeaconDetector::build_candidate(
    const std::vector<cv::Point>& contour) const {
  BeaconCandidate candidate;
  cv::convexHull(contour, candidate.contour);
  candidate.area = cv::contourArea(candidate.contour);
  candidate.rotated_rect = cv::minAreaRect(candidate.contour);
  is_strip_like(candidate.area, candidate.rotated_rect, &candidate.aspect_ratio,
                &candidate.rect_fill_ratio);
  candidate.score = candidate.area * candidate.aspect_ratio *
                    std::clamp(candidate.rect_fill_ratio, 0.0, 1.0);
  return candidate;
}

bool BeaconDetector::should_merge_candidates(const BeaconCandidate& lhs,
                                             const BeaconCandidate& rhs) const {
  std::vector<cv::Point2f> intersections;
  if (cv::rotatedRectangleIntersection(lhs.rotated_rect, rhs.rotated_rect,
                                       intersections) != cv::INTERSECT_NONE) {
    return true;
  }

  const auto lhs_axis = long_axis(lhs.rotated_rect);
  const auto rhs_axis = long_axis(rhs.rotated_rect);
  const float lhs_angle =
      std::atan2(lhs_axis.y, lhs_axis.x) * static_cast<float>(180.0 / CV_PI);
  const float rhs_angle =
      std::atan2(rhs_axis.y, rhs_axis.x) * static_cast<float>(180.0 / CV_PI);
  if (normalized_angle_diff_deg(lhs_angle, rhs_angle) >
      config_.merge_angle_tolerance_deg) {
    return false;
  }

  const cv::Point2f axis = lhs_axis;
  const cv::Point2f perp(-axis.y, axis.x);
  const cv::Point2f delta = rhs.rotated_rect.center - lhs.rotated_rect.center;
  const float along_dist = std::abs(delta.dot(axis));
  const float perp_dist = std::abs(delta.dot(perp));

  const float lhs_length =
      std::max(lhs.rotated_rect.size.width, lhs.rotated_rect.size.height);
  const float rhs_length =
      std::max(rhs.rotated_rect.size.width, rhs.rotated_rect.size.height);
  const float lhs_thickness =
      std::max(1.0f, std::min(lhs.rotated_rect.size.width, lhs.rotated_rect.size.height));
  const float rhs_thickness =
      std::max(1.0f, std::min(rhs.rotated_rect.size.width, rhs.rotated_rect.size.height));
  const float avg_thickness = 0.5f * (lhs_thickness + rhs_thickness);
  const float half_span = 0.5f * (lhs_length + rhs_length);
  const float axis_gap = std::max(0.0f, along_dist - half_span);
  const float max_axis_gap = avg_thickness * config_.merge_axis_gap_ratio;
  const float max_perp_gap = avg_thickness * config_.merge_perp_gap_ratio;
  return axis_gap <= max_axis_gap && perp_dist <= max_perp_gap;
}

bool BeaconDetector::is_strip_like(double contour_area, const cv::RotatedRect& rect,
                                   double* aspect_ratio,
                                   double* rect_fill_ratio) const {
  if (contour_area < config_.min_contour_area_px) {
    return false;
  }

  const double rect_area = static_cast<double>(rect.size.width) * rect.size.height;
  if (rect_area <= 1.0) {
    return false;
  }

  const double long_side = std::max(rect.size.width, rect.size.height);
  const double short_side = std::min(rect.size.width, rect.size.height);
  if (short_side <= 1e-6) {
    return false;
  }

  const double computed_aspect_ratio = long_side / short_side;
  const double computed_fill_ratio = contour_area / rect_area;
  if (aspect_ratio) {
    *aspect_ratio = computed_aspect_ratio;
  }
  if (rect_fill_ratio) {
    *rect_fill_ratio = computed_fill_ratio;
  }

  return computed_aspect_ratio >= config_.min_strip_aspect_ratio &&
         computed_fill_ratio >= config_.min_rect_fill_ratio;
}

std::string BeaconDetector::build_hsv_summary(const BeaconCandidate& candidate) const {
  std::ostringstream oss;
  bool wrote_any = false;
  for (size_t i = 0; i < candidate.segment_hsv_stats.size(); ++i) {
    const auto& stats = candidate.segment_hsv_stats[i];
    if (!is_reportable_color(stats.classified_color)) {
      continue;
    }
    if (wrote_any) {
      oss << " | ";
    }
    wrote_any = true;
    oss << "seg" << i << ":" << color_name(stats.classified_color);
    if (!stats.valid) {
      oss << " HSV=NA";
      continue;
    }
    oss << " H=" << static_cast<int>(std::round(stats.h_mean))
        << "[" << stats.h_min << "-" << stats.h_max << "]"
        << " S=" << static_cast<int>(std::round(stats.s_mean))
        << "[" << stats.s_min << "-" << stats.s_max << "]"
        << " V=" << static_cast<int>(std::round(stats.v_mean))
        << "[" << stats.v_min << "-" << stats.v_max << "]";
  }
  return wrote_any ? oss.str() : std::string();
}

std::vector<BeaconCandidate> BeaconDetector::merge_candidates(
    const std::vector<BeaconCandidate>& candidates) {
  if (candidates.size() <= 1) {
    return candidates;
  }

  std::vector<BeaconCandidate> merged;
  std::vector<bool> used(candidates.size(), false);
  for (size_t i = 0; i < candidates.size(); ++i) {
    if (used[i]) {
      continue;
    }

    auto combined = candidates[i];
    for (size_t j = i + 1; j < candidates.size(); ++j) {
      if (used[j]) {
        continue;
      }

      if (should_merge_candidates(combined, candidates[j])) {
        const double merged_area_ratio = combined.area_ratio + candidates[j].area_ratio;
        const double merged_score = combined.score + candidates[j].score;
        const double merged_mean_brightness =
            std::max(combined.mean_brightness, candidates[j].mean_brightness);
        combined.contour.insert(combined.contour.end(),
                                candidates[j].contour.begin(),
                                candidates[j].contour.end());
        combined = build_candidate(combined.contour);
        combined.area_ratio = merged_area_ratio;
        combined.mean_brightness = merged_mean_brightness;
        combined.score = merged_score;
        used[j] = true;
      }
    }

    merged.push_back(combined);
    used[i] = true;
  }

  return merged;
}

const BeaconCandidate* BeaconDetector::select_best(
    const std::vector<BeaconCandidate>& candidates) const {
  if (candidates.empty()) {
    return nullptr;
  }

  const BeaconCandidate* best = nullptr;
  double best_score = -std::numeric_limits<double>::max();

  for (const auto& candidate : candidates) {
    int visible_segments = 0;
    for (auto color : candidate.segment_colors) {
      if (color != ColorClass::UNKNOWN) {
        ++visible_segments;
      }
    }

    const double visible_bonus = visible_segments > 0 ? candidate.score * 0.10 : 0.0;
    const double stop_bonus = has_stop_segment(candidate) ? candidate.score * 0.15 : 0.0;
    const double score = candidate.score + visible_bonus + stop_bonus;
    if (score > best_score) {
      best_score = score;
      best = &candidate;
    }
  }

  return best;
}

void BeaconDetector::draw_debug(const cv::Mat& frame, const cv::Mat& hsv,
                                const std::vector<BeaconCandidate>& candidates,
                                const BeaconCandidate* best,
                                const BeaconState& state) {
  debug_overlay_ = frame.clone();
  if (debug_overlay_.empty()) {
    return;
  }

  const float scale_x =
      hsv.cols > 0 ? static_cast<float>(frame.cols) / hsv.cols : 1.0f;
  const float scale_y =
      hsv.rows > 0 ? static_cast<float>(frame.rows) / hsv.rows : 1.0f;
  auto scale_point = [&](const cv::Point2f& point) {
    return cv::Point(static_cast<int>(std::round(point.x * scale_x)),
                     static_cast<int>(std::round(point.y * scale_y)));
  };

  if (config_.use_roi) {
    const int x0 = std::clamp(static_cast<int>(debug_overlay_.cols * config_.roi_x_min_ratio),
                              0, debug_overlay_.cols);
    const int x1 = std::clamp(static_cast<int>(debug_overlay_.cols * config_.roi_x_max_ratio),
                              0, debug_overlay_.cols);
    const int y0 = std::clamp(static_cast<int>(debug_overlay_.rows * config_.roi_y_min_ratio),
                              0, debug_overlay_.rows);
    const int y1 = std::clamp(static_cast<int>(debug_overlay_.rows * config_.roi_y_max_ratio),
                              0, debug_overlay_.rows);
    if (x1 > x0 && y1 > y0) {
      cv::rectangle(debug_overlay_, cv::Rect(x0, y0, x1 - x0, y1 - y0),
                    cv::Scalar(255, 180, 0), 2);
    }
  }

  if (best != nullptr) {
    cv::Point2f vertices[4];
    best->rotated_rect.points(vertices);
    for (int i = 0; i < 4; ++i) {
      cv::line(debug_overlay_, scale_point(vertices[i]),
               scale_point(vertices[(i + 1) % 4]), cv::Scalar(0, 255, 0), 3);
    }
  }

  const std::string info = std::string(BeaconState::state_name(state.state)) +
                           " conf=" + std::to_string(state.confidence).substr(0, 4);
  cv::putText(debug_overlay_, info, {20, 34}, cv::FONT_HERSHEY_SIMPLEX, 0.9,
              cv::Scalar(0, 0, 255), 2);
  if (!latest_hsv_summary_.empty()) {
    cv::putText(debug_overlay_, latest_hsv_summary_, {20, 66},
                cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(0, 255, 255), 1);
  }
}

}  // namespace color_detect
