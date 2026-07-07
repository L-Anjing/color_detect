#ifndef COLOR_DETECT_BEACON_CONFIG_HPP_
#define COLOR_DETECT_BEACON_CONFIG_HPP_

#include <string>

namespace color_detect {

struct BeaconConfig {
  int resize_width = 480;
  int gaussian_kernel = 3;

  bool use_clahe = true;
  float clahe_clip_limit = 2.0f;
  int clahe_grid_size = 8;
  float saturation_gain = 1.8f;
  float value_gain = 1.05f;

  int v_threshold = 25;
  int v_min_bright = 20;
  int s_threshold_white = 40;
  int s_min_for_color = 30;
  int saturation_threshold = 30;
  int hue_tolerance = 22;
  int white_core_link_dilation = 11;
  int candidate_bright_threshold = 180;
  int candidate_threshold_delta = 45;
  float candidate_min_area_ratio = 0.001f;
  float candidate_min_fill_ratio = 0.06f;
  float candidate_min_aspect_ratio = 2.0f;
  int yellow_h_low = 18;
  int yellow_h_high = 52;
  int yellow_s_low = 32;
  int yellow_v_low = 20;
  int green_h_low = 45;
  int green_h_high = 80;
  int green_s_low = 30;
  int green_v_low = 20;
  int red_h_low = 172;
  int red_h_high = 8;
  int red_s_low = 35;
  int red_v_low = 40;
  int red_v_floor = 12;
  float red_min_ratio = 0.02f;

  int morph_open_size = 3;
  int morph_close_size = 7;
  int dilation_size = 3;
  int morph_close_iterations = 2;
  int morph_dilate_iterations = 1;

  int min_contour_area_px = 12;
  float min_strip_aspect_ratio = 2.0f;
  float min_rect_fill_ratio = 0.06f;
  float halo_sample_scale = 2.8f;
  float merge_angle_tolerance_deg = 12.0f;
  float merge_axis_gap_ratio = 4.0f;
  float merge_perp_gap_ratio = 1.5f;
  bool use_roi = true;
  float roi_y_min_ratio = 0.35f;
  float roi_y_max_ratio = 1.00f;
  float roi_x_min_ratio = 0.00f;
  float roi_x_max_ratio = 1.00f;

  int n_segments = 1;
  std::string color_stop = "RED";
  float color_consistency_thresh = 0.30f;
  int output_stop_frames = 1;
  int output_go_frames = 18;
  bool debug_output = false;
};

}  // namespace color_detect

#endif  // COLOR_DETECT_BEACON_CONFIG_HPP_
