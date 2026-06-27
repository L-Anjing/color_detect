#ifndef COLOR_DETECT_BEACON_CONFIG_HPP_
#define COLOR_DETECT_BEACON_CONFIG_HPP_

#include <cstdint>
#include <string>

namespace color_detect {

/// @brief 视觉信标系统——可调参数
struct BeaconConfig {
  // ── 预处理 ──
  int resize_width = 640;
  int gaussian_kernel = 3;

  // ── HSV 阈值 ──
  int v_threshold = 200;
  int v_min_bright = 180;
  int s_threshold_white = 40;
  int s_min_for_color = 60;
  int hue_tolerance = 12;

  // ── 形态学 ──
  int morph_open_size = 3;
  int morph_close_size = 7;

  // ── 灯带形状 ──
  float min_aspect_ratio = 3.0f;
  int min_area_px = 50;
  float max_area_ratio = 0.3f;
  float min_solidity = 0.85f;

  // ── 协议 ──
  int n_segments = 2;
  std::string sync_color = "YELLOW";
  std::string color_wait = "CYAN";
  std::string color_go = "MAGENTA";
  float color_consistency_thresh = 0.7f;

  // ── 时间滤波 ──
  int temporal_window = 5;
  int debounce_frames = 3;
  int lost_timeout_ms = 200;
  int init_confirm_frames = 5;

  // ── 相机话题（双摄） ──
  std::string camera_topic_left  = "/cam_left/image_raw";
  std::string camera_topic_right = "/cam_right/image_raw";

  // ── 调试 ──
  bool debug_output = false;
};

}  // namespace color_detect

#endif  // COLOR_DETECT_BEACON_CONFIG_HPP_
