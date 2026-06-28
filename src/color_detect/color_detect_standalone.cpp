/// @file color_detect_standalone.cpp
/// @brief 单相机本地测试 — 直接从 /dev/video* 读取，无需 ROS2
///
/// 用法:
///   ./color_detect_standalone [--device /dev/video0] [--config config.yaml]
///
/// 按键:
///   q / ESC — 退出
///   d       — 切换调试叠加层
///   f       — 切换全屏

#include <chrono>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>
#include <unordered_map>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

#include "color_detect/beacon_detector.hpp"
#include "color_detect/beacon_config.hpp"
#include "color_detect/beacon_protocol.hpp"

namespace {

// ============================================================
//  简易 YAML 解析器（仅支持 flat key: value，用于加载配置）
// ============================================================
std::string trim(std::string s) {
  auto f = s.find_first_not_of(" \t\r\n");
  auto l = s.find_last_not_of(" \t\r\n");
  return (f == std::string::npos) ? "" : s.substr(f, l - f + 1);
}

std::string unquote(const std::string& s) {
  if (s.size() >= 2 &&
      ((s.front() == '"' && s.back() == '"') ||
       (s.front() == '\'' && s.back() == '\'')))
    return s.substr(1, s.size() - 2);
  return s;
}

bool iequals(const std::string& a, const std::string& b) {
  if (a.size() != b.size()) return false;
  for (size_t i = 0; i < a.size(); i++)
    if (std::tolower(a[i]) != std::tolower(b[i])) return false;
  return true;
}

color_detect::BeaconConfig load_config_yaml(const std::string& path) {
  color_detect::BeaconConfig cfg;
  std::ifstream f(path);
  if (!f.is_open()) {
    std::cerr << "[WARN] 无法打开配置文件: " << path
              << "，使用默认参数\n";
    return cfg;
  }

  // 展平 key 前缀（如 "hsv.v_threshold" → "v_threshold"）
  std::string prefix;
  std::string line;
  while (std::getline(f, line)) {
    line = trim(line);
    if (line.empty() || line[0] == '#') continue;

    // 嵌套节： "hsv:" "beacon:" "filter:"
    if (line.back() == ':') {
      prefix = line.substr(0, line.size() - 1);
      continue;
    }

    auto colon = line.find(':');
    if (colon == std::string::npos) continue;

    std::string key = line.substr(0, colon);
    std::string val = line.substr(colon + 1);
    key = trim(key);
    val = unquote(trim(val));

    // 没有嵌套前缀时，检查是否 key 本身带点号
    if (prefix.empty()) {
      auto dot = key.find('.');
      if (dot != std::string::npos) {
        prefix = key.substr(0, dot);
        key = key.substr(dot + 1);
      }
    }

    // key 可能是带前缀的（如 "hsv.v_threshold"）
    auto dot = key.find('.');
    std::string ns;
    if (dot != std::string::npos) {
      ns  = key.substr(0, dot);
      key = key.substr(dot + 1);
    }

    // 用完整的 "ns.key" 或 "prefix.key" 来匹配
    std::string full_key = prefix.empty() ? key : prefix + "." + key;

    // 映射到配置字段
    auto set_int = [&](const std::string& k, auto& dst) {
      if (full_key == k) dst = std::stoi(val);
    };
    auto set_flt = [&](const std::string& k, auto& dst) {
      if (full_key == k) dst = std::stof(val);
    };
    auto set_str = [&](const std::string& k, auto& dst) {
      if (full_key == k) dst = val;
    };
    auto set_bool = [&](const std::string& k, auto& dst) {
      if (full_key == k) dst = (val == "true" || val == "1" || iequals(val, "yes"));
    };

    set_int("resize_width",              cfg.resize_width);
    set_int("gaussian_kernel",           cfg.gaussian_kernel);
    set_bool("clahe.enabled",            cfg.use_clahe);
    set_flt("clahe.clip_limit",          cfg.clahe_clip_limit);
    set_int("clahe.grid_size",           cfg.clahe_grid_size);
    set_int("hsv.v_threshold",           cfg.v_threshold);
    set_int("hsv.v_min_bright",          cfg.v_min_bright);
    set_int("hsv.s_threshold_white",     cfg.s_threshold_white);
    set_int("hsv.s_min_for_color",       cfg.s_min_for_color);
    set_int("hsv.saturation_threshold",  cfg.saturation_threshold);
    set_int("hsv.hue_tolerance",         cfg.hue_tolerance);
    set_int("morph.open_size",           cfg.morph_open_size);
    set_int("morph_open_size",           cfg.morph_open_size);  // 兼容旧格式
    set_int("morph.close_size",          cfg.morph_close_size);
    set_int("morph_close_size",          cfg.morph_close_size);  // 兼容旧格式
    set_int("morph.dilation_size",       cfg.dilation_size);
    set_flt("beacon.min_aspect_ratio",   cfg.min_aspect_ratio);
    set_int("beacon.min_area",           cfg.min_area_px);
    set_flt("beacon.max_area_ratio",     cfg.max_area_ratio);
    set_flt("beacon.min_solidity",       cfg.min_solidity);
    set_int("beacon.n_segments",         cfg.n_segments);
    set_str("beacon.sync_color",         cfg.sync_color);
    set_str("beacon.color_wait",         cfg.color_wait);
    set_str("beacon.color_go",           cfg.color_go);
    set_flt("beacon.color_consistency_thresh", cfg.color_consistency_thresh);
    set_flt("beacon.uniformity_threshold", cfg.uniformity_threshold);
    set_int("filter.temporal_window",    cfg.temporal_window);
    set_int("filter.debounce_frames",    cfg.debounce_frames);
    set_int("filter.lost_timeout_ms",    cfg.lost_timeout_ms);
    set_int("filter.init_confirm_frames", cfg.init_confirm_frames);
    set_bool("debug",                    cfg.debug_output);
  }
  return cfg;
}

void print_usage(const char* prog) {
  std::cerr << "用法: " << prog << " [选项]\n"
            << "选项:\n"
            << "  --device DEV   相机设备路径 (默认 /dev/video0)\n"
            << "  --config PATH  YAML 配置文件路径\n"
            << "  --width W      视频宽度 (默认 1280，0=不强制)\n"
            << "  --height H     视频高度 (默认 720，0=跟随宽度)\n"
            << "  --help, -h     显示帮助\n";
}

}  // anonymous namespace

int main(int argc, char* argv[]) {
  std::string device = "/dev/video0";
  std::string config_path;
  int width = 1280;    // 默认 720p（相机规格支持）
  int height = 720;    // 默认 720p

  // ---- 解析命令行 ----
  for (int i = 1; i < argc; i++) {
    std::string arg = argv[i];
    if (arg == "--device" && i + 1 < argc) {
      device = argv[++i];
    } else if (arg == "--config" && i + 1 < argc) {
      config_path = argv[++i];
    } else if (arg == "--width" && i + 1 < argc) {
      width = std::stoi(argv[++i]);
    } else if (arg == "--height" && i + 1 < argc) {
      height = std::stoi(argv[++i]);
    } else if (arg == "--help" || arg == "-h") {
      print_usage(argv[0]);
      return 0;
    } else {
      std::cerr << "未知选项: " << arg << "\n";
      print_usage(argv[0]);
      return 1;
    }
  }

  // ---- 加载配置 ----
  color_detect::BeaconConfig cfg;
  if (!config_path.empty()) {
    cfg = load_config_yaml(config_path);
    std::cout << "[INFO] 已加载配置: " << config_path << "\n";
  }
  cfg.debug_output = true;  // 独立模式始终显示调试叠加

  // ---- 打开相机并设置 MJPEG 格式 + 目标分辨率 ----
  cv::VideoCapture cap;
  if (!cap.open(device, cv::CAP_V4L2)) {
    cap.open(device);
  }
  if (!cap.isOpened()) {
    std::cerr << "[ERROR] 无法打开相机设备: " << device << "\n";
    return 1;
  }

  // 设置 MJPEG 格式（这款相机在 MJPEG 下才支持 720p+）
  cap.set(cv::CAP_PROP_FOURCC,
          cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));

  // 设置分辨率（先宽后高，部分相机顺序敏感）
  if (width > 0)   cap.set(cv::CAP_PROP_FRAME_WIDTH, width);
  if (height > 0)  cap.set(cv::CAP_PROP_FRAME_HEIGHT, height);

  // 回读实际生效的分辨率
  int cam_w = static_cast<int>(cap.get(cv::CAP_PROP_FRAME_WIDTH));
  int cam_h = static_cast<int>(cap.get(cv::CAP_PROP_FRAME_HEIGHT));
  double cam_fps = cap.get(cv::CAP_PROP_FPS);

  std::cout << "\n=== Color Detect Standalone ===\n"
            << "  设备:     " << device << "\n"
            << "  分辨率:   " << cam_w << "x" << cam_h << "\n"
            << "  FPS:      " << cam_fps << "\n"
            << "==============================\n"
            << "按键: q=退出  d=调试叠加  f=全屏\n\n";

  // ---- 创建检测器 ----
  color_detect::BeaconDetector detector(cfg);

  cv::Mat frame;
  bool show_debug = true;
  bool fullscreen = false;
  int frame_count = 0;
  int print_interval = 10;  // 每 N 帧打印一次状态

  cv::namedWindow("Color Detect", cv::WINDOW_NORMAL);

  while (true) {
    cap >> frame;
    if (frame.empty()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
      continue;
    }

    frame_count++;

    // ---- 执行检测 ----
    auto state = detector.process_frame(frame);

    // ---- 打印状态 ----
    if (frame_count % print_interval == 0 ||
        state.state != color_detect::BeaconState::UNKNOWN) {
      std::cout << "[" << frame_count << "] "
                << color_detect::BeaconState::state_name(state.state)
                << "  conf=" << state.confidence
                << "  angle=" << static_cast<int>(state.beacon_angle_deg)
                << "deg"
                << "  area=" << static_cast<int>(state.beacon_area_px)
                << "px"
                << "\n";
    }

    // ---- 显示 ----
    cv::Mat display;
    if (show_debug) {
      display = detector.debug_overlay().clone();
    }
    if (display.empty()) {
      display = frame.clone();
    }

    // 叠加文本信息
    std::string info = std::string(color_detect::BeaconState::state_name(state.state))
                       + "  c:" + std::to_string(state.confidence).substr(0, 4);
    cv::putText(display, info, {display.cols - 220, 30},
                cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 255), 2);

    // 叠加帧计数
    cv::putText(display, std::to_string(frame_count), {10, 30},
                cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(200, 200, 200), 2);

    cv::imshow("Color Detect", display);

    // ---- 按键处理 ----
    int key = cv::waitKey(30) & 0xFF;
    if (key == 'q' || key == 27) break;   // q / ESC → 退出
    if (key == 'd') {
      show_debug = !show_debug;
      std::cout << "[INFO] 调试叠加: " << (show_debug ? "开" : "关") << "\n";
    }
    if (key == 'f') {
      fullscreen = !fullscreen;
      cv::setWindowProperty("Color Detect", cv::WND_PROP_FULLSCREEN,
                            fullscreen ? cv::WINDOW_FULLSCREEN : cv::WINDOW_NORMAL);
    }
  }

  cap.release();
  cv::destroyWindow("Color Detect");
  std::cout << "[INFO] 已退出。\n";
  return 0;
}
