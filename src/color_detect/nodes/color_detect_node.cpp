/// @file color_detect_node.cpp
/// @brief Main light-strip pipeline: single direct camera -> detection -> serial.

#include <algorithm>
#include <chrono>
#include <cctype>
#include <cstdint>
#include <cstdlib>
#include <exception>
#include <functional>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <rclcpp/rclcpp.hpp>
#include <serial/serial.h>

#include "color_detect/beacon_config.hpp"
#include "color_detect/beacon_detector.hpp"
#include "color_detect/beacon_protocol.hpp"

namespace color_detect {

namespace {

constexpr uint8_t kHead[2] = {0xCA, 0xCA};
constexpr uint8_t kTail[2] = {0xAA, 0xBB};

uint8_t state_to_byte(BeaconState::State state) {
  switch (state) {
    case BeaconState::WAIT:
      return 1;
    case BeaconState::STOP:
    case BeaconState::GO:
    case BeaconState::UNKNOWN:
    default:
      return 0;
  }
}

}  // namespace

class ColorDetectNode : public rclcpp::Node {
public:
  ColorDetectNode() : Node("color_detect_node") {
    declare_params();
    cfg_ = load_config();
    load_camera_params();
    load_serial_params();

    detector_ = std::make_unique<BeaconDetector>(cfg_);

    open_camera();
    open_serial();

    timer_ = create_wall_timer(std::chrono::milliseconds(33),
                               std::bind(&ColorDetectNode::timer_cb, this));

    RCLCPP_INFO(get_logger(), "===========================================");
    RCLCPP_INFO(get_logger(), "  Color Detect Single-Camera Pipeline Started");
    RCLCPP_INFO(get_logger(), "  device=%s", camera_device_.c_str());
    RCLCPP_INFO(get_logger(), "  camera=%dx%d@%d %s buffer=%d exposure(auto=%d time=%d)",
                camera_width_, camera_height_, camera_fps_, camera_format_.c_str(),
                camera_buffer_size_, camera_auto_exposure_, camera_exposure_time_);
    RCLCPP_INFO(get_logger(), "  serial=%s @ %d", serial_port_.c_str(), serial_baudrate_);
    RCLCPP_INFO(get_logger(), "  red-detect=%s => WAIT, else GO", cfg_.color_stop.c_str());
    RCLCPP_INFO(get_logger(), "  output debounce wait=%d go=%d frames",
                cfg_.output_stop_frames, cfg_.output_go_frames);
    RCLCPP_INFO(get_logger(), "  serial tx=CA CA [00 GO | 01 WAIT] AA BB");
    RCLCPP_INFO(get_logger(), "===========================================");
  }

  ~ColorDetectNode() override {
    if (capture_.isOpened()) {
      capture_.release();
    }
    if (serial_.isOpen()) {
      serial_.close();
    }
  }

private:
  static std::string shell_quote(const std::string& value) {
    std::string quoted = "'";
    for (char ch : value) {
      if (ch == '\'') {
        quoted += "'\\''";
      } else {
        quoted.push_back(ch);
      }
    }
    quoted.push_back('\'');
    return quoted;
  }

  static int fourcc_from_string(std::string format) {
    for (char& ch : format) {
      ch = static_cast<char>(std::toupper(static_cast<unsigned char>(ch)));
    }
    if (format == "MJPEG" || format == "MJPG") {
      return cv::VideoWriter::fourcc('M', 'J', 'P', 'G');
    }
    if (format.size() == 4) {
      return cv::VideoWriter::fourcc(format[0], format[1], format[2], format[3]);
    }
    return cv::VideoWriter::fourcc('M', 'J', 'P', 'G');
  }

  void declare_params() {
    declare_parameter<std::string>("direct_camera.device", "/dev/cam_right");
    declare_parameter<int>("direct_camera.width", 1280);
    declare_parameter<int>("direct_camera.height", 720);
    declare_parameter<int>("direct_camera.fps", 30);
    declare_parameter<std::string>("direct_camera.input_format", "MJPG");
    declare_parameter<int>("direct_camera.buffer_size", 1);
    declare_parameter<int>("direct_camera.auto_exposure", 1);
    declare_parameter<int>("direct_camera.exposure_time", 1);

    declare_parameter<std::string>("serial.port", "/dev/detection");
    declare_parameter<int>("serial.baudrate", 115200);
    declare_parameter<bool>("display.enabled", true);
    declare_parameter<double>("display.scale", 0.75);

    declare_parameter<int>("resize_width", 480);
    declare_parameter<bool>("clahe.enabled", true);
    declare_parameter<double>("clahe.clip_limit", 2.0);
    declare_parameter<int>("clahe.grid_size", 8);
    declare_parameter<int>("color.gaussian_kernel", 3);
    declare_parameter<double>("color.saturation_gain", 1.8);
    declare_parameter<double>("color.value_gain", 1.05);
    declare_parameter<int>("hsv.v_threshold", 25);
    declare_parameter<int>("hsv.v_min_bright", 20);
    declare_parameter<int>("hsv.s_threshold_white", 40);
    declare_parameter<int>("hsv.s_min_for_color", 30);
    declare_parameter<int>("hsv.saturation_threshold", 30);
    declare_parameter<int>("hsv.hue_tolerance", 22);
    declare_parameter<int>("hsv.white_core_link_dilation", 11);
    declare_parameter<int>("hsv.candidate_bright_threshold", 180);
    declare_parameter<int>("hsv.candidate_threshold_delta", 45);
    declare_parameter<double>("hsv.candidate_min_area_ratio", 0.001);
    declare_parameter<double>("hsv.candidate_min_fill_ratio", 0.06);
    declare_parameter<double>("hsv.candidate_min_aspect_ratio", 2.0);
    declare_parameter<int>("hsv.yellow.h_low", 18);
    declare_parameter<int>("hsv.yellow.h_high", 52);
    declare_parameter<int>("hsv.yellow.s_low", 32);
    declare_parameter<int>("hsv.yellow.v_low", 20);
    declare_parameter<int>("hsv.green.h_low", 45);
    declare_parameter<int>("hsv.green.h_high", 80);
    declare_parameter<int>("hsv.green.s_low", 30);
    declare_parameter<int>("hsv.green.v_low", 20);
    declare_parameter<int>("hsv.red.h_low", 172);
    declare_parameter<int>("hsv.red.h_high", 8);
    declare_parameter<int>("hsv.red.s_low", 35);
    declare_parameter<int>("hsv.red.v_low", 40);
    declare_parameter<int>("hsv.red.v_floor", 12);
    declare_parameter<double>("hsv.red.min_ratio", 0.02);
    declare_parameter<int>("morph.open_size", 3);
    declare_parameter<int>("morph.close_size", 7);
    declare_parameter<int>("morph.dilation_size", 3);
    declare_parameter<int>("morph.close_iterations", 2);
    declare_parameter<int>("morph.dilate_iterations", 1);
    declare_parameter<int>("beacon.min_contour_area", 12);
    declare_parameter<double>("beacon.min_strip_aspect_ratio", 2.0);
    declare_parameter<double>("beacon.min_rect_fill_ratio", 0.06);
    declare_parameter<double>("beacon.halo_sample_scale", 2.8);
    declare_parameter<double>("beacon.merge_angle_tolerance_deg", 12.0);
    declare_parameter<double>("beacon.merge_axis_gap_ratio", 4.0);
    declare_parameter<double>("beacon.merge_perp_gap_ratio", 1.5);
    declare_parameter<int>("beacon.n_segments", 1);
    declare_parameter<std::string>("beacon.color_stop", "RED");
    declare_parameter<double>("beacon.color_consistency_threshold", 0.30);
    declare_parameter<int>("output.stop_frames", 1);
    declare_parameter<int>("output.go_frames", 18);
    declare_parameter<bool>("roi.enabled", true);
    declare_parameter<double>("roi.y_min_ratio", 0.35);
    declare_parameter<double>("roi.y_max_ratio", 1.00);
    declare_parameter<double>("roi.x_min_ratio", 0.00);
    declare_parameter<double>("roi.x_max_ratio", 1.00);
    declare_parameter<double>("input_timeout_sec", 10.0);
  }

  BeaconConfig load_config() {
    BeaconConfig c;
    c.resize_width = get_parameter("resize_width").as_int();
    c.use_clahe = get_parameter("clahe.enabled").as_bool();
    c.clahe_clip_limit = static_cast<float>(get_parameter("clahe.clip_limit").as_double());
    c.clahe_grid_size = get_parameter("clahe.grid_size").as_int();
    c.gaussian_kernel = get_parameter("color.gaussian_kernel").as_int();
    c.saturation_gain = static_cast<float>(get_parameter("color.saturation_gain").as_double());
    c.value_gain = static_cast<float>(get_parameter("color.value_gain").as_double());
    c.v_threshold = get_parameter("hsv.v_threshold").as_int();
    c.v_min_bright = get_parameter("hsv.v_min_bright").as_int();
    c.s_threshold_white = get_parameter("hsv.s_threshold_white").as_int();
    c.s_min_for_color = get_parameter("hsv.s_min_for_color").as_int();
    c.saturation_threshold = get_parameter("hsv.saturation_threshold").as_int();
    c.hue_tolerance = get_parameter("hsv.hue_tolerance").as_int();
    c.white_core_link_dilation = get_parameter("hsv.white_core_link_dilation").as_int();
    c.candidate_bright_threshold = get_parameter("hsv.candidate_bright_threshold").as_int();
    c.candidate_threshold_delta = get_parameter("hsv.candidate_threshold_delta").as_int();
    c.candidate_min_area_ratio =
        static_cast<float>(get_parameter("hsv.candidate_min_area_ratio").as_double());
    c.candidate_min_fill_ratio =
        static_cast<float>(get_parameter("hsv.candidate_min_fill_ratio").as_double());
    c.candidate_min_aspect_ratio =
        static_cast<float>(get_parameter("hsv.candidate_min_aspect_ratio").as_double());
    c.yellow_h_low = get_parameter("hsv.yellow.h_low").as_int();
    c.yellow_h_high = get_parameter("hsv.yellow.h_high").as_int();
    c.yellow_s_low = get_parameter("hsv.yellow.s_low").as_int();
    c.yellow_v_low = get_parameter("hsv.yellow.v_low").as_int();
    c.green_h_low = get_parameter("hsv.green.h_low").as_int();
    c.green_h_high = get_parameter("hsv.green.h_high").as_int();
    c.green_s_low = get_parameter("hsv.green.s_low").as_int();
    c.green_v_low = get_parameter("hsv.green.v_low").as_int();
    c.red_h_low = get_parameter("hsv.red.h_low").as_int();
    c.red_h_high = get_parameter("hsv.red.h_high").as_int();
    c.red_s_low = get_parameter("hsv.red.s_low").as_int();
    c.red_v_low = get_parameter("hsv.red.v_low").as_int();
    c.red_v_floor = get_parameter("hsv.red.v_floor").as_int();
    c.red_min_ratio =
        static_cast<float>(get_parameter("hsv.red.min_ratio").as_double());
    c.morph_open_size = get_parameter("morph.open_size").as_int();
    c.morph_close_size = get_parameter("morph.close_size").as_int();
    c.dilation_size = get_parameter("morph.dilation_size").as_int();
    c.morph_close_iterations = get_parameter("morph.close_iterations").as_int();
    c.morph_dilate_iterations = get_parameter("morph.dilate_iterations").as_int();
    c.min_contour_area_px = get_parameter("beacon.min_contour_area").as_int();
    c.min_strip_aspect_ratio =
        static_cast<float>(get_parameter("beacon.min_strip_aspect_ratio").as_double());
    c.min_rect_fill_ratio =
        static_cast<float>(get_parameter("beacon.min_rect_fill_ratio").as_double());
    c.halo_sample_scale =
        static_cast<float>(get_parameter("beacon.halo_sample_scale").as_double());
    c.merge_angle_tolerance_deg =
        static_cast<float>(get_parameter("beacon.merge_angle_tolerance_deg").as_double());
    c.merge_axis_gap_ratio =
        static_cast<float>(get_parameter("beacon.merge_axis_gap_ratio").as_double());
    c.merge_perp_gap_ratio =
        static_cast<float>(get_parameter("beacon.merge_perp_gap_ratio").as_double());
    c.n_segments = get_parameter("beacon.n_segments").as_int();
    c.color_stop = get_parameter("beacon.color_stop").as_string();
    c.color_consistency_thresh =
        static_cast<float>(get_parameter("beacon.color_consistency_threshold").as_double());
    c.output_stop_frames =
        static_cast<int>(get_parameter("output.stop_frames").as_int());
    c.output_go_frames =
        static_cast<int>(get_parameter("output.go_frames").as_int());
    c.use_roi = get_parameter("roi.enabled").as_bool();
    c.roi_y_min_ratio = static_cast<float>(get_parameter("roi.y_min_ratio").as_double());
    c.roi_y_max_ratio = static_cast<float>(get_parameter("roi.y_max_ratio").as_double());
    c.roi_x_min_ratio = static_cast<float>(get_parameter("roi.x_min_ratio").as_double());
    c.roi_x_max_ratio = static_cast<float>(get_parameter("roi.x_max_ratio").as_double());
    c.debug_output = get_parameter("display.enabled").as_bool();
    return c;
  }

  void load_camera_params() {
    camera_device_ = get_parameter("direct_camera.device").as_string();
    camera_width_ = get_parameter("direct_camera.width").as_int();
    camera_height_ = get_parameter("direct_camera.height").as_int();
    camera_fps_ = get_parameter("direct_camera.fps").as_int();
    camera_format_ = get_parameter("direct_camera.input_format").as_string();
    camera_buffer_size_ = get_parameter("direct_camera.buffer_size").as_int();
    camera_auto_exposure_ = get_parameter("direct_camera.auto_exposure").as_int();
    camera_exposure_time_ = get_parameter("direct_camera.exposure_time").as_int();
  }

  void load_serial_params() {
    serial_port_ = get_parameter("serial.port").as_string();
    serial_baudrate_ = get_parameter("serial.baudrate").as_int();
    display_enabled_ = get_parameter("display.enabled").as_bool();
    display_scale_ = std::max(0.1, get_parameter("display.scale").as_double());
  }

  void set_v4l2_controls(const std::string& device) {
    if (camera_auto_exposure_ != 1 && camera_exposure_time_ >= 0) {
      RCLCPP_WARN(get_logger(),
                  "auto_exposure=%d is not manual mode; exposure_time_absolute=%d may be ignored by the camera",
                  camera_auto_exposure_, camera_exposure_time_);
    }

    std::vector<std::string> pairs;
    if (camera_auto_exposure_ >= 0) {
      pairs.push_back("auto_exposure=" + std::to_string(camera_auto_exposure_));
    }
    if (camera_exposure_time_ >= 0) {
      pairs.push_back("exposure_time_absolute=" + std::to_string(camera_exposure_time_));
    }
    if (pairs.empty()) {
      return;
    }

    std::ostringstream joined;
    for (size_t i = 0; i < pairs.size(); ++i) {
      if (i != 0) {
        joined << ",";
      }
      joined << pairs[i];
    }

    const std::string command =
        "v4l2-ctl -d " + shell_quote(device) + " --set-ctrl=" + joined.str();
    const int rc = std::system(command.c_str());
    if (rc != 0) {
      RCLCPP_WARN(get_logger(), "v4l2-ctl failed for %s: %s",
                  device.c_str(), joined.str().c_str());
      return;
    }
    RCLCPP_INFO(get_logger(), "v4l2 controls for %s: %s",
                device.c_str(), joined.str().c_str());
  }

  void open_camera() {
    set_v4l2_controls(camera_device_);
    capture_.open(camera_device_, cv::CAP_V4L2);
    if (!capture_.isOpened()) {
      throw std::runtime_error("Cannot open camera: " + camera_device_);
    }

    capture_.set(cv::CAP_PROP_BUFFERSIZE, camera_buffer_size_);
    capture_.set(cv::CAP_PROP_FRAME_WIDTH, camera_width_);
    capture_.set(cv::CAP_PROP_FRAME_HEIGHT, camera_height_);
    capture_.set(cv::CAP_PROP_FPS, camera_fps_);
    capture_.set(cv::CAP_PROP_FOURCC, fourcc_from_string(camera_format_));

    RCLCPP_INFO(get_logger(), "camera opened: %s actual=%.0fx%.0f@%.1f",
                camera_device_.c_str(),
                capture_.get(cv::CAP_PROP_FRAME_WIDTH),
                capture_.get(cv::CAP_PROP_FRAME_HEIGHT),
                capture_.get(cv::CAP_PROP_FPS));
  }

  void open_serial() {
    try {
      serial_.setPort(serial_port_);
      serial_.setBaudrate(serial_baudrate_);
      serial_.setTimeout(serial::Timeout::max(), 1000, 0, 1000, 0);
      serial_.open();
    } catch (const std::exception& e) {
      RCLCPP_ERROR(get_logger(),
                   "Failed to open serial %s: %s; detection/display will continue without TX",
                   serial_port_.c_str(), e.what());
      return;
    }
    RCLCPP_INFO(get_logger(), "Serial opened: %s @ %d",
                serial_port_.c_str(), serial_baudrate_);
  }

  void show_debug(const cv::Mat& frame) {
    if (!display_enabled_) {
      return;
    }

    cv::Mat vis = detector_->debug_overlay();
    if (vis.empty()) {
      vis = frame.clone();
    }
    if (vis.empty()) {
      return;
    }

    cv::putText(vis, "CAM", {20, 70},
                cv::FONT_HERSHEY_SIMPLEX, 0.9, cv::Scalar(255, 255, 255), 2);
    const std::string stable_info =
        std::string("OUT: ") + BeaconState::state_name(stable_state_.state);
    cv::putText(vis, stable_info, {20, 104},
                cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 255), 2);

    if (std::abs(display_scale_ - 1.0) > 1e-3) {
      cv::Mat resized;
      cv::resize(vis, resized, cv::Size(), display_scale_, display_scale_, cv::INTER_LINEAR);
      cv::imshow("Color Detect", resized);
    } else {
      cv::imshow("Color Detect", vis);
    }
  }

  bool read_and_process() {
    cv::Mat frame;
    if (!capture_.read(frame) || frame.empty()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Camera frame grab failed");
      return false;
    }

    last_frame_time_ = std::chrono::steady_clock::now();
    ++frame_count_;
    latest_state_ = detector_->process_frame(frame);
    update_output_state(latest_state_);

    show_debug(frame);
    return true;
  }

  void send_serial(const BeaconState& state) {
    if (!serial_.isOpen()) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000, "Serial is not open");
      return;
    }

    std::vector<uint8_t> packet;
    packet.insert(packet.end(), kHead, kHead + 2);
    packet.push_back(state_to_byte(state.state));
    packet.insert(packet.end(), kTail, kTail + 2);

    try {
      serial_.write(packet.data(), packet.size());
    } catch (const std::exception& e) {
      RCLCPP_ERROR(get_logger(), "Serial write failed: %s", e.what());
    }
  }

  void update_output_state(const BeaconState& raw_state) {
    if (raw_state.state == stable_state_.state) {
      pending_state_ = raw_state.state;
      pending_count_ = 0;
      stable_state_ = raw_state;
      return;
    }

    if (raw_state.state != pending_state_) {
      pending_state_ = raw_state.state;
      pending_count_ = 1;
    } else {
      ++pending_count_;
    }

    const int threshold = (raw_state.state == BeaconState::WAIT)
                              ? std::max(1, cfg_.output_stop_frames)
                              : std::max(1, cfg_.output_go_frames);
    if (pending_count_ >= threshold) {
      stable_state_ = raw_state;
      pending_count_ = 0;
    }
  }

  void log_output_state() {
    if (stable_state_.state == BeaconState::WAIT) {
      go_log_counter_ = 0;
      RCLCPP_INFO(get_logger(), "state=WAIT");
      return;
    }

    ++go_log_counter_;
    if (go_log_counter_ >= 5) {
      RCLCPP_INFO(get_logger(), "state=GO");
      go_log_counter_ = 0;
    }
  }

  void timer_cb() {
    const bool frame_ok = read_and_process();

    const auto now = std::chrono::steady_clock::now();
    const double input_timeout_sec = get_parameter("input_timeout_sec").as_double();
    const double silent_sec =
        std::chrono::duration<double>(now - last_frame_time_).count();
    if (silent_sec > input_timeout_sec) {
      RCLCPP_FATAL(get_logger(), "No input frame for %.1fs, exit for restart", silent_sec);
      rclcpp::shutdown();
      return;
    }

    if (frame_ok) {
      send_serial(stable_state_);
      log_output_state();
    }

    if (display_enabled_) {
      cv::waitKey(1);
    }
  }

  BeaconConfig cfg_;
  std::unique_ptr<BeaconDetector> detector_;
  BeaconState latest_state_;
  BeaconState stable_state_{BeaconState::GO, 0.80f};
  BeaconState::State pending_state_ = BeaconState::GO;
  int pending_count_ = 0;
  int go_log_counter_ = 0;

  cv::VideoCapture capture_;
  std::string camera_device_ = "/dev/cam_right";
  int camera_width_ = 1280;
  int camera_height_ = 720;
  int camera_fps_ = 30;
  std::string camera_format_ = "MJPG";
  int camera_buffer_size_ = 1;
  int camera_auto_exposure_ = 1;
  int camera_exposure_time_ = 1;

  serial::Serial serial_;
  std::string serial_port_ = "/dev/detection";
  int serial_baudrate_ = 115200;
  bool display_enabled_ = true;
  double display_scale_ = 0.75;

  rclcpp::TimerBase::SharedPtr timer_;
  std::chrono::steady_clock::time_point last_frame_time_ =
      std::chrono::steady_clock::now();
  uint64_t frame_count_ = 0;
};

}  // namespace color_detect

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<color_detect::ColorDetectNode>());
  rclcpp::shutdown();
  return 0;
}
