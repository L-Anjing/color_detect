/// @file color_detect_node.cpp
/// @brief 双摄像头视觉信标节点
///
/// 架构:
///   /cam_left/image_raw ──→ DetectorLeft ──→ StateMachineLeft ──┐
///                                                               ├──→ Fusion ──→ /color_detect/state
///   /cam_right/image_raw ─→ DetectorRight ─→ StateMachineRight ─┘
///
/// 用法:
///   ros2 run color_detect color_detect_node --ros-args \
///     -p camera_topic_left:=/cam_left/image_raw \
///     -p camera_topic_right:=/cam_right/image_raw

#include <chrono>
#include <memory>
#include <string>
#include <mutex>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/string.hpp>
#include <cv_bridge/cv_bridge.h>

#include "color_detect/beacon_detector.hpp"
#include "color_detect/beacon_config.hpp"
#include "color_detect/beacon_protocol.hpp"
#include "color_detect/beacon_fusion.hpp"

namespace color_detect {

class ColorDetectNode : public rclcpp::Node {
public:
  ColorDetectNode() : Node("color_detect_node") {
    declare_params();
    auto cfg = load_config();

    // 两个独立的检测器（各带自己的状态机）
    det_left_  = std::make_unique<BeaconDetector>(cfg);
    det_right_ = std::make_unique<BeaconDetector>(cfg);

    // 订阅双摄像头
    sub_left_ = create_subscription<sensor_msgs::msg::Image>(
        cfg.camera_topic_left, rclcpp::QoS(2).best_effort(),
        std::bind(&ColorDetectNode::cb_left, this, std::placeholders::_1));

    sub_right_ = create_subscription<sensor_msgs::msg::Image>(
        cfg.camera_topic_right, rclcpp::QoS(2).best_effort(),
        std::bind(&ColorDetectNode::cb_right, this, std::placeholders::_1));

    // 发布融合结果
    pub_ = create_publisher<std_msgs::msg::String>("/color_detect/state", 10);

    // 可选：左右各自状态（调试用）
    pub_left_state_  = create_publisher<std_msgs::msg::String>("/color_detect/left_state", 10);
    pub_right_state_ = create_publisher<std_msgs::msg::String>("/color_detect/right_state", 10);

    // 可选：调试图像
    if (cfg.debug_output) {
      pub_debug_left_ = create_publisher<sensor_msgs::msg::Image>(
          "/color_detect/debug_left", rclcpp::QoS(2));
      pub_debug_right_ = create_publisher<sensor_msgs::msg::Image>(
          "/color_detect/debug_right", rclcpp::QoS(2));
    }

    // 延迟补偿定时器：如果某一侧长时间无帧，强制融合
    timer_ = create_wall_timer(std::chrono::milliseconds(33),
                               std::bind(&ColorDetectNode::timer_cb, this));

    RCLCPP_INFO(get_logger(), "===========================================");
    RCLCPP_INFO(get_logger(), "  Color Detect Node (Dual Cam) Started");
    RCLCPP_INFO(get_logger(), "  Left:  %s", cfg.camera_topic_left.c_str());
    RCLCPP_INFO(get_logger(), "  Right: %s", cfg.camera_topic_right.c_str());
    RCLCPP_INFO(get_logger(), "  Sync=%s  WAIT=%s  GO=%s",
                cfg.sync_color.c_str(), cfg.color_wait.c_str(), cfg.color_go.c_str());
    RCLCPP_INFO(get_logger(), "  Segments=%d | V_thresh=%d | Aspect=%.1f",
                cfg.n_segments, cfg.v_threshold, cfg.min_aspect_ratio);
    RCLCPP_INFO(get_logger(), "===========================================");
  }

private:
  void declare_params() {
    declare_parameter<std::string>("camera_topic_left",  "/cam_left/image_raw");
    declare_parameter<std::string>("camera_topic_right", "/cam_right/image_raw");
    declare_parameter<int>("resize_width", 640);
    declare_parameter<int>("hsv.v_threshold", 200);
    declare_parameter<int>("hsv.s_threshold_white", 40);
    declare_parameter<int>("hsv.hue_tolerance", 12);
    declare_parameter<float>("beacon.min_aspect_ratio", 3.0f);
    declare_parameter<int>("beacon.min_area", 50);
    declare_parameter<int>("beacon.n_segments", 2);
    declare_parameter<std::string>("beacon.sync_color", "YELLOW");
    declare_parameter<std::string>("beacon.color_wait", "CYAN");
    declare_parameter<std::string>("beacon.color_go", "MAGENTA");
    declare_parameter<int>("filter.temporal_window", 5);
    declare_parameter<int>("filter.debounce_frames", 3);
    declare_parameter<int>("filter.lost_timeout_ms", 200);
    declare_parameter<bool>("debug", false);
  }

  BeaconConfig load_config() {
    BeaconConfig c;
    c.camera_topic_left   = get_parameter("camera_topic_left").as_string();
    c.camera_topic_right  = get_parameter("camera_topic_right").as_string();
    c.resize_width        = get_parameter("resize_width").as_int();
    c.v_threshold         = get_parameter("hsv.v_threshold").as_int();
    c.s_threshold_white   = get_parameter("hsv.s_threshold_white").as_int();
    c.hue_tolerance       = get_parameter("hsv.hue_tolerance").as_int();
    c.min_aspect_ratio    = get_parameter("beacon.min_aspect_ratio").as_double();
    c.min_area_px         = get_parameter("beacon.min_area").as_int();
    c.n_segments          = get_parameter("beacon.n_segments").as_int();
    c.sync_color          = get_parameter("beacon.sync_color").as_string();
    c.color_wait          = get_parameter("beacon.color_wait").as_string();
    c.color_go            = get_parameter("beacon.color_go").as_string();
    c.temporal_window     = get_parameter("filter.temporal_window").as_int();
    c.debounce_frames     = get_parameter("filter.debounce_frames").as_int();
    c.lost_timeout_ms     = get_parameter("filter.lost_timeout_ms").as_int();
    c.debug_output        = get_parameter("debug").as_bool();
    return c;
  }

  /// @brief 检测一帧 + 发布融合结果（左右共用）
  void process_and_publish(CameraSide side, const cv::Mat& frame) {
    // 选择对应的检测器
    auto& det = (side == LEFT) ? det_left_ : det_right_;
    auto& fuse_state = (side == LEFT) ? fuse_left_raw_ : fuse_right_raw_;
    auto& pub_debug = (side == LEFT) ? pub_debug_left_ : pub_debug_right_;
    const char* side_name = (side == LEFT) ? "L" : "R";

    // 检测
    auto raw = (*det).process_frame(frame);
    fuse_state = raw;

    // 发布侧边状态（调试）
    auto side_pub = (side == LEFT) ? pub_left_state_ : pub_right_state_;
    auto side_msg = std::make_unique<std_msgs::msg::String>();
    side_msg->data = raw.to_json();
    side_pub->publish(std::move(side_msg));

    // 更新融合器（互斥保护）
    {
      std::lock_guard<std::mutex> lock(fusion_mutex_);
      if (side == LEFT) fusion_.update_left(raw);
      else              fusion_.update_right(raw);

      // 融合
      auto fused = fusion_.fused();

      // 发布
      auto out = std::make_unique<std_msgs::msg::String>();
      out->data = fused.to_json();
      pub_->publish(std::move(out));

      RCLCPP_DEBUG(get_logger(), "[%s] %s | fused=%s",
                   side_name, raw.to_string().c_str(),
                   fused.to_string().c_str());
    }

    // 调试图像
    if (pub_debug) {
      cv::Mat debug = (*det).debug_overlay();
      if (!debug.empty())
        pub_debug->publish(*cv_bridge::CvImage(
            std_msgs::msg::Header(), "bgr8", debug).toImageMsg());
    }
  }

  void cb_left(const sensor_msgs::msg::Image::SharedPtr msg) {
    try {
      cv::Mat frame = cv_bridge::toCvCopy(msg, "bgr8")->image;
      process_and_publish(LEFT, frame);
    } catch (const std::exception& e) {
      RCLCPP_ERROR(get_logger(), "Left cb error: %s", e.what());
    }
  }

  void cb_right(const sensor_msgs::msg::Image::SharedPtr msg) {
    try {
      cv::Mat frame = cv_bridge::toCvCopy(msg, "bgr8")->image;
      process_and_publish(RIGHT, frame);
    } catch (const std::exception& e) {
      RCLCPP_ERROR(get_logger(), "Right cb error: %s", e.what());
    }
  }

  /// @brief 定时器：一侧长时间无帧时强制输出融合结果
  void timer_cb() {
    std::lock_guard<std::mutex> lock(fusion_mutex_);
    auto fused = fusion_.fused();
    // 仅在融合结果变化时发布（定时刷新）
    if (fused.state != last_published_state_ ||
        std::abs(fused.confidence - last_published_conf_) > 0.05f) {
      auto out = std::make_unique<std_msgs::msg::String>();
      out->data = fused.to_json();
      pub_->publish(std::move(out));
      last_published_state_ = fused.state;
      last_published_conf_ = fused.confidence;
    }
  }

  enum CameraSide { LEFT, RIGHT };

  // 两个检测器（各带独立状态机）
  std::unique_ptr<BeaconDetector> det_left_;
  std::unique_ptr<BeaconDetector> det_right_;

  // 订阅
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_left_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_right_;

  // 发布
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_left_state_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_right_state_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_debug_left_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_debug_right_;

  // 融合
  BeaconFusion fusion_;
  std::mutex fusion_mutex_;
  BeaconState fuse_left_raw_;   // 左相机最新原始状态（未融合）
  BeaconState fuse_right_raw_;  // 右相机最新原始状态

  // 定时器：保证无帧时也维持输出
  rclcpp::TimerBase::SharedPtr timer_;
  BeaconState::State last_published_state_ = BeaconState::UNKNOWN;
  float last_published_conf_ = 0.0f;
};

}  // namespace color_detect

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<color_detect::ColorDetectNode>());
  rclcpp::shutdown();
  return 0;
}
