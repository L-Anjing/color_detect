#ifndef COLOR_DETECT_BEACON_FUSION_HPP_
#define COLOR_DETECT_BEACON_FUSION_HPP_

#include "color_detect/beacon_protocol.hpp"

namespace color_detect {

/// @brief 双相机状态融合器
///
/// 融合规则：
///   Left    Right     Result
///   ──────  ──────    ──────
///   A       A         A
///   B       B         B
///   A       UNKNOWN   A
///   UNKNOWN B         B
///   UNKNOWN UNKNOWN   UNKNOWN
///   A       B         UNKNOWN  ← 冲突，不猜
///
/// 时序说明：
///   左右相机帧率可能不同步，融合器缓存每个相机最新的有效状态，
///   任一相机来新帧即触发重新融合。
class BeaconFusion {
public:
  BeaconFusion() = default;

  /// @brief 输入左相机检测结果
  void update_left(const BeaconState& state);

  /// @brief 输入右相机检测结果
  void update_right(const BeaconState& state);

  /// @brief 获取当前融合结果
  BeaconState fused() const;

  /// @brief 左/右各自最新状态（调试用）
  const BeaconState& left_state()  const { return left_; }
  const BeaconState& right_state() const { return right_; }

  /// @brief 是否至少有1个相机看到有效状态
  bool has_valid_detection() const;

  /// @brief 重置
  void reset();

private:
  BeaconState left_;
  BeaconState right_;
};

}  // namespace color_detect

#endif  // COLOR_DETECT_BEACON_FUSION_HPP_
