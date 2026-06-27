# color_detect — 视觉信标通信系统

基于 LED 灯带、纯 OpenCV（无深度学习）的机器人间视觉通信系统。

## 架构

```
                    ┌──────────────────────┐
                    │   camera_stream 包    │
                    │  /cam_left/image_raw  │
                    │  /cam_right/image_raw │
                    └──────┬──────────┬────┘
                           │          │
                           ▼          ▼
                    ┌──────────────────────┐
                    │   color_detect_node   │
                    │                      │
                    │  DetectorLeft  ─┐    │
                    │                ├ Fusion
                    │  DetectorRight ─┘    │
                    │                      │
                    │  StateMachine(去抖)   │
                    └──────────┬───────────┘
                               │
                               ▼
                    ┌──────────────────────┐
                    │  /color_detect/state  │
                    │  {"s":1,"c":0.95,...} │
                    └──────────┬───────────┘
                               │
                               ▼
                    ┌──────────────────────┐
                    │ serial_bridge_node    │
                    │ FF FE + state + AA DD │
                    └──────────────────────┘
```

## 通信协议

### 灯带编码（2段）

```
[YELLOW-SYNC] [DATA]
   固定参考      CYAN → WAIT(1)
                MAGENTA → GO(2)
```

同步头和数据色均可通过参数配置。

### 三态输出

| 值 | 名称 | 含义 | 灯带状态 |
|----|------|------|---------|
| 0 | UNKNOWN | 无法确定 | 无同步头 / 颜色异常 / 左右冲突 |
| 1 | WAIT | 等待 | SYNC + CYAN |
| 2 | GO | 前进/动作 | SYNC + MAGENTA |

## 依赖

```bash
sudo apt install ros-humble-cv-bridge ros-humble-sensor-msgs libopencv-dev

# 串口（可选）
# 从 https://github.com/wjwwood/serial 编译安装
```

## 编译

```bash
cd ~/workspace
colcon build --packages-select color_detect
source install/setup.bash
```

## 用法

```bash
# 检测 + 串口（摄像头由 camera_stream 独立提供）
ros2 launch color_detect color_detect_all.launch.py \
  serial_port:=/dev/ttyACM0

# 仅检测（无串口）
ros2 run color_detect color_detect_node --ros-args \
  -p camera_topic_left:=/cam_left/image_raw \
  -p camera_topic_right:=/cam_right/image_raw \
  -p debug:=true

# 查看检测结果
ros2 topic echo /color_detect/state
# → {"s":1,"c":0.95,"v":2,"a":12.3}

# 调试画面
ros2 run image_view image_view /color_detect/debug_left
```

## 参数

### 摄像头话题

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `camera_topic_left` | `/cam_left/image_raw` | 左相机话题 |
| `camera_topic_right` | `/cam_right/image_raw` | 右相机话题 |

### HSV 阈值

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `hsv.v_threshold` | 200 | V通道阈值，LED 主动发光特征 |
| `hsv.s_threshold_white` | 40 | 白色/低饱和区分阈值 |
| `hsv.hue_tolerance` | 12 | H通道匹配容差 |

### 协议颜色

| 参数 | 默认值 | 说明 |

|------|--------|------|
| `beacon.sync_color` | YELLOW | 同步头颜色 |
| `beacon.color_wait` | CYAN | WAIT 对应的数据色 |
| `beacon.color_go` | MAGENTA | GO 对应的数据色 |

### 滤波

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `filter.temporal_window` | 5 | 滑动窗口帧数 |
| `filter.debounce_frames` | 3 | 状态去抖连续帧数 |
| `filter.lost_timeout_ms` | 200 | 目标丢失超时(ms) |

## 检测流程

```
输入帧 (bgr8)
  ↓
1. 等比例缩放 → 640px 宽
2. BGR → HSV
3. V通道阈值 (V > 200) → 高亮区域 mask
4. 形态学开闭 → 去噪 + 连接
5. 轮廓提取 + 形状筛选（宽高比 > 3, 凸性 > 0.85）
6. minAreaRect → 主轴方向
7. 沿主轴分割为 2 段 ROI
8. 每段 HSV 颜色分类 + 一致性检查
9. 协议解码：找到同步色 → 读取数据色 → 映射为三态
10. 状态机：滑动窗口多数决 + 去抖 + 丢失保持
  ↓
输出融合状态
```

## 串口协议

```
 HEAD(0xFF 0xFE) + state(1B) + TAIL(0xAA 0xDD)
                   ↓
             0 = UNKNOWN
             1 = WAIT
             2 = GO
```

## 文件结构

```
color_detect/
├── inc/color_detect/
│   ├── beacon_config.hpp       参数配置
│   ├── color_classifier.hpp    HSV 颜色分类器
│   ├── beacon_protocol.hpp     2段协议解码
│   ├── state_machine.hpp       5态状态机（去抖+丢失恢复）
│   ├── beacon_detector.hpp     核心检测器（6步流程）
│   └── beacon_fusion.hpp       双相机融合投票
├── src/color_detect/
│   ├── *.cpp                   (同上)
│   ├── color_detect_node.cpp   ROS2 双摄检测节点
│   └── serial_bridge_node.cpp  串口桥接节点
├── config/color_detect_config.yaml
├── launch/color_detect*.launch.py
└── CMakeLists.txt
```
