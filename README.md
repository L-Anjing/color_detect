# color_detect — 视觉信标通信系统

基于 LED 灯带、纯 OpenCV（无深度学习）的机器人间视觉通信系统。

支持 **ROS2 模式**（配合 camera_stream 双摄）和 **Standalone 模式**（单 USB 摄像头直读，无需 ROS2）。

## 通信协议

### 灯带编码（2段）

```
[YELLOW-SYNC] [DATA]
   固定参考      RED   → WAIT(1)
                GREEN → GO(2)
```

同步头和数据色均可通过参数配置。

### 三态输出

| 值 | 名称 | 含义 | 灯带颜色 |
|----|------|------|---------|
| 0 | UNKNOWN | 无法确定 | 无同步头 / 颜色异常 / 左右冲突 |
| 1 | WAIT | 等待 | 🟡YELLOW + 🔴RED |
| 2 | GO | 前进/动作 | 🟡YELLOW + 🟢GREEN |

## 依赖

### Standalone 模式（Ubuntu 20.04，无需 ROS2）

```bash
sudo apt install libopencv-dev
```

### ROS2 模式（Ubuntu 22.04+）

```bash
sudo apt install ros-humble-cv-bridge ros-humble-sensor-msgs libopencv-dev

# 串口（可选）
# 从 https://github.com/wjwwood/serial 编译安装
```

## 编译

### Standalone 模式（无 ROS2，本地调试用）

```bash
cd /home/li/workspace/src
bash build.sh --standalone
# 可执行文件: ./build_standalone/color_detect_standalone
```

### ROS2 模式

```bash
cd ~/workspace
colcon build --packages-select color_detect
source install/setup.bash
```

## 用法

### Standalone 模式（单 USB 摄像头直读）

```bash
# 默认 /dev/video0
./build_standalone/color_detect_standalone

# 指定设备和配置文件
./build_standalone/color_detect_standalone \
    --device /dev/video1 \
    --config color_detect/config/color_detect_config.yaml
```

按键：`q`=退出  `d`=切换调试叠加  `f`=全屏

### ROS2 模式（双摄 + 串口）

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

### CLAHE（自适应直方图均衡）

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `clahe.enabled` | true | 启用 CLAHE，改善不均匀光照 |
| `clahe.clip_limit` | 2.0 | 对比度限制 |
| `clahe.grid_size` | 8 | 网格大小 |

### HSV 阈值

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `hsv.v_threshold` | 200 | V 通道基础阈值，LED 主动发光特征 |
| `hsv.s_threshold_white` | 40 | 白色/低饱和区分阈值 |
| `hsv.saturation_threshold` | 100 | S 通道二值化阈值（**过滤白色灯管**） |
| `hsv.hue_tolerance` | 12 | H 通道匹配容差 |

### 形态学

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `morph.open_size` | 3 | 开运算核大小（去噪） |
| `morph.close_size` | 7 | 闭运算核大小（填孔） |
| `morph.dilation_size` | 3 | 膨胀核大小（**合并碎片**） |

### 协议颜色

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `beacon.sync_color` | YELLOW | 同步头颜色 |
| `beacon.color_wait` | RED | WAIT 对应的数据色 |
| `beacon.color_go` | GREEN | GO 对应的数据色 |

### 候选筛选

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `beacon.min_aspect_ratio` | 3.0 | 最小长宽比 |
| `beacon.min_area` | 50 | 最小面积(px) |
| `beacon.uniformity_threshold` | 0.3 | 亮度均匀性门槛（**std/mean，过滤反光**） |

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
① 等比例缩放 → 640px 宽
② BGR → HSV
③ CLAHE ▷ V 通道自适应直方图均衡（改善不均匀光照）
④ S 通道阈值（S > 100）→ 过滤白色灯管/低饱和反光
⑤ 自适应 V 阈值（max(固定值, 亮区平均V×0.85)）
⑥ 合并 mask = S_mask & V_mask
⑦ 形态学：开运算去噪 → 闭运算填孔 → 膨胀合并碎片
⑧ 轮廓提取
⑨ 面积 + 长宽比 + 凸度筛选
⑩ 亮度均匀性筛选（std/mean < 0.3，拒绝反光/局部遮挡）
⑪ minAreaRect → 主轴方向
⑫ 沿主轴分割为 2 段 ROI
⑬ 每段 Hue 直方图分类（36-bin 峰值检测，比像素投票更抗噪）
⑭ 置信度计算 + 一致性检查
⑮ 协议解码：找到同步色 → 读取数据色 → 映射为三态
⑯ 状态机：滑动窗口多数决 + 去抖 + 丢失保持
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
│   ├── color_classifier.hpp    HSV 颜色分类器（含 Hue 直方图分类）
│   ├── beacon_protocol.hpp     2段协议解码
│   ├── state_machine.hpp       5态状态机（去抖+丢失恢复）
│   ├── beacon_detector.hpp     核心检测器
│   └── beacon_fusion.hpp       双相机融合投票
├── src/color_detect/
│   ├── *.cpp                   (同上)
│   ├── color_detect_node.cpp   ROS2 双摄检测节点
│   ├── color_detect_standalone.cpp  Standalone 单摄节点（无需 ROS2）
│   └── serial_bridge_node.cpp  串口桥接节点
├── config/color_detect_config.yaml
├── launch/color_detect*.launch.py
└── CMakeLists.txt
```

## 本地测试（无 ROS2）

用电脑屏幕模拟信标：

```bash
# 1. 生成测试图
python3 generate_test_beacon.py
# → beacon_wait.png（🟡+🔴=WAIT）
# → beacon_go.png（🟡+🟢=GO）

# 2. 全屏显示测试图，摄像头对准屏幕

# 3. 运行检测
./build_standalone/color_detect_standalone --device /dev/video0
```
