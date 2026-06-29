# color_detect

`color_detect` 用于识别机器人灯带信标。当前工程主线是 ROS2 双摄版本：

`camera_stream` 发布 `/cam_left/image_raw`、`/cam_right/image_raw`，`color_detect` 订阅两路图像，分别检测后融合发布 `/color_detect/state`。

## 原理

灯带按 2 段颜色编码：

```text
[SYNC] [DATA]
```

默认协议：

| 状态 | 编码 | 输出 |
|------|------|------|
| UNKNOWN | 未检测到有效同步头，或左右结果冲突 | `0` |
| WAIT | `YELLOW + RED` | `1` |
| GO | `YELLOW + GREEN` | `2` |

检测流程：

1. 图像从 BGR 转 HSV。
2. 对 V 通道做 CLAHE，增强不均匀光照下的灯带亮度。
3. 用 S/V 阈值提取高饱和、高亮区域，过滤白色灯管和反光。
4. 形态学处理后找轮廓，用面积、长宽比、凸度、亮度均匀性筛选长条灯带。
5. 对 `minAreaRect` 长边切成 2 段，用 Hue 直方图判断每段颜色。
6. 左右相机各自输出状态，再由 `BeaconFusion` 融合。

## 坐标计算

检测器会取灯带外接旋转矩形的中心点和长边像素长度。

已知灯带真实长度 `beacon_real_length_m` 和当前相机内参 `fx/fy/cx/cy` 后，使用单目成像关系做粗定位：

```text
z = fx * real_length / length_px
x = (u - cx) * z / fx
y = (v - cy) * z / fy
yaw = atan2(x, z)
```

坐标含义：

| 字段 | 含义 |
|------|------|
| `x` | 相机坐标系横向位置，右为正，单位 m |
| `y` | 相机坐标系纵向位置，下为正，单位 m |
| `z` | 相机前方距离，单位 m |
| `yaw` | 目标相对相机光轴的水平偏角，单位 deg |

左右相机使用各自的标定内参：左图用 `pose.left.*`，右图用 `pose.right.*`。双摄同时看到目标且状态一致时，当前按左右置信度对坐标加权平均。后续如果加入左右相机到机器人坐标系的外参，可以把这里升级成机器人坐标系融合。

## 使用

启动相机发布：

```bash
ros2 launch camera_stream camera.launch.py
```

启动灯带检测：

```bash
ros2 launch color_detect color_detect.launch.py
```

启动灯带检测并通过 USB-TTL 输出到单片机：

```bash
ros2 launch color_detect color_detect_all.launch.py serial_port:=/dev/ttyACM0
```

查看融合结果：

```bash
ros2 topic echo /color_detect/state
```

开启调试图：

```bash
ros2 launch color_detect color_detect.launch.py debug:=true
ros2 run image_view image_view /color_detect/debug_left
ros2 run image_view image_view /color_detect/debug_right
```

输出示例：

```json
{"s":1,"c":0.95,"v":2,"a":3.1,"u":642.0,"p":358.0,"l":120.0,"pv":1,"x":0.01,"y":-0.01,"z":1.0,"yaw":0.6}
```

字段说明：

| 字段 | 含义 |
|------|------|
| `s` | 状态，`0=UNKNOWN`，`1=WAIT`，`2=GO` |
| `c` | 置信度 |
| `v` | 可见颜色段数量 |
| `a` | 灯带图像角度 |
| `u,p` | 灯带中心像素坐标 |
| `l` | 灯带长边像素长度 |
| `pv` | 坐标是否有效 |
| `x,y,z,yaw` | 粗定位结果 |

串口输出协议：

```text
HEAD(0xFF 0xFE) + state(1B) + x/y/z/yaw(4B float) + TAIL(0xAA 0xDD)
```

当检测无效、坐标无效或消息解析异常时，`state/x/y/z/yaw` 全部发送 0。

## 参数

配置文件：

[config/color_detect_config.yaml](/home/li/workspace/src/color_detect/config/color_detect_config.yaml)

主要参数：

| 参数 | 说明 |
|------|------|
| `camera_topic_left/right` | 左右相机图像话题，默认 `/cam_left/image_raw`、`/cam_right/image_raw` |
| `resize_width` | 检测前缩放宽度，检测结果会映射回原图坐标 |
| `clahe.*` | 亮度增强参数 |
| `hsv.*` | HSV 阈值，主要影响颜色提取 |
| `morph.*` | 形态学参数，影响碎片合并和噪声过滤 |
| `beacon.n_segments` | 灯带分段数量，当前默认 2 |
| `beacon.sync_color` | 同步头颜色 |
| `beacon.color_wait` | WAIT 数据颜色 |
| `beacon.color_go` | GO 数据颜色 |
| `beacon.min_aspect_ratio` | 灯带最小长宽比 |
| `beacon.min_area` | 最小有效面积 |
| `beacon.uniformity_threshold` | 亮度均匀性阈值，用于过滤反光和遮挡 |
| `filter.*` | 时间滤波和状态去抖参数 |
| `pose.left.fx/fy/cx/cy` | 左相机内参，按左相机标定结果填写 |
| `pose.right.fx/fy/cx/cy` | 右相机内参，按右相机标定结果填写 |
| `pose.beacon_real_length_m` | 灯带真实长度，坐标计算必须准确填写 |
| `debug` | 是否发布调试图像 |
