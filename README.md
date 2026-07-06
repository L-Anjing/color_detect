# color_detect

Single main pipeline for the light-strip beacon:

```text
/dev/cam_right
  -> low-exposure V4L2 capture
  -> light-strip detection
  -> red => WAIT, else GO
  -> serial packet CA CA <state> AA BB
```

State byte:

```text
0 = GO
1 = WAIT
2 = STOP FUNC
```

## Build

```bash
cd ~/workspace/camera_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select color_detect
source install/setup.bash
```

## Run

```bash
bash src/color_detect/scripts/startup.sh
```

## Main Config

Edit `config/color_detect_config.yaml`. This is the main runtime config file.
`startup.sh` now loads this file directly instead of carrying a second copy of
camera / serial / display defaults.

Use it for:

```text
direct_camera.*  camera device, resolution, FPS, MJPG, low exposure
serial.*         serial port, baudrate, stop command
display.*        local OpenCV windows
hsv.*            color thresholds
roi.*            detection crop
morph.*          morphology cleanup
beacon.*         strip geometry and red WAIT trigger
```
