#!/bin/bash
# Minimal autostart entry for startup_applications.
# Keep this script intentionally simple: open a visible gnome-terminal and
# launch color_detect inside it.

set -e

WORKSPACE_DIR=${WORKSPACE_DIR:-/home/pi/workspace/camera_ws}
CONFIG_FILE="$WORKSPACE_DIR/src/color_detect/config/color_detect_config.yaml"

if [ ! -f "$CONFIG_FILE" ]; then
    CONFIG_FILE="$WORKSPACE_DIR/install/color_detect/share/color_detect/config/color_detect_config.yaml"
fi

if [ ! -f "$CONFIG_FILE" ]; then
    echo "[COLOR-START] config not found: $CONFIG_FILE"
    exit 1
fi

# Do this outside the terminal command. If pkill -f runs inside bash -c, it can
# match the bash command line itself because the command contains color_detect_node.
pkill -f "ros2 run color_detect color_detect_node" 2>/dev/null || true
pkill -f "/color_detect_node" 2>/dev/null || true

exec gnome-terminal --title="Color Detect" -- bash -c "\
echo '[COLOR-START] waiting desktop...'; \
sleep 5; \
export DISPLAY=\${DISPLAY:-:0}; \
export XAUTHORITY=\${XAUTHORITY:-/home/pi/.Xauthority}; \
echo '[COLOR-START] source ROS'; \
source /opt/ros/humble/setup.bash; \
if [ -f /home/pi/thirdparty/serial_ws/install/setup.bash ]; then source /home/pi/thirdparty/serial_ws/install/setup.bash; fi; \
if [ -f '$WORKSPACE_DIR/install/setup.bash' ]; then source '$WORKSPACE_DIR/install/setup.bash'; fi; \
cd '$WORKSPACE_DIR'; \
echo '[COLOR-START] run color_detect_node'; \
ros2 run color_detect color_detect_node --ros-args --params-file '$CONFIG_FILE'; \
echo '[COLOR-START] color_detect_node exited'; \
exec bash"
