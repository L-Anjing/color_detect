cmake_minimum_required(VERSION 3.10)
project(color_detect)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ---- 构建模式选项 ----
option(BUILD_STANDALONE "Build standalone color_detect (no ROS2)" OFF)

# ═════════════════════════════════════════════
#  独立模式 — 无 ROS2，直接读取 /dev/video*
# ═════════════════════════════════════════════
if(BUILD_STANDALONE)

  find_package(OpenCV REQUIRED COMPONENTS core imgproc highgui videoio)

  # ---- 包含路径 ----
  include_directories(inc)

  # ── Beacon 核心库（纯 OpenCV） ──
  set(BEACON_SOURCES
      src/color_detect/color_classifier.cpp
      src/color_detect/beacon_protocol.cpp
      src/color_detect/state_machine.cpp
      src/color_detect/beacon_detector.cpp
      src/color_detect/beacon_fusion.cpp
  )

  add_library(color_detect_core STATIC ${BEACON_SOURCES})
  target_include_directories(color_detect_core PUBLIC inc)
  target_link_libraries(color_detect_core PUBLIC ${OpenCV_LIBS})

  # ── 独立运行节点 ──
  add_executable(color_detect_standalone
      src/color_detect/color_detect_standalone.cpp)

  target_include_directories(color_detect_standalone PRIVATE inc)
  target_link_libraries(color_detect_standalone
      color_detect_core
      ${OpenCV_LIBS})

  message(STATUS "[color_detect] Standalone mode — no ROS2 required")
  message(STATUS "[color_detect]   Target: color_detect_standalone")
  return()

endif()

# ═════════════════════════════════════════════
#  ROS2 模式 — 标准 ROS2 节点
# ═════════════════════════════════════════════

# ---- ROS2 + OpenCV 依赖 ----
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc)

# ---- Serial 库（可选，默认尝试常见路径） ----
find_path(SERIAL_INCLUDE_DIR serial/serial.h
    HINTS
        /usr/include
        /usr/local/include
        ${CMAKE_SOURCE_DIR}/../../3rd_ws/install/serial/include
    NO_DEFAULT_PATH)
find_library(SERIAL_LIBRARY serial
    HINTS
        /usr/lib
        /usr/local/lib
        ${CMAKE_SOURCE_DIR}/../../3rd_ws/install/serial/lib
    NO_DEFAULT_PATH)

if(SERIAL_INCLUDE_DIR AND SERIAL_LIBRARY)
    message(STATUS "[color_detect] Serial library FOUND — building serial_bridge_node")
    set(HAS_SERIAL TRUE)
    get_filename_component(SERIAL_LIBRARY_DIR "${SERIAL_LIBRARY}" DIRECTORY)
else()
    message(STATUS "[color_detect] Serial library NOT FOUND — skipping serial_bridge_node")
    set(HAS_SERIAL FALSE)
endif()

# ---- 包含路径 ----
include_directories(inc ${OpenCV_INCLUDE_DIRS})

# ─────────────────────────────────────────────
#  Beacon 核心库（静态库，纯 OpenCV，可被其他节点复用）
# ─────────────────────────────────────────────
set(BEACON_SOURCES
    src/color_detect/color_classifier.cpp
    src/color_detect/beacon_protocol.cpp
    src/color_detect/state_machine.cpp
    src/color_detect/beacon_detector.cpp
    src/color_detect/beacon_fusion.cpp
)

add_library(color_detect_core STATIC ${BEACON_SOURCES})
target_include_directories(color_detect_core PUBLIC inc)
target_link_libraries(color_detect_core PUBLIC ${OpenCV_LIBS})
# 注意：核心库源代码不依赖任何 ROS2 头文件，无需 ament_target_dependencies

# ─────────────────────────────────────────────
#  检测节点
# ─────────────────────────────────────────────
add_executable(color_detect_node
    src/color_detect/color_detect_node.cpp)

target_link_libraries(color_detect_node
    color_detect_core
    ${OpenCV_LIBS})

ament_target_dependencies(color_detect_node
    rclcpp std_msgs sensor_msgs cv_bridge)

# ─────────────────────────────────────────────
#  Serial 桥接节点（可选）
# ─────────────────────────────────────────────
if(HAS_SERIAL)
    add_executable(serial_bridge_node
        src/color_detect/serial_bridge_node.cpp)

    ament_target_dependencies(serial_bridge_node
        rclcpp std_msgs)

    target_include_directories(serial_bridge_node PRIVATE
        ${SERIAL_INCLUDE_DIR})

    target_link_libraries(serial_bridge_node
        ${SERIAL_LIBRARY})

    set_target_properties(serial_bridge_node PROPERTIES
        BUILD_RPATH "${SERIAL_LIBRARY_DIR}"
        INSTALL_RPATH "${SERIAL_LIBRARY_DIR}")
endif()

# ─────────────────────────────────────────────
#  安装
# ─────────────────────────────────────────────
set(INSTALL_TARGETS color_detect_core color_detect_node)
if(HAS_SERIAL)
    list(APPEND INSTALL_TARGETS serial_bridge_node)
endif()

install(TARGETS ${INSTALL_TARGETS}
    DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config
    DESTINATION share/${PROJECT_NAME})

install(DIRECTORY inc/
    DESTINATION include/${PROJECT_NAME}
    FILES_MATCHING PATTERN "*.hpp")

# ---- Ament ----
ament_package()
