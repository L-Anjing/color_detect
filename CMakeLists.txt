cmake_minimum_required(VERSION 3.10)
project(color_detect)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc highgui videoio)

set(SERIAL_PREFIX_HINTS
    "$ENV{HOME}/thirdparty/serial_ws/install/serial"
    "$ENV{HOME}/thirdparty/serial_ws/install"
    "$ENV{HOME}/workspace/3rd_ws/install/serial"
    "$ENV{HOME}/workspace/3rd_ws/install"
    "${CMAKE_SOURCE_DIR}/../../thirdparty/serial_ws/install/serial"
    "${CMAKE_SOURCE_DIR}/../../thirdparty/serial_ws/install"
    "${CMAKE_SOURCE_DIR}/../../3rd_ws/install/serial"
    "${CMAKE_SOURCE_DIR}/../../3rd_ws/install"
)

if(DEFINED ENV{AMENT_PREFIX_PATH})
    string(REPLACE ":" ";" AMENT_PREFIX_HINTS "$ENV{AMENT_PREFIX_PATH}")
    list(APPEND SERIAL_PREFIX_HINTS ${AMENT_PREFIX_HINTS})
endif()

list(APPEND SERIAL_PREFIX_HINTS ${CMAKE_PREFIX_PATH})
list(REMOVE_DUPLICATES SERIAL_PREFIX_HINTS)

find_path(SERIAL_INCLUDE_DIR serial/serial.h
    HINTS ${SERIAL_PREFIX_HINTS}
    PATH_SUFFIXES include)
find_library(SERIAL_LIBRARY serial
    HINTS ${SERIAL_PREFIX_HINTS}
    PATH_SUFFIXES lib lib64)

if(NOT SERIAL_INCLUDE_DIR OR NOT SERIAL_LIBRARY)
    message(STATUS "SERIAL_PREFIX_HINTS=${SERIAL_PREFIX_HINTS}")
    message(FATAL_ERROR
        "color_detect requires serial/serial.h and libserial.so. "
        "Source your serial workspace first, e.g. "
        "source ~/thirdparty/serial_ws/install/setup.bash")
endif()
message(STATUS "[color_detect] serial include: ${SERIAL_INCLUDE_DIR}")
message(STATUS "[color_detect] serial library: ${SERIAL_LIBRARY}")
get_filename_component(SERIAL_LIBRARY_DIR "${SERIAL_LIBRARY}" DIRECTORY)

set(BEACON_SOURCES
    src/color_detect/core/color_classifier.cpp
    src/color_detect/core/beacon_protocol.cpp
    src/color_detect/core/beacon_detector.cpp
)

add_library(color_detect_core STATIC ${BEACON_SOURCES})
target_include_directories(color_detect_core PUBLIC inc)
target_link_libraries(color_detect_core PUBLIC ${OpenCV_LIBS})

add_executable(color_detect_node
    src/color_detect/nodes/color_detect_node.cpp)

target_include_directories(color_detect_node PRIVATE
    ${SERIAL_INCLUDE_DIR})

target_link_libraries(color_detect_node
    color_detect_core
    ${OpenCV_LIBS}
    ${SERIAL_LIBRARY})

ament_target_dependencies(color_detect_node rclcpp)

set_target_properties(color_detect_node PROPERTIES
    BUILD_RPATH "${SERIAL_LIBRARY_DIR}"
    INSTALL_RPATH "${SERIAL_LIBRARY_DIR}")

install(TARGETS color_detect_core color_detect_node
    DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config scripts
    DESTINATION share/${PROJECT_NAME})

install(DIRECTORY inc/
    DESTINATION include/${PROJECT_NAME}
    FILES_MATCHING PATTERN "*.hpp")

ament_package()
