cmake_minimum_required(VERSION 3.16)

project(robobaton_4p_ros2_demo LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Keep target ROS discovery build-local so the merged install remains relocatable.
set(X5_BUILD_AMENT_PREFIX_PATH "" CACHE STRING
  "Build-only AMENT_PREFIX_PATH used to discover target ROS resources")
if(X5_BUILD_AMENT_PREFIX_PATH)
  set(ENV{AMENT_PREFIX_PATH} "${X5_BUILD_AMENT_PREFIX_PATH}")
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Threads REQUIRED)

get_filename_component(DEFAULT_ROBOBATON_LIB_DIR "${CMAKE_CURRENT_SOURCE_DIR}/lib" ABSOLUTE)
set(ROBOBATON_LIB_DIR "${DEFAULT_ROBOBATON_LIB_DIR}" CACHE PATH
  "Existing package library directory containing synchronized ABI-v2 producers")

set(ICM42688_REAL "${ROBOBATON_LIB_DIR}/libicm42688.so.2.0.0")
set(ICM42688_SONAME "${ROBOBATON_LIB_DIR}/libicm42688.so.2")
set(ICM42688_LINKER "${ROBOBATON_LIB_DIR}/libicm42688.so")
set(SC132_REAL "${ROBOBATON_LIB_DIR}/libsc132.so.2.0.0")
set(SC132_SONAME "${ROBOBATON_LIB_DIR}/libsc132.so.2")
set(SC132_LINKER "${ROBOBATON_LIB_DIR}/libsc132.so")

foreach(required_path
    "${CMAKE_CURRENT_SOURCE_DIR}/include/robobaton_4p_ros2_demo/icm42688_driver.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/include/robobaton_4p_ros2_demo/sc132camera.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/include/robobaton_4p_ros2_demo/cam_demo_common.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/include/robobaton_4p_ros2_demo/cam_demo_config.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/cam_demo_common.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/cam_demo_config.cpp"
    "${ICM42688_REAL}" "${ICM42688_SONAME}" "${ICM42688_LINKER}"
    "${SC132_REAL}" "${SC132_SONAME}" "${SC132_LINKER}")
  if(NOT EXISTS "${required_path}")
    message(FATAL_ERROR "Missing required RoboBaton ABI-v2 artifact: ${required_path}")
  endif()
endforeach()

add_library(robobaton_icm42688 SHARED IMPORTED GLOBAL)
set_target_properties(robobaton_icm42688 PROPERTIES
  IMPORTED_LOCATION "${ICM42688_REAL}"
  IMPORTED_SONAME "libicm42688.so.2"
  INTERFACE_INCLUDE_DIRECTORIES "${CMAKE_CURRENT_SOURCE_DIR}/include")

add_library(robobaton_sc132 SHARED IMPORTED GLOBAL)
set_target_properties(robobaton_sc132 PROPERTIES
  IMPORTED_LOCATION "${SC132_REAL}"
  IMPORTED_SONAME "libsc132.so.2"
  INTERFACE_INCLUDE_DIRECTORIES "${CMAKE_CURRENT_SOURCE_DIR}/include")

add_executable(robobaton_sensors_node
  src/camera_publisher.cpp
  src/imu_publisher.cpp
  src/robobaton_sensors_node.cpp
  src/cam_demo_common.cpp
  src/cam_demo_config.cpp)

ament_target_dependencies(robobaton_sensors_node rclcpp sensor_msgs)
target_include_directories(robobaton_sensors_node PRIVATE
  "${CMAKE_CURRENT_SOURCE_DIR}/include")
target_link_libraries(robobaton_sensors_node
  robobaton_icm42688
  robobaton_sc132
  Threads::Threads)
target_compile_options(robobaton_sensors_node PRIVATE
  -Wall -Wextra -Wpedantic -Werror)
target_link_options(robobaton_sensors_node PRIVATE
  "LINKER:-z,defs" "LINKER:--as-needed")

# Project producers resolve package-locally; canonical vendor search locations remain unchanged.
set_target_properties(robobaton_sensors_node PROPERTIES
  BUILD_WITH_INSTALL_RPATH TRUE
  INSTALL_RPATH "\$ORIGIN;/usr/hobot/lib;/usr/hobot/lib/sensor;/usr/lib;/lib64;/lib")

install(TARGETS robobaton_sensors_node RUNTIME DESTINATION lib/${PROJECT_NAME})
install(FILES
  "${ICM42688_REAL}" "${ICM42688_SONAME}" "${ICM42688_LINKER}"
  "${SC132_REAL}" "${SC132_SONAME}" "${SC132_LINKER}"
  DESTINATION lib/${PROJECT_NAME})

set(X5_RUNTIME_LIB_DIR "" CACHE PATH
  "X5 runtime library directory providing canonical ROS2 transitive runtime libs")
if(NOT X5_RUNTIME_LIB_DIR AND CMAKE_SYSROOT AND
   EXISTS "${CMAKE_SYSROOT}/usr/lib/aarch64-linux-gnu/libspdlog.so.1")
  set(X5_RUNTIME_LIB_DIR "${CMAKE_SYSROOT}/usr/lib/aarch64-linux-gnu")
elseif(NOT X5_RUNTIME_LIB_DIR AND
       EXISTS "/root/x5/cross_compile/new/sysroot/usr/lib/aarch64-linux-gnu/libspdlog.so.1")
  set(X5_RUNTIME_LIB_DIR "/root/x5/cross_compile/new/sysroot/usr/lib/aarch64-linux-gnu")
endif()
set(X5_RUNTIME_LIBS
  libspdlog.so libspdlog.so.1 libspdlog.so.1.9.2
  libfmt.so libfmt.so.8 libfmt.so.8.1.1
  libtinyxml2.so libtinyxml2.so.9 libtinyxml2.so.9.0.0
  libssl.so libssl.so.3
  libcrypto.so libcrypto.so.3)
foreach(runtime_lib IN LISTS X5_RUNTIME_LIBS)
  if(NOT X5_RUNTIME_LIB_DIR OR NOT EXISTS "${X5_RUNTIME_LIB_DIR}/${runtime_lib}")
    message(FATAL_ERROR "Missing X5 runtime lib: ${runtime_lib}; pass -DX5_RUNTIME_LIB_DIR=<path>")
  endif()
  install(FILES "${X5_RUNTIME_LIB_DIR}/${runtime_lib}" DESTINATION lib/${PROJECT_NAME})
endforeach()

ament_environment_hooks(
  "${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/robobaton_runtime_library_path.dsv.in"
  "${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/robobaton_runtime_library_path.sh.in")
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

ament_package()
