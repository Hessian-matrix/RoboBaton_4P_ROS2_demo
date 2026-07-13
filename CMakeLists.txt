cmake_minimum_required(VERSION 3.16)

project(robobaton_4p_ros2_demo LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Threads REQUIRED)


set(X5_USR_X5_INCLUDE_DIR "" CACHE PATH
  "X5 usr_x5 include directory that provides hb_mem_mgr.h and hbn_api.h")
if(NOT X5_USR_X5_INCLUDE_DIR AND CMAKE_SYSROOT AND EXISTS "${CMAKE_SYSROOT}/usr_x5/include/hb_mem_mgr.h")
  set(X5_USR_X5_INCLUDE_DIR "${CMAKE_SYSROOT}/usr_x5/include")
elseif(NOT X5_USR_X5_INCLUDE_DIR AND EXISTS "/root/x5/cross_compile/new/sysroot/usr_x5/include/hb_mem_mgr.h")
  set(X5_USR_X5_INCLUDE_DIR "/root/x5/cross_compile/new/sysroot/usr_x5/include")
endif()

set(X5_HOBOT_LIB_DIR "" CACHE PATH
  "X5 hobot library directory that provides libhbmem.so")
if(NOT X5_HOBOT_LIB_DIR AND CMAKE_SYSROOT AND EXISTS "${CMAKE_SYSROOT}/usr_x5/hobot/lib/libhbmem.so")
  set(X5_HOBOT_LIB_DIR "${CMAKE_SYSROOT}/usr_x5/hobot/lib")
elseif(NOT X5_HOBOT_LIB_DIR AND EXISTS "/root/x5/cross_compile/new/sysroot/usr_x5/hobot/lib/libhbmem.so")
  set(X5_HOBOT_LIB_DIR "/root/x5/cross_compile/new/sysroot/usr_x5/hobot/lib")
endif()

get_filename_component(DEFAULT_ROBOBATON_LIB_DIR
  "${CMAKE_CURRENT_SOURCE_DIR}/lib" ABSOLUTE)
# 2026-07-13 修改原因：ROS2 包固定从包内 lib/ 取 SO，保持本工程可独立配置和构建。
set(ROBOBATON_LIB_DIR "${DEFAULT_ROBOBATON_LIB_DIR}" CACHE PATH
  "Path to updated RoboBaton shared libraries")
set(ICM42688_SO "${ROBOBATON_LIB_DIR}/libicm42688.so")
set(SC132_SO "${ROBOBATON_LIB_DIR}/libsc132.so")
set(HBMEM_SO "${X5_HOBOT_LIB_DIR}/libhbmem.so")

set(X5_RUNTIME_LIB_DIR "" CACHE PATH
  "X5 runtime library directory that provides ROS2 transitive runtime libs")
if(NOT X5_RUNTIME_LIB_DIR AND CMAKE_SYSROOT AND EXISTS "${CMAKE_SYSROOT}/usr/lib/aarch64-linux-gnu/libspdlog.so.1")
  set(X5_RUNTIME_LIB_DIR "${CMAKE_SYSROOT}/usr/lib/aarch64-linux-gnu")
elseif(NOT X5_RUNTIME_LIB_DIR AND EXISTS "/root/x5/cross_compile/new/sysroot/usr/lib/aarch64-linux-gnu/libspdlog.so.1")
  set(X5_RUNTIME_LIB_DIR "/root/x5/cross_compile/new/sysroot/usr/lib/aarch64-linux-gnu")
endif()

set(X5_RUNTIME_LIBS
  libspdlog.so libspdlog.so.1 libspdlog.so.1.9.2
  libfmt.so libfmt.so.8 libfmt.so.8.1.1
  libtinyxml2.so libtinyxml2.so.9 libtinyxml2.so.9.0.0
  libssl.so libssl.so.3
  libcrypto.so libcrypto.so.3
)

foreach(runtime_lib IN LISTS X5_RUNTIME_LIBS)
  if(NOT X5_RUNTIME_LIB_DIR OR NOT EXISTS "${X5_RUNTIME_LIB_DIR}/${runtime_lib}")
    message(FATAL_ERROR "Missing X5 runtime lib: ${runtime_lib}; pass -DX5_RUNTIME_LIB_DIR=<path>")
  endif()
endforeach()

foreach(required_path
    "${CMAKE_CURRENT_SOURCE_DIR}/include/robobaton_4p_ros2_demo/icm42688_driver.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/include/robobaton_4p_ros2_demo/sc132camera.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/include/robobaton_4p_ros2_demo/cam_demo_common.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/include/robobaton_4p_ros2_demo/cam_demo_config.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/cam_demo_common.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/cam_demo_config.cpp"
    "${ICM42688_SO}"
    "${SC132_SO}")
  if(NOT EXISTS "${required_path}")
    message(FATAL_ERROR "Missing required RoboBaton demo artifact: ${required_path}")
  endif()
endforeach()

if(NOT X5_USR_X5_INCLUDE_DIR OR NOT EXISTS "${X5_USR_X5_INCLUDE_DIR}/hb_mem_mgr.h")
  message(FATAL_ERROR "Missing X5_USR_X5_INCLUDE_DIR with hb_mem_mgr.h; pass -DX5_USR_X5_INCLUDE_DIR=<path>")
endif()

if(NOT X5_HOBOT_LIB_DIR OR NOT EXISTS "${HBMEM_SO}")
  message(FATAL_ERROR "Missing X5_HOBOT_LIB_DIR with libhbmem.so; pass -DX5_HOBOT_LIB_DIR=<path>")
endif()

add_library(robobaton_icm42688 SHARED IMPORTED GLOBAL)
set_target_properties(robobaton_icm42688 PROPERTIES
  IMPORTED_LOCATION "${ICM42688_SO}"
  INTERFACE_INCLUDE_DIRECTORIES "${CMAKE_CURRENT_SOURCE_DIR}/include"
  IMPORTED_NO_SONAME TRUE
)

add_library(robobaton_sc132 SHARED IMPORTED GLOBAL)
set_target_properties(robobaton_sc132 PROPERTIES
  IMPORTED_LOCATION "${SC132_SO}"
  INTERFACE_INCLUDE_DIRECTORIES "${CMAKE_CURRENT_SOURCE_DIR}/include;${X5_USR_X5_INCLUDE_DIR}"
  IMPORTED_NO_SONAME TRUE
)

add_library(hbmem SHARED IMPORTED GLOBAL)
set_target_properties(hbmem PROPERTIES
  IMPORTED_LOCATION "${HBMEM_SO}"
  INTERFACE_INCLUDE_DIRECTORIES "${X5_USR_X5_INCLUDE_DIR}"
)

add_executable(robobaton_sensors_node
  src/camera_publisher.cpp
  src/imu_publisher.cpp
  src/robobaton_sensors_node.cpp
  src/cam_demo_common.cpp
  src/cam_demo_config.cpp
)

ament_target_dependencies(robobaton_sensors_node
  rclcpp
  sensor_msgs
)

target_include_directories(robobaton_sensors_node PRIVATE
  "${CMAKE_CURRENT_SOURCE_DIR}/include"
  "${X5_USR_X5_INCLUDE_DIR}"
)

target_link_libraries(robobaton_sensors_node
  robobaton_icm42688
  robobaton_sc132
  hbmem
  Threads::Threads
)

target_compile_options(robobaton_sensors_node PRIVATE
  -Wall
  -Wextra
  -Wpedantic
)

target_link_options(robobaton_sensors_node PRIVATE
  "LINKER:--allow-shlib-undefined"
)

set_target_properties(robobaton_sensors_node PROPERTIES
  BUILD_RPATH "${ROBOBATON_LIB_DIR};${X5_HOBOT_LIB_DIR};/usr/hobot/lib;/usr/hobot/lib/sensor;/usr/lib;/lib64;/lib"
  INSTALL_RPATH "\$ORIGIN;/usr/hobot/lib;/usr/hobot/lib/sensor;/usr/lib;/lib64;/lib"
)
install(TARGETS robobaton_sensors_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(FILES "${ICM42688_SO}" "${SC132_SO}"
  DESTINATION lib/${PROJECT_NAME}
)


foreach(runtime_lib IN LISTS X5_RUNTIME_LIBS)
  install(FILES "${X5_RUNTIME_LIB_DIR}/${runtime_lib}"
    DESTINATION lib/${PROJECT_NAME}
  )
endforeach()

ament_environment_hooks(
  "${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/robobaton_runtime_library_path.dsv.in"
  "${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/robobaton_runtime_library_path.sh.in"
)


install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
