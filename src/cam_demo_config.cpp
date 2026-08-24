#include "robobaton_4p_ros2_demo/cam_demo_config.h"

#include <cstdlib>
#include <iostream>
#include <stdexcept>

namespace robobaton_demo {
namespace {

constexpr const char* kSc132SensorProfileEnv = "SC132_SENSOR_PROFILE";
constexpr const char* kSc132TriggerModeEnv = "SC132_TRIGGER_MODE";
constexpr const char* kSc132Single30FpsProfile =
    "sc132gs_linear_1088x1280_raw10_30fps_1lane";
constexpr const char* kSc132Single60FpsProfile =
    "sc132gs_linear_1088x1280_raw10_60fps_1lane";

}  // namespace

void ValidateCameraOptions(const Options& options) {
  switch (options.camera_mask) {
    case 0x1U:
    case 0x2U:
    case 0x4U:
    case 0x8U:
    case 0xFU:
      break;
    default:
      throw std::invalid_argument("camera.camera_mask supports only 1, 2, 4, 8, or 15");
  }

  switch (options.fps) {
    case 25:
    case 30:

      break;
    default:
      throw std::invalid_argument("camera.fps must be 25 or 30");
  }

  switch (options.rotate_degrees) {
    case 0:
    case 90:
    case 180:
    case 270:
      break;
    default:
      throw std::invalid_argument("camera.rotate_degrees must be 0, 90, 180, or 270");
  }
  if (options.rotate_degrees == 180 && options.fps != 30) {
    throw std::invalid_argument("camera.rotate_degrees=180 is supported only at 30fps");
  }
  if (options.frame_set_timeout_ms == 0U || options.frame_set_max_skew_ns == 0U) {
    throw std::invalid_argument("camera frame-set timeout and max skew must be positive");
  }
  if (options.trigger_mode != "software_gpio" && options.trigger_mode != "none") {
    throw std::invalid_argument("camera.trigger_mode must be software_gpio or none");
  }
}

// 功能：把 ROS 参数选择的触发模式写入 libsc132 使用的环境变量。
// 输入：options.trigger_mode，接受 software_gpio 或 none；V1 仅验证 software_gpio。
// 副作用：覆盖当前进程的 SC132_TRIGGER_MODE；software_gpio 模式使用 GPIO417。
void ConfigureSc132TriggerMode(const Options& options) {
  // ROS 参数优先于 shell 环境，确保进程按显式配置启动。
  if (setenv(kSc132TriggerModeEnv, options.trigger_mode.c_str(), 1) != 0) {
    throw std::runtime_error("set SC132_TRIGGER_MODE failed");
  }
  std::cout << kSc132TriggerModeEnv << "=" << options.trigger_mode
            << " (GPIO417 is used when mode=software_gpio)\n";
}

// 单颗30fps使用30fps master profile；25fps使用兼容的60fps base master profile并由libsc132写入目标VTS。
// 显式SC132_SENSOR_PROFILE优先且不覆盖；自动选择仅修改当前进程环境，供后续libsc132初始化读取。
void ConfigureSc132SensorProfile(const Options& options) {
  const char* current_profile = std::getenv(kSc132SensorProfileEnv);
  if (current_profile != nullptr && current_profile[0] != '\0') {
    std::cout << "SC132 sensor profile already configured\n";
    return;
  }

  // 单颗 sensor 必须用 master profile；30fps slave-right 在当前板卡 vflow_start 返回 -36。
  if (CameraMaskPopCount(options.camera_mask) != 1) {
    return;
  }

  // 25fps由60fps base profile派生；30fps使用匹配的master profile。
  const char* profile = options.fps == 30 ? kSc132Single30FpsProfile
                                          : kSc132Single60FpsProfile;
  if (setenv(kSc132SensorProfileEnv, profile, 1) != 0) {
    throw std::runtime_error("set SC132_SENSOR_PROFILE failed");
  }
  std::cout << "Auto selected single-sensor " << options.fps << "fps profile\n";
}

}  // namespace robobaton_demo
