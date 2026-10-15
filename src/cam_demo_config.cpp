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
    case 40:
    case 50:
    case 60:
      break;
    default:
      throw std::invalid_argument("camera.fps must be 25, 30, 40, 50, or 60");
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
  if (options.trigger_mode != "vin_lpwm" && options.trigger_mode != "none") {
    throw std::invalid_argument("camera.trigger_mode must be vin_lpwm or none");
  }
}

// 功能：把触发模式写入 libsc132 使用的环境变量。
// 输入：options.trigger_mode，公开只接受 vin_lpwm、none。
// 环境变量已预设 SC132_TRIGGER_MODE 时保留预设值，供 software_gpio 调试路径直驱。
// 副作用：设置进程内环境变量 SC132_TRIGGER_MODE；触发线路由 libsc132 的板级绑定决定。
void ConfigureSc132TriggerMode(const Options& options) {
  const char* preset = std::getenv(kSc132TriggerModeEnv);
  if (preset != nullptr && preset[0] != '\0') {
    std::cout << kSc132TriggerModeEnv << "=" << preset
              << " (preset env retained for software_gpio debug)\n";
    return;
  }
  if (setenv(kSc132TriggerModeEnv, options.trigger_mode.c_str(), 1) != 0) {
    throw std::runtime_error("set SC132_TRIGGER_MODE failed");
  }
  std::cout << kSc132TriggerModeEnv << "=" << options.trigger_mode
            << " (trigger ownership is resolved by libsc132)\n";
}

// 单颗30fps使用30fps master profile；25/40/50/60fps使用兼容的60fps base master profile并由libsc132写入目标VTS。
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

  // 25/40/50/60fps由60fps base profile派生；30fps使用匹配的master profile。
  const char* profile = options.fps == 30 ? kSc132Single30FpsProfile
                                          : kSc132Single60FpsProfile;
  if (setenv(kSc132SensorProfileEnv, profile, 1) != 0) {
    throw std::runtime_error("set SC132_SENSOR_PROFILE failed");
  }
  std::cout << "Auto selected single-sensor " << options.fps << "fps profile\n";
}

}  // namespace robobaton_demo
