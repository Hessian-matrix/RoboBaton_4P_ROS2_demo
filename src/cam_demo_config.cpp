#include "robobaton_4p_ros2_demo/cam_demo_config.h"

#include <cstdlib>
#include <iostream>
#include <stdexcept>

namespace robobaton_demo {
namespace {

constexpr const char* kSc132SensorProfileEnv = "SC132_SENSOR_PROFILE";
constexpr const char* kSc132TriggerModeEnv = "SC132_TRIGGER_MODE";
constexpr const char* kSc132Single60FpsProfile =
    "sc132gs_linear_1088x1280_raw10_60fps_1lane";

}  // namespace

// 功能：把 ROS 参数选择的触发模式写入 libsc132 使用的环境变量。
// 输入：options.trigger_mode，支持 software_gpio、vin_lpwm、none 等已验证值。
// 副作用：覆盖当前进程的 SC132_TRIGGER_MODE；software_gpio 模式使用 GPIO417。
void ConfigureSc132TriggerMode(const Options& options) {
  // ROS 参数优先于 shell 环境，确保进程按显式配置启动。
  if (setenv(kSc132TriggerModeEnv, options.trigger_mode.c_str(), 1) != 0) {
    throw std::runtime_error("set SC132_TRIGGER_MODE failed");
  }
  std::cout << kSc132TriggerModeEnv << "=" << options.trigger_mode
            << " (GPIO417 is used when mode=software_gpio)\n";
}

// 功能：为内部单颗 sensor smoke 自动补齐 60fps sensor profile。
// 输入：options.camera_mask/options.fps。
// 副作用：当内部诊断只启用一颗 sensor 且未预设 SC132_SENSOR_PROFILE 时设置兼容 profile。
void ConfigureSc132SensorProfile(const Options& options) {
  const char* current_profile = std::getenv(kSc132SensorProfileEnv);
  if (current_profile != nullptr && current_profile[0] != '\0') {
    std::cout << "SC132 sensor profile already configured\n";
    return;
  }

  // 四路同步使用 libsc132 默认 profile；单颗 60fps 需要匹配 SDK pipeline 的 1-lane profile。
  if (CameraMaskPopCount(options.camera_mask) != 1 || options.fps != 60) {
    return;
  }

  // setenv 仅影响当前进程，不修改板端全局 shell 环境。
  if (setenv(kSc132SensorProfileEnv, kSc132Single60FpsProfile, 1) != 0) {
    throw std::runtime_error("set SC132_SENSOR_PROFILE failed");
  }
  std::cout << "Auto selected single-sensor 60fps profile\n";
}

}  // namespace robobaton_demo
