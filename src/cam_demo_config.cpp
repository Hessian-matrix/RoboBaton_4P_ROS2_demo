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

// 单颗30/60fps分别使用匹配sensor/MIPI时序的master profile，不修改四路默认profile选择。
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

  // 30/60fps必须匹配profile内部sensor和MIPI时序，不能仅靠GPIO节拍限帧。
  const char* profile = options.fps == 30 ? kSc132Single30FpsProfile
                                          : kSc132Single60FpsProfile;
  if (setenv(kSc132SensorProfileEnv, profile, 1) != 0) {
    throw std::runtime_error("set SC132_SENSOR_PROFILE failed");
  }
  std::cout << "Auto selected single-sensor " << options.fps << "fps profile\n";
}

}  // namespace robobaton_demo
