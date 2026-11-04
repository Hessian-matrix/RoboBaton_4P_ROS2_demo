#pragma once

#include "robobaton_4p_ros2_demo/cam_demo_common.h"

namespace robobaton_demo {

// 功能：在任何硬件或进程环境副作用之前校验ROS相机参数合同。
// 输入：options；相机仅支持单颗或四颗、25/30/40/50/60fps以及公开旋转集合。
// 异常：参数或组合不受支持时抛出std::invalid_argument。
void ValidateCameraOptions(const Options& options);

// Configure startup-only trigger ownership and camera timestamp selection.
// Sets SC132_TRIGGER_MODE and SC132_TIMESTAMP_PATH before libsc132 starts.
void ConfigureSc132TriggerMode(const Options& options);

// 功能：为特定启动组合选择兼容的 sensor profile。
// 输入：运行参数 options。
// 副作用：必要时设置进程内环境变量 SC132_SENSOR_PROFILE。
void ConfigureSc132SensorProfile(const Options& options);

}  // namespace robobaton_demo
