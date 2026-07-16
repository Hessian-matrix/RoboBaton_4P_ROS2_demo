#pragma once

#include "robobaton_4p_ros2_demo/cam_demo_common.h"

namespace robobaton_demo {

// 功能：配置 libsc132 触发输出模式。
// 输入：options.trigger_mode，默认 software_gpio。
// 副作用：设置进程内环境变量 SC132_TRIGGER_MODE，libsc132 初始化时读取。
void ConfigureSc132TriggerMode(const Options& options);

// 功能：为特定启动组合选择兼容的 sensor profile。
// 输入：运行参数 options。
// 副作用：必要时设置进程内环境变量 SC132_SENSOR_PROFILE。
void ConfigureSc132SensorProfile(const Options& options);

}  // namespace robobaton_demo
