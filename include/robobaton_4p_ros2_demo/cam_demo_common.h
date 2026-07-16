#pragma once

#include <cstdint>
#include <string>

namespace robobaton_demo {

// Demo 支持的最大相机路数。
constexpr int kMaxChannels = 4;
constexpr int kSensorInputWidth = 1088;
constexpr int kSensorInputHeight = 1280;
constexpr int kDefaultFps = 60;
constexpr int kDefaultRotateDegrees = 0;
constexpr int kMountRotateDegrees = 90;
constexpr uint32_t kDefaultCameraMask = (1U << kMaxChannels) - 1U;
constexpr uint64_t kDefaultFrameSetMaxSkewNs = 1000000ULL;
constexpr uint32_t kDefaultFrameSetTimeoutMs = 100;
constexpr const char* kDefaultSc132TriggerMode = "software_gpio";

// ROS 相机发布路径使用的运行参数。
// 输入来源：ROS 参数或默认值。
// 输出用途：相机初始化、帧组同步和触发模式配置。
struct Options {
  uint32_t camera_mask = kDefaultCameraMask;
  int fps = kDefaultFps;
  int rotate_degrees = kDefaultRotateDegrees;
  uint64_t frame_set_max_skew_ns = kDefaultFrameSetMaxSkewNs;
  uint32_t frame_set_timeout_ms = kDefaultFrameSetTimeoutMs;
  std::string trigger_mode = kDefaultSc132TriggerMode;
};

// 功能：统计相机 mask 内的启用物理相机数量。
// 输入：camera_mask bit0..bit3 对应 cam0..cam3。
// 输出：启用 bit 数量，超出 bit 会被忽略。
int CameraMaskPopCount(uint32_t camera_mask);

// 功能：判断物理相机是否在当前 mask 内启用。
// 输入：camera_id 为 0..3。
// 输出：true 表示该物理相机应启动 ROS 发布 worker。
bool CameraMaskContains(uint32_t camera_mask, int camera_id);

// 功能：把对外旋转角度转换为底层相机输出旋转角度。
// 输入：options.rotate_degrees 为用户理解的正装画面相对旋转。
// 输出：传给 libsc132 的真实旋转角度。
int InternalRotateDegrees(const Options& options);

}  // namespace robobaton_demo
