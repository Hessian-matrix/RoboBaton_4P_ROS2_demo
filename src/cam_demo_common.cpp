#include "robobaton_4p_ros2_demo/cam_demo_common.h"

namespace robobaton_demo {

// 功能：统计 mask 内有效物理相机数量。
// 输入：只统计 cam0..cam3 四个有效 bit，忽略更高 bit。
// 输出：有效 bit 数量，用于同步帧组 camera_count。
int CameraMaskPopCount(uint32_t camera_mask) {
  int count = 0;
  for (int camera_id = 0; camera_id < kMaxChannels; ++camera_id) {
    if ((camera_mask & (1U << static_cast<uint32_t>(camera_id))) != 0) {
      ++count;
    }
  }
  return count;
}

// 功能：判断指定物理相机是否启用。
// 输入：camera_id 越界时直接返回 false。
// 输出：true 表示当前进程会为该物理相机启动 ROS 发布 worker。
bool CameraMaskContains(uint32_t camera_mask, int camera_id) {
  if (camera_id < 0 || camera_id >= kMaxChannels) {
    return false;
  }
  return (camera_mask & (1U << static_cast<uint32_t>(camera_id))) != 0;
}

// 功能：把用户旋转角度映射到底层安装补偿角度。
// 输入：用户看到的正装画面以 rotate=0 表示；SC132 原始安装方向需要内部右旋 90 度。
// 输出：传给 libsc132 的真实旋转角度。
int InternalRotateDegrees(const Options& options) {
  // 2026-06-17 修改原因：对外隐藏 sensor 竖装原始方向；用户 rotate=0 时底层仍执行 90 度安装补偿。
  return (options.rotate_degrees + kMountRotateDegrees) % 360;
}

}  // namespace robobaton_demo
