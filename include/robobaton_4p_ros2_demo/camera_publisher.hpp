#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include "robobaton_4p_ros2_demo/cam_demo_common.h"

namespace robobaton_4p_ros2_demo {

class CameraPublisher {
 public:
  enum class QueuePolicy {
    kBlock,
    kDropNewest,
  };

  struct Config {
    robobaton_demo::Options options;
    std::size_t queue_capacity = 4;
    QueuePolicy queue_policy = QueuePolicy::kBlock;
    bool publish_camera_info = true;
    std::string image_encoding = "nv12";
    std::string frame_id_prefix = "robobaton_cam";
  };

  // 节点统一管理生命周期；本类只封装 X5 相机采集与 ROS 发布资源。
  CameraPublisher(rclcpp::Node* node, Config config);
  ~CameraPublisher();

  CameraPublisher(const CameraPublisher&) = delete;
  CameraPublisher& operator=(const CameraPublisher&) = delete;

  void Start();
  void Stop();

 private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace robobaton_4p_ros2_demo
