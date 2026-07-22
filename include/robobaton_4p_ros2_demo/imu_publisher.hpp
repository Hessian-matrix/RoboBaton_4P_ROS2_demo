#pragma once

#include <cstdint>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

namespace robobaton_4p_ros2_demo {

class ImuPublisher {
 public:
  struct Config {
    uint32_t sample_rate_hz = 1000U;
    uint32_t fifo_watermark_samples = 8U;
    bool direct_read = false;
    std::string frame_id = "robobaton_imu_link";
    bool publish_temperature = true;
    bool rate_metrics_enabled = false;
    uint32_t rate_log_period_ms = 1000U;
    std::string rate_run_id;
  };

  // 发布器独占一个 ICM C handle，关闭后禁止同进程重启。
  ImuPublisher(rclcpp::Node* node, Config config);
  ~ImuPublisher();

  ImuPublisher(const ImuPublisher&) = delete;
  ImuPublisher& operator=(const ImuPublisher&) = delete;

  void Start();
  void Stop();

 private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace robobaton_4p_ros2_demo
