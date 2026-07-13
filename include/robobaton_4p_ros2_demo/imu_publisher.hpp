#pragma once

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include "robobaton_4p_ros2_demo/icm42688_driver.h"

namespace robobaton_4p_ros2_demo {

class ImuPublisher {
 public:
  struct Config {
    icm42688_x5::DriverConfig driver_config;
    std::string frame_id = "robobaton_imu_link";
    bool publish_temperature = true;
  };

  // 2026-07-09 修改原因：IMU 发布模块独立管理驱动生命周期，便于和相机模块分别启停。
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
