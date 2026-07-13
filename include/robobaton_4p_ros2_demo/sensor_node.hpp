#pragma once

#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "robobaton_4p_ros2_demo/camera_publisher.hpp"
#include "robobaton_4p_ros2_demo/imu_publisher.hpp"

namespace robobaton_4p_ros2_demo {

class SensorNode : public rclcpp::Node {
 public:
  explicit SensorNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  ~SensorNode() override;

 private:
  void StartEnabledPublishers();
  void StopPublishers();
  CameraPublisher::Config LoadCameraConfig();
  ImuPublisher::Config LoadImuConfig();

  bool enable_camera_ = true;
  bool enable_imu_ = true;
  std::unique_ptr<CameraPublisher> camera_publisher_;
  std::unique_ptr<ImuPublisher> imu_publisher_;
};

}  // namespace robobaton_4p_ros2_demo
