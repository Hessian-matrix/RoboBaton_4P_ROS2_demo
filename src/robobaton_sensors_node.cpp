#include "robobaton_4p_ros2_demo/sensor_node.hpp"

#include <atomic>
#include <cstddef>
#include <cstdio>
#include <cstdint>
#include <exception>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>

#include "robobaton_4p_ros2_demo/cam_demo_common.h"

namespace robobaton_4p_ros2_demo {

namespace {
std::atomic<bool> g_process_failed{false};
}

void RecordProcessFailure() noexcept {
  g_process_failed.store(true, std::memory_order_release);
}

bool ProcessFailed() noexcept {
  return g_process_failed.load(std::memory_order_acquire);
}

namespace {

int DeclareInt(rclcpp::Node* node, const std::string& name, int default_value) {
  return node->declare_parameter<int>(name, default_value);
}

bool DeclareBool(rclcpp::Node* node, const std::string& name, bool default_value) {
  return node->declare_parameter<bool>(name, default_value);
}

std::string DeclareString(rclcpp::Node* node, const std::string& name,
                          const std::string& default_value) {
  return node->declare_parameter<std::string>(name, default_value);
}

uint64_t DeclareUint64(rclcpp::Node* node, const std::string& name, uint64_t default_value) {
  const int64_t value =
      node->declare_parameter<int64_t>(name, static_cast<int64_t>(default_value));
  if (value < 0) {
    throw std::invalid_argument(name + " must be non-negative");
  }
  return static_cast<uint64_t>(value);
}

uint32_t DeclareUint32(rclcpp::Node* node, const std::string& name, uint32_t default_value) {
  const int64_t value =
      node->declare_parameter<int64_t>(name, static_cast<int64_t>(default_value));
  if (value < 0 || value > 0xffffffffLL) {
    throw std::invalid_argument(name + " must be a uint32 value");
  }
  return static_cast<uint32_t>(value);
}

std::size_t DeclareSize(rclcpp::Node* node, const std::string& name,
                        std::size_t default_value) {
  const int64_t value =
      node->declare_parameter<int64_t>(name, static_cast<int64_t>(default_value));
  if (value <= 0) {
    throw std::invalid_argument(name + " must be positive");
  }
  return static_cast<std::size_t>(value);
}

CameraPublisher::QueuePolicy ParseQueuePolicy(const std::string& value) {
  if (value == "block") {
    return CameraPublisher::QueuePolicy::kBlock;
  }
  if (value == "drop_newest") {
    return CameraPublisher::QueuePolicy::kDropNewest;
  }
  throw std::invalid_argument("camera.queue_policy must be block or drop_newest");
}

bool ParseDirectRead(const std::string& value) {
  if (value == "fifo") {
    return false;
  }
  if (value == "direct") {
    return true;
  }
  throw std::invalid_argument("imu.read_mode must be fifo or direct");
}

}  // namespace

SensorNode::SensorNode(const rclcpp::NodeOptions& options)
    : rclcpp::Node("robobaton_sensors_node", options) {
  enable_camera_ = DeclareBool(this, "enable_camera", true);
  enable_imu_ = DeclareBool(this, "enable_imu", true);
  const std::string timestamp_source = DeclareString(this, "timestamp_source", "ros_now");
  if (timestamp_source != "ros_now") {
    throw std::invalid_argument("timestamp_source only supports ros_now");
  }
  StartEnabledPublishers();
}

SensorNode::~SensorNode() { StopPublishers(); }

void SensorNode::StartEnabledPublishers() {
  if (enable_imu_) {
    imu_publisher_ = std::make_unique<ImuPublisher>(this, LoadImuConfig());
    imu_publisher_->Start();
  }
  if (enable_camera_) {
    camera_publisher_ = std::make_unique<CameraPublisher>(this, LoadCameraConfig());
    camera_publisher_->Start();
  }
}

void SensorNode::StopPublishers() {
  if (camera_publisher_) {
    camera_publisher_->Stop();
    camera_publisher_.reset();
  }
  if (imu_publisher_) {
    imu_publisher_->Stop();
    imu_publisher_.reset();
  }
}

CameraPublisher::Config SensorNode::LoadCameraConfig() {
  CameraPublisher::Config config;
  auto& options = config.options;
  options.camera_mask =
      DeclareUint32(this, "camera.camera_mask", robobaton_demo::kDefaultCameraMask);
  options.fps = DeclareInt(this, "camera.fps", robobaton_demo::kDefaultFps);
  options.rotate_degrees =
      DeclareInt(this, "camera.rotate_degrees", robobaton_demo::kDefaultRotateDegrees);
  options.frame_set_max_skew_ns = DeclareUint64(
      this, "camera.frame_set_max_skew_ns", robobaton_demo::kDefaultFrameSetMaxSkewNs);
  options.frame_set_timeout_ms = DeclareUint32(
      this, "camera.frame_set_timeout_ms", robobaton_demo::kDefaultFrameSetTimeoutMs);
  options.trigger_mode =
      DeclareString(this, "camera.trigger_mode", robobaton_demo::kDefaultSc132TriggerMode);
  config.queue_capacity = DeclareSize(this, "camera.queue_capacity", 4U);
  config.queue_policy =
      ParseQueuePolicy(DeclareString(this, "camera.queue_policy", "block"));
  config.publish_camera_info = DeclareBool(this, "camera.publish_camera_info", true);
  config.image_encoding = DeclareString(this, "camera.image_encoding", "nv12");
  config.frame_id_prefix =
      DeclareString(this, "camera.frame_id_prefix", "robobaton_cam");
  return config;
}

ImuPublisher::Config SensorNode::LoadImuConfig() {
  ImuPublisher::Config config;
  config.sample_rate_hz = DeclareUint32(this, "imu.sample_rate_hz", 1000U);
  config.direct_read = ParseDirectRead(DeclareString(this, "imu.read_mode", "fifo"));
  config.fifo_watermark_samples =
      DeclareUint32(this, "imu.fifo_watermark_samples", 8U);
  config.frame_id = DeclareString(this, "imu.frame_id", "robobaton_imu_link");
  config.publish_temperature = DeclareBool(this, "imu.publish_temperature", true);
  return config;
}

}  // namespace robobaton_4p_ros2_demo

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  int exit_code = 0;
  try {
    auto node = std::make_shared<robobaton_4p_ros2_demo::SensorNode>();
    rclcpp::spin(node);
    node.reset();
  } catch (const std::exception& error) {
    std::fprintf(stderr, "fatal: %s\n", error.what());
    exit_code = 1;
  }
  rclcpp::shutdown();
  if (robobaton_4p_ros2_demo::ProcessFailed()) {
    exit_code = 1;
  }
  return exit_code;
}
