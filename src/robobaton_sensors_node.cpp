#include "robobaton_4p_ros2_demo/sensor_node.hpp"

#include <atomic>
#include <cerrno>
#include <chrono>
#include <csignal>
#include <cstddef>
#include <cstdio>
#include <cstdint>
#include <exception>
#include <memory>
#include <pthread.h>
#include <sched.h>
#include <system_error>
#include <stdexcept>
#include <string>
#include <utility>

#include "robobaton_4p_ros2_demo/cam_demo_common.h"

extern "C" {
#include "robobaton_4p_ros2_demo/icm42670_driver.h"
#include "robobaton_4p_ros2_demo/sc132camera.h"
}

#ifndef ROBOBATON_RELEASE_VERSION
#define ROBOBATON_RELEASE_VERSION "0.0.0+unknown"
#endif

namespace {
volatile sig_atomic_t g_signal_stop = 0;

void RequestSignalStop(int) { g_signal_stop = 1; }

void InstallStopSignalHandlers() {
  struct sigaction action{};
  action.sa_handler = RequestSignalStop;
  sigemptyset(&action.sa_mask);
  if (sigaction(SIGINT, &action, nullptr) != 0 ||
      sigaction(SIGTERM, &action, nullptr) != 0)
    throw std::system_error(errno, std::generic_category(), "install stop signal handlers");
}

// DDS transport owners inherit the creating thread's policy. Vendor pipeline
// workers must instead inherit normal scheduling so they cannot starve ISP.
class ScopedThreadScheduling {
 public:
  ScopedThreadScheduling(int policy, int priority) {
    int result = pthread_getschedparam(pthread_self(), &policy_, &parameter_);
    if (result) throw std::system_error(result, std::generic_category(), "read thread scheduling");
    sched_param requested{};
    requested.sched_priority = priority;
    result = pthread_setschedparam(pthread_self(), policy, &requested);
    if (result) throw std::system_error(result, std::generic_category(), "set thread scheduling");
  }
  ~ScopedThreadScheduling() {
    if (pthread_setschedparam(pthread_self(), policy_, &parameter_) != 0)
      std::terminate();
  }
  ScopedThreadScheduling(const ScopedThreadScheduling&) = delete;
  ScopedThreadScheduling& operator=(const ScopedThreadScheduling&) = delete;
 private:
  int policy_ = SCHED_OTHER;
  sched_param parameter_{};
};
}

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

void RequireSensorTimestampFifoReadMode(const std::string& value) {
  if (value == "sensor_timestamp_fifo") {
    return;
  }
  throw std::invalid_argument("imu.read_mode only supports sensor_timestamp_fifo");
}

}  // namespace

SensorNode::SensorNode(const rclcpp::NodeOptions& options)
    : rclcpp::Node("robobaton_sensors_node", options) {
  enable_camera_ = DeclareBool(this, "enable_camera", true);
  enable_imu_ = DeclareBool(this, "enable_imu", true);
  // 三个诊断参数由节点统一声明，确保Camera和IMU使用同一run ID与窗口周期。
  rate_metrics_enabled_ = LoadRateMetricsEnabled();
  rate_log_period_ms_ = LoadRateLogPeriodMs();
  rate_run_id_ = LoadRateRunId();
  fsync_enabled_ = DeclareBool(this, "imu.fsync_enabled", false);
  if (fsync_enabled_) {
    if (!enable_camera_ || !enable_imu_) throw std::invalid_argument("FSYNC requires camera and IMU");
    fsync_publisher_ = create_publisher<std_msgs::msg::String>("/robobaton/fsync", rclcpp::QoS(256).reliable());
    fsync_association_ = std::make_unique<robobaton_demo::FsyncAssociation>(
        [](const robobaton_demo::FsyncAssociationRecord& record, void* user) {
          auto* node = static_cast<SensorNode*>(user);
          char json[1024]; const size_t size = robobaton_demo::FormatFsyncAssociation(record,json,sizeof(json));
          std_msgs::msg::String message; message.data.assign(json,size); node->fsync_publisher_->publish(message);
        }, this);
  }
  StartEnabledPublishers();
}

SensorNode::~SensorNode() { StopPublishers(); }

void SensorNode::StartEnabledPublishers() {
  if (enable_camera_)
    camera_publisher_ = std::make_unique<CameraPublisher>(this, LoadCameraConfig());
  if (enable_imu_)
    imu_publisher_ = std::make_unique<ImuPublisher>(this, LoadImuConfig());
  ScopedThreadScheduling acquisition_scheduling(SCHED_OTHER, 0);
  // Establish LPWM before anchoring the measured IMU FIFO/TMST epoch.
  if (camera_publisher_) camera_publisher_->Start();
  if (imu_publisher_) imu_publisher_->Start();
}

void SensorNode::StopPublishers() {
  if (imu_publisher_) {
    imu_publisher_->Stop();
    imu_publisher_.reset();
  }
  if (camera_publisher_) {
    camera_publisher_->Stop();
    camera_publisher_.reset();
  }
  try {
    if (fsync_association_) fsync_association_->Finish();
  } catch (const std::exception& error) {
    RecordProcessFailure();
    RCLCPP_ERROR(get_logger(), "FSYNC finalization failed: %s", error.what());
  }
}

// 默认关闭内部频率指标；关闭时publisher不创建timer也不输出ROB2_RATE日志。
bool SensorNode::LoadRateMetricsEnabled() {
  return DeclareBool(this, "diagnostics.rate_metrics_enabled", false);
}

uint32_t SensorNode::LoadRateLogPeriodMs() {
  const uint32_t period_ms =
      DeclareUint32(this, "diagnostics.rate_log_period_ms", 1000U);
  if (rate_metrics_enabled_ && period_ms == 0U) {
    throw std::invalid_argument("diagnostics.rate_log_period_ms must be positive when enabled");
  }
  return period_ms;
}

std::string SensorNode::LoadRateRunId() {
  // run ID直接进入单行JSON日志；仅接受可打印安全标识字符，禁止引号、反斜杠和换行破坏证据。
  const std::string run_id = DeclareString(this, "diagnostics.rate_run_id", "");
  for (const unsigned char value : run_id) {
    const bool safe = (value >= 'a' && value <= 'z') || (value >= 'A' && value <= 'Z') ||
                      (value >= '0' && value <= '9') || value == '-' || value == '_' ||
                      value == '.' || value == ':';
    if (!safe) {
      throw std::invalid_argument(
          "diagnostics.rate_run_id accepts only ASCII alnum and -_.:");
    }
  }
  return run_id;
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
  config.publish_compressed_image = DeclareBool(this, "camera.publish_compressed_image", true);
  config.compressed_jpeg_quality = DeclareInt(this, "camera.compressed_jpeg_quality", 80);
  if (config.compressed_jpeg_quality < 1 || config.compressed_jpeg_quality > 100) {
    throw std::invalid_argument("camera.compressed_jpeg_quality must be between 1 and 100");
  }
  config.frame_id_prefix =
      DeclareString(this, "camera.frame_id_prefix", "robobaton_cam");
  config.rate_metrics_enabled = rate_metrics_enabled_;
  config.rate_log_period_ms = rate_log_period_ms_;
  config.rate_run_id = rate_run_id_;
  if (fsync_enabled_) config.fsync_group_sink = [this](const sc132_frame_set_t& set) { fsync_association_->ObserveGroup(set); };
  return config;
}

ImuPublisher::Config SensorNode::LoadImuConfig() {
  ImuPublisher::Config config;
  config.sample_rate_hz = DeclareUint32(this, "imu.sample_rate_hz", 800U);
  if (config.sample_rate_hz != 25U && config.sample_rate_hz != 50U &&
      config.sample_rate_hz != 100U && config.sample_rate_hz != 200U &&
      config.sample_rate_hz != 400U && config.sample_rate_hz != 800U &&
      config.sample_rate_hz != 1600U) {
    throw std::invalid_argument("imu.sample_rate_hz only supports 25, 50, 100, 200, 400, 800, or 1600");
  }
  RequireSensorTimestampFifoReadMode(
      DeclareString(this, "imu.read_mode", "sensor_timestamp_fifo"));
  config.fifo_watermark_samples =
      DeclareUint32(this, "imu.fifo_watermark_samples", 1U);
  if (config.fifo_watermark_samples != 1U) {
    throw std::invalid_argument("imu.fifo_watermark_samples only supports 1");
  }
  config.frame_id = DeclareString(this, "imu.frame_id", "robobaton_imu_link");
  config.publish_temperature = DeclareBool(this, "imu.publish_temperature", true);
  config.rate_metrics_enabled = rate_metrics_enabled_;
  config.rate_log_period_ms = rate_log_period_ms_;
  config.rate_run_id = rate_run_id_;
  if (fsync_enabled_) {
    if (config.sample_rate_hz < 800) throw std::invalid_argument("FSYNC requires 800 or 1600 Hz");
    config.fsync_sink = [this](const icm42670_fsync_event_t& event) { fsync_association_->ObserveEvent(event); };
  }
  return config;
}

}  // namespace robobaton_4p_ros2_demo

int main(int argc, char** argv) {
  if (argc == 2 && std::string(argv[1]) == "--version") {
    std::printf("robobaton_sensors_node %s\n", ROBOBATON_RELEASE_VERSION);
    std::printf("libicm42670 %s abi=%u.%u\n", icm42670_get_version(),
                ICM42670_ABI_VERSION_MAJOR, ICM42670_ABI_VERSION_MINOR);
    std::printf("libsc132 %s abi=%u.%u\n", sc132_get_version(),
                SC132_ABI_VERSION_MAJOR, SC132_ABI_VERSION_MINOR);
    return 0;
  }
  int exit_code = 0;
  try {
    InstallStopSignalHandlers();
    rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
    std::shared_ptr<robobaton_4p_ros2_demo::SensorNode> node;
    {
      ScopedThreadScheduling dds_scheduling(SCHED_RR, 5);
      node = std::make_shared<robobaton_4p_ros2_demo::SensorNode>();
    }
    // Signals request owner-thread cleanup while the ROS context can still
    // publish the queued tail. Only shut down the context after node cleanup.
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    while (!g_signal_stop && rclcpp::ok())
      executor.spin_once(std::chrono::milliseconds(20));
    executor.remove_node(node);
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
