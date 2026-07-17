#ifdef RELEASE008_CONTRACT_TEST
#include "release008_fake_producers.hpp"
#else
#include "robobaton_4p_ros2_demo/imu_publisher.hpp"
#include "robobaton_4p_ros2_demo/cam_demo_common.h"
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/temperature.hpp>
#endif

#include "robobaton_4p_ros2_demo/icm42688_driver.h"

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <utility>

#ifndef RELEASE008_CONTRACT_TEST
namespace robobaton_4p_ros2_demo {
void RecordProcessFailure() noexcept;
}
#endif

namespace {

struct ImuCoreConfig {
  uint32_t sample_rate_hz = 1000U;
  uint32_t fifo_watermark_samples = 8U;
  uint32_t read_mode = ICM42688_READ_MODE_FIFO;
};

class ImuLifecycleCore {
 public:
  using Sink = std::function<void(const icm42688_sample_t&)>;
  using FailureNotifier = std::function<void()>;

  ImuLifecycleCore() = default;
  ~ImuLifecycleCore() { (void)Stop(); }

  ImuLifecycleCore(const ImuLifecycleCore&) = delete;
  ImuLifecycleCore& operator=(const ImuLifecycleCore&) = delete;

  bool Start(const ImuCoreConfig& config, Sink sink, FailureNotifier notifier) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (start_claimed_ || terminal_) {
      return false;
    }
    start_claimed_ = true;
    sink_ = std::move(sink);
    notifier_ = std::move(notifier);

    icm42688_config_t producer_config = ICM42688_CONFIG_INIT;
    producer_config.sample_rate_hz = config.sample_rate_hz;
    producer_config.fifo_watermark_samples = config.fifo_watermark_samples;
    producer_config.read_mode = config.read_mode;

    icm42688_handle_t* candidate = nullptr;
    const int create_status = icm42688_create(&producer_config, &candidate);
    if (create_status != ICM42688_STATUS_OK || candidate == nullptr) {
      if (candidate != nullptr) {
        icm42688_destroy(candidate);
      }
      terminal_ = true;
      RecordFailure();
      return false;
    }
    handle_ = candidate;

    if (icm42688_set_callback(handle_, &ImuLifecycleCore::SampleTrampoline, this) !=
        ICM42688_STATUS_OK) {
      admission_.store(false, std::memory_order_release);
      icm42688_destroy(handle_);
      handle_ = nullptr;
      terminal_ = true;
      RecordFailure();
      return false;
    }

    admission_.store(true, std::memory_order_release);
    start_attempted_ = true;
    const int start_status = icm42688_start(handle_);
    if (start_status != ICM42688_STATUS_OK) {
      admission_.store(false, std::memory_order_release);
      const int stop_status = icm42688_stop(handle_);
      if (stop_status != ICM42688_STATUS_OK) {
        stop_failed_.store(true, std::memory_order_release);
      }
      // start 部分失败仍由唯一 owner destroy 非空 C handle。
      icm42688_destroy(handle_);
      handle_ = nullptr;
      terminal_ = true;
      RecordFailure();
      sink_ = {};
      notifier_ = {};
      return false;
    }
    running_ = true;
    return true;
  }

  bool Stop() noexcept {
    std::lock_guard<std::mutex> lock(mutex_);
    if (terminal_ && handle_ == nullptr) {
      return !stop_failed_.load(std::memory_order_acquire);
    }
    terminal_ = true;
    admission_.store(false, std::memory_order_release);
    int stop_status = ICM42688_STATUS_OK;
    if (handle_ != nullptr && start_attempted_) {
      // context 保持到 blocking stop 返回；即使 stop 失败，仍执行唯一 destroy finalizer。
      stop_status = icm42688_stop(handle_);
      if (stop_status != ICM42688_STATUS_OK) {
        stop_failed_.store(true, std::memory_order_release);
        RecordFailure();
      }
    }
    if (handle_ != nullptr) {
      icm42688_destroy(handle_);
      handle_ = nullptr;
    }
    running_ = false;
    sink_ = {};
    notifier_ = {};
    return stop_status == ICM42688_STATUS_OK;
  }

  bool failed() const noexcept { return failed_.load(std::memory_order_acquire); }

 private:
  static void SampleTrampoline(const icm42688_sample_t* sample, void* user_data) noexcept {
    auto* owner = static_cast<ImuLifecycleCore*>(user_data);
    if (owner == nullptr || !owner->admission_.load(std::memory_order_acquire)) {
      return;
    }
    try {
      if (sample == nullptr || sample->struct_size != sizeof(*sample)) {
        throw std::runtime_error("invalid ICM sample");
      }
      owner->sink_(*sample);
    } catch (...) {
      owner->admission_.store(false, std::memory_order_release);
      owner->RecordFailure();
    }
  }

  void RecordFailure() noexcept {
    const bool first = !failed_.exchange(true, std::memory_order_acq_rel);
    if (first && notifier_) {
      try {
        notifier_();
      } catch (...) {
      }
    }
  }

  std::mutex mutex_;
  icm42688_handle_t* handle_ = nullptr;
  Sink sink_;
  FailureNotifier notifier_;
  std::atomic<bool> admission_{false};
  std::atomic<bool> failed_{false};
  std::atomic<bool> stop_failed_{false};
  bool start_claimed_ = false;
  bool start_attempted_ = false;
  bool running_ = false;
  bool terminal_ = false;
};

}  // namespace

#ifdef RELEASE008_CONTRACT_TEST

namespace release008_test {

struct ImuHarness {
  ImuLifecycleCore core;
  ImuHarnessConfig config;
  std::atomic<bool> throw_in_callback{false};
  std::atomic<std::size_t> sample_count{0U};
};

void* ImuHarnessCreate(const ImuHarnessConfig& config) {
  auto* harness = new ImuHarness();
  harness->config = config;
  return harness;
}

bool ImuHarnessStart(void* opaque) {
  auto* harness = static_cast<ImuHarness*>(opaque);
  ImuCoreConfig config;
  config.sample_rate_hz = harness->config.sample_rate_hz;
  config.fifo_watermark_samples = harness->config.fifo_watermark_samples;
  config.read_mode = harness->config.read_mode;
  return harness->core.Start(
      config,
      [harness](const icm42688_sample_t&) {
        if (harness->throw_in_callback.exchange(false)) {
          throw std::runtime_error("injected ICM callback exception");
        }
        harness->sample_count.fetch_add(1U);
      },
      [] {});
}

bool ImuHarnessStop(void* opaque) {
  return static_cast<ImuHarness*>(opaque)->core.Stop();
}

void ImuHarnessDestroy(void* opaque) { delete static_cast<ImuHarness*>(opaque); }

void ImuHarnessThrowInCallback(void* opaque) {
  static_cast<ImuHarness*>(opaque)->throw_in_callback.store(true);
}

bool ImuHarnessFailed(void* opaque) {
  return static_cast<ImuHarness*>(opaque)->core.failed();
}

std::size_t ImuHarnessSampleCount(void* opaque) {
  return static_cast<ImuHarness*>(opaque)->sample_count.load();
}

}  // namespace release008_test

#else

namespace robobaton_4p_ros2_demo {
namespace {
constexpr char kImuTopic[] = "/robobaton/imu/data";
constexpr char kTemperatureTopic[] = "/robobaton/imu/temperature";
}  // namespace

class ImuPublisher::Impl {
 public:
  Impl(rclcpp::Node* node, Config config) : node_(node), config_(std::move(config)) {
    if (node_ == nullptr) {
      throw std::invalid_argument("IMU publisher requires a node");
    }
    imu_pub_ = node_->create_publisher<sensor_msgs::msg::Imu>(
        kImuTopic, rclcpp::SensorDataQoS().keep_last(100));
    if (config_.publish_temperature) {
      temperature_pub_ = node_->create_publisher<sensor_msgs::msg::Temperature>(
          kTemperatureTopic, rclcpp::SensorDataQoS().keep_last(10));
    }
  }

  ~Impl() { Stop(); }

  void Start() {
    ImuCoreConfig core_config;
    core_config.sample_rate_hz = config_.sample_rate_hz;
    core_config.fifo_watermark_samples = config_.fifo_watermark_samples;
    core_config.read_mode = config_.direct_read ? ICM42688_READ_MODE_DIRECT
                                                : ICM42688_READ_MODE_FIFO;
    if (!core_.Start(
            core_config,
            [this](const icm42688_sample_t& sample) { PublishSample(sample); },
            [this] {
              robobaton_4p_ros2_demo::RecordProcessFailure();
              RCLCPP_ERROR(node_->get_logger(), "ICM callback failure; shutting down");
              rclcpp::shutdown();
            })) {
      throw std::runtime_error("ICM publisher start failed or restart was rejected");
    }
    started_ = true;
    RCLCPP_INFO(node_->get_logger(), "Started ICM IMU publisher on %s", kImuTopic);
  }

  void Stop() noexcept {
    if (!started_ && !core_.failed()) {
      return;
    }
    if (!core_.Stop()) {
      robobaton_4p_ros2_demo::RecordProcessFailure();
      RCLCPP_ERROR(node_->get_logger(), "ICM stop failed; handle was finalized and restart is forbidden");
    }
    if (core_.failed()) {
      robobaton_4p_ros2_demo::RecordProcessFailure();
    }
    started_ = false;
  }

 private:
  void PublishSample(const icm42688_sample_t& sample) {
    sensor_msgs::msg::Imu imu;
    imu.header.stamp = node_->now();
    imu.header.frame_id = config_.frame_id;
    imu.orientation_covariance[0] = -1.0;
    imu.angular_velocity.x = sample.gyro_rps[0];
    imu.angular_velocity.y = sample.gyro_rps[1];
    imu.angular_velocity.z = sample.gyro_rps[2];
    imu.linear_acceleration.x = sample.accel_mps2[0];
    imu.linear_acceleration.y = sample.accel_mps2[1];
    imu.linear_acceleration.z = sample.accel_mps2[2];
    imu_pub_->publish(imu);
    if (temperature_pub_) {
      sensor_msgs::msg::Temperature temperature;
      temperature.header = imu.header;
      temperature.temperature = sample.temperature_c;
      temperature_pub_->publish(temperature);
    }
  }

  rclcpp::Node* node_;
  Config config_;
  ImuLifecycleCore core_;
  bool started_ = false;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr temperature_pub_;
};

ImuPublisher::ImuPublisher(rclcpp::Node* node, Config config)
    : impl_(std::make_unique<Impl>(node, std::move(config))) {}
ImuPublisher::~ImuPublisher() = default;
void ImuPublisher::Start() { impl_->Start(); }
void ImuPublisher::Stop() { impl_->Stop(); }

}  // namespace robobaton_4p_ros2_demo

#endif
