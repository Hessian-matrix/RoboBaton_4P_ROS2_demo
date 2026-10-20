#ifdef RELEASE008_CONTRACT_TEST
#include "release008_fake_producers.hpp"
#else
#include "robobaton_4p_ros2_demo/imu_publisher.hpp"
#include "robobaton_4p_ros2_demo/cam_demo_common.h"
#include "robobaton_4p_ros2_demo/publication_rate_metrics.hpp"
#include "robobaton_4p_ros2_demo/timestamp_mapper.hpp"
#include <rclcpp/create_timer.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/temperature.hpp>
#endif

#include "robobaton_4p_ros2_demo/icm42670_driver.h"
#include "robobaton_4p_ros2_demo/realtime_mutex.h"

#include <array>
#include <condition_variable>
#include <thread>
#include <variant>
#include <pthread.h>
#include <sched.h>
#include <system_error>
#include <atomic>
#include <chrono>
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
  uint32_t sample_rate_hz = 800U;
  uint32_t fifo_watermark_samples = 1U;
  uint32_t read_mode = ICM42670_READ_MODE_SENSOR_TIMESTAMP_FIFO;
  bool rate_metrics_enabled = false;
  std::function<void(const icm42670_fsync_event_t&)> fsync_sink;
};

class ImuLifecycleCore {
 public:
  using Sink = std::function<void(const icm42670_sample_t&)>;
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
    metrics_enabled_ = config.rate_metrics_enabled;
    fsync_sink_ = config.fsync_sink;

    icm42670_config_t producer_config = ICM42670_CONFIG_INIT;
    producer_config.sample_rate_hz = config.sample_rate_hz;
    producer_config.fifo_watermark_samples = config.fifo_watermark_samples;
    producer_config.read_mode = config.read_mode;
    producer_config.reserved[ICM42670_CONFIG_FSYNC_ENABLE_INDEX] = fsync_sink_ ? 1U : 0U;

    icm42670_handle_t* candidate = nullptr;
    const int create_status = icm42670_create(&producer_config, &candidate);
    if (create_status != ICM42670_STATUS_OK || candidate == nullptr) {
      if (candidate != nullptr) {
        icm42670_destroy(candidate);
      }
      terminal_ = true;
      RecordFailure();
      return false;
    }
    handle_ = candidate;

    if (icm42670_set_callback(handle_, &ImuLifecycleCore::SampleTrampoline, this) !=
        ICM42670_STATUS_OK) {
      admission_.store(false, std::memory_order_release);
      icm42670_destroy(handle_);
      handle_ = nullptr;
      terminal_ = true;
      RecordFailure();
      return false;
    }
    if (fsync_sink_ && icm42670_set_fsync_callback(handle_, &ImuLifecycleCore::FsyncTrampoline, this) != ICM42670_STATUS_OK) {
      icm42670_destroy(handle_); handle_ = nullptr; terminal_ = true; RecordFailure(); return false;
    }

    try {
      publisher_worker_ = std::thread([this] { PublishLoop(); });
    } catch (...) {
      icm42670_destroy(handle_);
      handle_ = nullptr;
      terminal_ = true;
      RecordFailure();
      return false;
    }
    admission_.store(true, std::memory_order_release);
    start_attempted_ = true;
    const int start_status = icm42670_start(handle_);
    if (start_status != ICM42670_STATUS_OK) {
      admission_.store(false, std::memory_order_release);
      const int stop_status = icm42670_stop(handle_);
      if (stop_status != ICM42670_STATUS_OK) {
        stop_failed_.store(true, std::memory_order_release);
      }
      // start 部分失败仍由唯一 owner destroy 非空 C handle。
      icm42670_destroy(handle_);
      handle_ = nullptr;
      terminal_ = true;
      RecordFailure();
      FinishPublication();
      final_max_uncertainty_us_ = sample_max_uncertainty_us_;
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
    if (handle_ != nullptr && start_attempted_ && running_ &&
        icm42670_is_running(handle_) == 0) {
      admission_.store(false, std::memory_order_release);
      running_ = false;
      terminal_ = true;
      RecordFailure();
    }
    terminal_ = true;
    admission_.store(false, std::memory_order_release);
    int stop_status = ICM42670_STATUS_OK;
    int health_status = ICM42670_STATUS_INVALID_STATE;
    if (handle_ != nullptr && start_attempted_) {
      // context 保持到 blocking stop 返回；即使 stop 失败，仍执行唯一 destroy finalizer。
      stop_status = icm42670_stop(handle_);
      if (stop_status == ICM42670_STATUS_OK) {
        runtime_health_ = ICM42670_RUNTIME_HEALTH_INIT;
        health_status = icm42670_get_runtime_health(handle_, &runtime_health_);
        runtime_health_valid_ = health_status == ICM42670_STATUS_OK &&
                                runtime_health_.session_generation != 0U;
      }
      if (stop_status != ICM42670_STATUS_OK || !runtime_health_valid_) {
        stop_failed_.store(true, std::memory_order_release);
        RecordFailure();
      }
    }
    if (handle_ != nullptr) {
      icm42670_destroy(handle_);
      handle_ = nullptr;
    }
    FinishPublication();
    final_max_uncertainty_us_ = sample_max_uncertainty_us_;
    running_ = false;
    sink_ = {};
    notifier_ = {};
    return stop_status == ICM42670_STATUS_OK && health_status == ICM42670_STATUS_OK &&
           runtime_health_valid_;
  }

  bool failed() const noexcept { return failed_.load(std::memory_order_acquire); }
  bool CheckProducerHealth() noexcept {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!terminal_ && handle_ != nullptr && start_attempted_ && running_ &&
        icm42670_is_running(handle_) == 0) {
      admission_.store(false, std::memory_order_release);
      running_ = false;
      terminal_ = true;
      RecordFailure();
    }
    return !failed_.load(std::memory_order_acquire);
  }


 private:
  static void FsyncTrampoline(const icm42670_fsync_event_t* event, void* user) noexcept {
    auto* owner = static_cast<ImuLifecycleCore*>(user);
    if (!owner->admission_.load(std::memory_order_acquire)) return;
    try {
      if (!event || event->struct_size != sizeof(*event)) throw std::runtime_error("invalid FSYNC event");
      owner->Enqueue(*event);
    } catch (...) { owner->admission_.store(false); owner->RecordFailure(); }
  }
  static void SampleTrampoline(const icm42670_sample_t* sample, void* user_data) noexcept {
    auto* owner = static_cast<ImuLifecycleCore*>(user_data);
    if (owner == nullptr || !owner->admission_.load(std::memory_order_acquire)) {
      return;
    }
    try {
      if (sample == nullptr || sample->struct_size != sizeof(*sample)) {
        throw std::runtime_error("invalid ICM sample");
      }
      if (sample->timestamp_uncertainty_us > owner->sample_max_uncertainty_us_)
        owner->sample_max_uncertainty_us_ = sample->timestamp_uncertainty_us;
      // 默认关闭时不读取clock、不进入metrics mutex/atomic；启用后在ROS sink前记录producer输入。
      if (owner->metrics_enabled_) {
        owner->metrics_.RecordSource(sample->sample_sequence,
                                      sample->sample_timestamp_ns,
                                      SteadyNowNs());
      }
      owner->Enqueue(*sample);
    } catch (...) {
      owner->admission_.store(false, std::memory_order_release);
      owner->RecordFailure();
    }
  }

  // callback和ROS publisher统一使用steady clock纳秒，避免wall/ROS时间调整污染区间与耗时。
  static uint64_t SteadyNowNs() noexcept {
    return static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
                                     std::chrono::steady_clock::now().time_since_epoch())
                                     .count());
  }

 public:
  // ROS层只提交发布结果和读取按值snapshot，不获得C handle或metrics所有权。
  void RecordPublishSuccess(uint64_t latency_ns) noexcept {
    if (metrics_enabled_) {
      metrics_.RecordPublishSuccess(latency_ns);
    }
  }
  // 主/辅助ROS publish异常统一计入同一门禁，并由callback继续触发既有fail-closed。
  void RecordPublishFailure() noexcept {
    if (metrics_enabled_) {
      metrics_.RecordPublishFailure();
    }
  }

  // Temperature辅助topic独立计数，禁止把主IMU发布频率翻倍。
  void RecordAuxiliaryPublishSuccess() noexcept {
    if (metrics_enabled_) {
      metrics_.RecordAuxiliaryPublishSuccess();
    }
  }

  robobaton_4p_ros2_demo::PublicationRateSnapshot Snapshot(uint64_t monotonic_ns) const noexcept {
    return metrics_.Snapshot(monotonic_ns);
  }

  icm42670_runtime_health_t RuntimeHealth() noexcept {
    std::lock_guard<std::mutex> lock(mutex_);
    return runtime_health_;
  }

  uint32_t MaxUncertaintyUs() noexcept {
    std::lock_guard<std::mutex> lock(mutex_);
    return final_max_uncertainty_us_;
  }

 private:
  // The producer copies into bounded storage only; DDS and association sinks
  // run on a separate owner without holding the queue lock. Exhaustion is fatal.
  using Pending = std::variant<icm42670_sample_t, icm42670_fsync_event_t>;
  template <typename Value>
  void Enqueue(const Value& value) {
    {
      std::lock_guard<robobaton_demo::RealtimeMutex> lock(pending_mutex_);
      if (pending_closed_ || pending_count_ == pending_.size())
        throw std::runtime_error("IMU publication queue exhausted");
      pending_[(pending_head_ + pending_count_) % pending_.size()] = value;
      ++pending_count_;
    }
    pending_ready_.notify_one();
  }

  void PublishLoop() noexcept {
    try {
#ifndef RELEASE008_CONTRACT_TEST
      sched_param scheduling{};
      scheduling.sched_priority = 3;
      const int result = pthread_setschedparam(pthread_self(), SCHED_RR, &scheduling);
      if (result) throw std::system_error(result, std::generic_category(), "schedule IMU publication");
#endif
      for (;;) {
        Pending job;
        {
          std::unique_lock<robobaton_demo::RealtimeMutex> lock(pending_mutex_);
          pending_ready_.wait(lock, [this] { return pending_closed_ || pending_count_; });
          if (!pending_count_) return;
          job = pending_[pending_head_];
          pending_head_ = (pending_head_ + 1U) % pending_.size();
          --pending_count_;
        }
        if (const auto* sample = std::get_if<icm42670_sample_t>(&job)) sink_(*sample);
        else fsync_sink_(std::get<icm42670_fsync_event_t>(job));
      }
    } catch (...) {
      admission_.store(false, std::memory_order_release);
      RecordFailure();
    }
  }

  void FinishPublication() noexcept {
    {
      std::lock_guard<robobaton_demo::RealtimeMutex> lock(pending_mutex_);
      pending_closed_ = true;
    }
    pending_ready_.notify_one();
    if (publisher_worker_.joinable()) publisher_worker_.join();
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
  robobaton_demo::RealtimeMutex pending_mutex_;
  robobaton_demo::RealtimeCondition pending_ready_;
  std::array<Pending, 512> pending_{};
  std::thread publisher_worker_;
  std::size_t pending_head_ = 0U, pending_count_ = 0U;
  bool pending_closed_ = false;
  icm42670_handle_t* handle_ = nullptr;
  Sink sink_;
  std::function<void(const icm42670_fsync_event_t&)> fsync_sink_;
  FailureNotifier notifier_;
  std::atomic<bool> admission_{false};
  std::atomic<bool> failed_{false};
  std::atomic<bool> stop_failed_{false};
  robobaton_4p_ros2_demo::PublicationRateMetrics metrics_{
      robobaton_4p_ros2_demo::SequenceWidth::k64Bit};
  icm42670_runtime_health_t runtime_health_ = ICM42670_RUNTIME_HEALTH_INIT;
  bool runtime_health_valid_ = false;
  // Only the serial acquisition callback writes the running maximum. Finalize
  // after stop/destroy joins that owner; readers only see the locked final value.
  uint32_t sample_max_uncertainty_us_ = 0U;
  uint32_t final_max_uncertainty_us_ = 0U;
  bool metrics_enabled_ = false;
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
  std::atomic<bool> throw_in_auxiliary_publish{false};
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
  config.rate_metrics_enabled = harness->config.rate_metrics_enabled;
  return harness->core.Start(
      config,
      [harness](const icm42670_sample_t&) {
        if (harness->config.before_publish) harness->config.before_publish();
        // host harness按production顺序模拟主/辅助发布，验证任一异常都会计数后rethrow。
        try {
          if (harness->throw_in_callback.exchange(false)) {
            throw std::runtime_error("injected IMU publish failure");
          }
          harness->core.RecordPublishSuccess(15U);
          if (harness->throw_in_auxiliary_publish.exchange(false)) {
            throw std::runtime_error("injected temperature publish failure");
          }
          harness->core.RecordAuxiliaryPublishSuccess();
          harness->sample_count.fetch_add(1U);
        } catch (...) {
          harness->core.RecordPublishFailure();
          throw;
        }
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

void ImuHarnessThrowInAuxiliaryPublish(void* opaque) {
  static_cast<ImuHarness*>(opaque)->throw_in_auxiliary_publish.store(true);
}
bool ImuHarnessCheckProducerHealth(void* opaque) {
  return static_cast<ImuHarness*>(opaque)->core.CheckProducerHealth();
}

robobaton_4p_ros2_demo::PublicationRateSnapshot ImuHarnessSnapshot(
    void* opaque, uint64_t monotonic_ns) {
  return static_cast<ImuHarness*>(opaque)->core.Snapshot(monotonic_ns);
}

icm42670_runtime_health_t ImuHarnessRuntimeHealth(void* opaque) {
  return static_cast<ImuHarness*>(opaque)->core.RuntimeHealth();
}

uint32_t ImuHarnessMaxUncertaintyUs(void* opaque) {
  return static_cast<ImuHarness*>(opaque)->core.MaxUncertaintyUs();
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
  Impl(rclcpp::Node* node, Config config)
      : node_(node), config_(std::move(config)) {
    if (node_ == nullptr || (config_.rate_metrics_enabled && config_.rate_log_period_ms == 0U)) {
      throw std::invalid_argument("invalid IMU publisher configuration");
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
    core_config.read_mode = ICM42670_READ_MODE_SENSOR_TIMESTAMP_FIFO;
    core_config.rate_metrics_enabled = config_.rate_metrics_enabled;
    core_config.fsync_sink = config_.fsync_sink;
    if (!core_.Start(
            core_config,
            [this](const icm42670_sample_t& sample) { PublishSample(sample); },
            [this] {
              robobaton_4p_ros2_demo::RecordProcessFailure();
              RCLCPP_ERROR(node_->get_logger(), "ICM producer or callback failure; shutting down");
              rclcpp::shutdown();
            })) {
      throw std::runtime_error("ICM publisher start failed or restart was rejected");
    }
    started_ = true;
    RCLCPP_INFO(node_->get_logger(), "Started ICM IMU publisher on %s", kImuTopic);
    if (config_.rate_metrics_enabled) {
      previous_snapshot_ = core_.Snapshot(SteadyNowNs());
      const auto period = std::chrono::milliseconds(config_.rate_log_period_ms);
      rate_timer_ = rclcpp::create_wall_timer(
          period, [this] { LogRateMetrics(); }, nullptr, node_->get_node_base_interface().get(),
          node_->get_node_timers_interface().get());
    }
    // Producer failure detection remains active when rate diagnostics are disabled.
    health_timer_ = rclcpp::create_wall_timer(
        std::chrono::milliseconds(100U), [this] { (void)core_.CheckProducerHealth(); },
        nullptr, node_->get_node_base_interface().get(),
        node_->get_node_timers_interface().get());
  }

  void Stop() noexcept {
    if (finalized_) return;
    health_timer_.reset();
    rate_timer_.reset();
    if (!started_ && !core_.failed()) {
      return;
    }
    finalized_ = true;
    const bool stop_ok = core_.Stop();
    const auto health = core_.RuntimeHealth();
    const auto publications = core_.Snapshot(SteadyNowNs());
    const bool health_valid = stop_ok && health.struct_size == sizeof(health) &&
                              health.session_generation != 0U;
    RCLCPP_INFO(node_->get_logger(),
        "IMU_RUNTIME_HEALTH final=yes health_valid=%s session_generation=%u published_samples=%llu imu_published_samples=%llu invalid_packet_drop_count=%u gpio_event_gap_count=%u fifo_overflow_count=%u mapper_failure_count=%u uncertainty_over_200_drop_count=%u max_uncertainty_us=%u max_consecutive_timing_drop_count=%u fsync_captured=%u fsync_dropped=%u",
        health_valid ? "yes" : "no", health.session_generation,
        static_cast<unsigned long long>(health.published_samples),
        static_cast<unsigned long long>(publications.publish_count),
        health.invalid_packet_drop_count, health.gpio_event_gap_count, health.fifo_overflow_count,
        health.mapper_failure_count, health.uncertainty_over_200_drop_count, core_.MaxUncertaintyUs(),
        health.max_consecutive_timing_drop_count,
        health.reserved[ICM42670_HEALTH_FSYNC_CAPTURED_INDEX],
        health.reserved[ICM42670_HEALTH_FSYNC_DROPPED_INDEX]);
    if (!stop_ok) {
      robobaton_4p_ros2_demo::RecordProcessFailure();
      RCLCPP_ERROR(node_->get_logger(), "ICM stop failed; handle was finalized and restart is forbidden");
    }
    if (core_.failed()) {
      robobaton_4p_ros2_demo::RecordProcessFailure();
    }
    started_ = false;
  }

 private:
  // rate timer只读取metrics snapshot并输出单行JSON；不触碰ICM handle或发布状态。
  void LogRateMetrics() {
    const auto current = core_.Snapshot(SteadyNowNs());
    if (current.interval_start_monotonic_ns == 0U) {
      return;
    }
    const auto delta = robobaton_4p_ros2_demo::PublicationRateMetrics::Delta(
        previous_snapshot_, current);
    previous_snapshot_ = current;
    const uint64_t interval_start = delta.interval_start_monotonic_ns == 0U
                                        ? current.interval_start_monotonic_ns
                                        : delta.interval_start_monotonic_ns;
    RCLCPP_INFO(
        node_->get_logger(),
        "ROB2_RATE {\"schema\":\"robobaton-rate-v1\",\"run_id\":\"%s\",\"kind\":\"imu\",\"interval_start_monotonic_ns\":%llu,\"interval_end_monotonic_ns\":%llu,\"source\":%llu,\"published\":%llu,\"auxiliary_published\":%llu,\"dropped\":%llu,\"publish_failures\":%llu,\"timestamp_duplicates\":%llu,\"timestamp_regressions\":%llu,\"publish_latency_count\":%llu,\"publish_latency_sum_ns\":%llu,\"publish_latency_max_ns\":%llu}",
        config_.rate_run_id.c_str(), static_cast<unsigned long long>(interval_start),
        static_cast<unsigned long long>(delta.interval_end_monotonic_ns),
        static_cast<unsigned long long>(delta.source_count),
        static_cast<unsigned long long>(delta.publish_count),
        static_cast<unsigned long long>(delta.auxiliary_publish_count),
        static_cast<unsigned long long>(delta.drop_count),
        static_cast<unsigned long long>(delta.publish_failure_count),
        static_cast<unsigned long long>(delta.timestamp_duplicate_count),
        static_cast<unsigned long long>(delta.timestamp_regression_count),
        static_cast<unsigned long long>(delta.publish_latency_count),
        static_cast<unsigned long long>(delta.publish_latency_sum_ns),
        static_cast<unsigned long long>(delta.publish_latency_max_ns));
  }

  static uint64_t SteadyNowNs() noexcept {
    return static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
                                     std::chrono::steady_clock::now().time_since_epoch())
                                     .count());
  }

  // Publication runs off the acquisition owner; DDS stalls cannot stop FIFO draining.
  void PublishSample(const icm42670_sample_t& sample) {
    sensor_msgs::msg::Imu imu;
    AssignStamp(imu.header.stamp, ToRosStampParts(sample.sample_timestamp_ns));
    imu.header.frame_id = config_.frame_id;
    imu.orientation_covariance[0] = -1.0;
    imu.angular_velocity.x = sample.gyro_rps[0];
    imu.angular_velocity.y = sample.gyro_rps[1];
    imu.angular_velocity.z = sample.gyro_rps[2];
    imu.linear_acceleration.x = sample.accel_mps2[0];
    imu.linear_acceleration.y = sample.accel_mps2[1];
    imu.linear_acceleration.z = sample.accel_mps2[2];
    // 主IMU或辅助Temperature任一publish异常都先计数再rethrow，保留callback既有fail-closed语义。
    try {
      if (config_.rate_metrics_enabled) {
        const uint64_t publish_start_ns = SteadyNowNs();
        imu_pub_->publish(imu);
        core_.RecordPublishSuccess(SteadyNowNs() - publish_start_ns);
      } else {
        imu_pub_->publish(imu);
      }
      if (temperature_pub_) {
        sensor_msgs::msg::Temperature temperature;
        temperature.header = imu.header;
        temperature.temperature = sample.temperature_c;
        temperature_pub_->publish(temperature);
        if (config_.rate_metrics_enabled) {
          core_.RecordAuxiliaryPublishSuccess();
        }
      }
    } catch (...) {
      core_.RecordPublishFailure();
      throw;
    }
  }

  rclcpp::Node* node_;
  Config config_;
  ImuLifecycleCore core_;
  bool started_ = false;
  bool finalized_ = false;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr temperature_pub_;
  rclcpp::TimerBase::SharedPtr rate_timer_;
  rclcpp::TimerBase::SharedPtr health_timer_;
  robobaton_4p_ros2_demo::PublicationRateSnapshot previous_snapshot_{};
};

ImuPublisher::ImuPublisher(rclcpp::Node* node, Config config)
    : impl_(std::make_unique<Impl>(node, std::move(config))) {}
ImuPublisher::~ImuPublisher() = default;
void ImuPublisher::Start() { impl_->Start(); }
void ImuPublisher::Stop() { impl_->Stop(); }

}  // namespace robobaton_4p_ros2_demo

#endif
