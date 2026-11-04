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
#include <std_msgs/msg/string.hpp>
#endif

#include "robobaton_4p_ros2_demo/icm42670_driver.h"
#include "robobaton_4p_ros2_demo/realtime_mutex.h"
#include "robobaton_4p_ros2_demo/imu_realtime.h"

#include <array>
#include <condition_variable>
#include <thread>
#include <pthread.h>
#include <sched.h>
#include <system_error>
#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdio>
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
  uint32_t buffer_capacity = 4096U;
  uint32_t sample_drop_policy = ICM42670_SAMPLE_DROP_POLICY_ALLOW_COUNTED;
  uint32_t timestamp_mode = ICM42670_TIMESTAMP_MODE_HOST;
  bool rate_metrics_enabled = false;
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
    sample_drop_policy_ = config.sample_drop_policy;

    icm42670_config_t producer_config = ICM42670_CONFIG_INIT;
    producer_config.sample_rate_hz = config.sample_rate_hz;
    producer_config.fifo_watermark_samples = config.fifo_watermark_samples;
    producer_config.buffer_capacity = config.buffer_capacity;
    producer_config.sample_drop_policy = config.sample_drop_policy;
    producer_config.timestamp_mode = config.timestamp_mode;

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
      const int stop_status = icm42670_stop(handle_);
      if (stop_status != ICM42670_STATUS_OK) {
        stop_failed_.store(true, std::memory_order_release);
      }
      CaptureFinalHealth(stop_status);
      admission_.store(false, std::memory_order_release);
      // start 部分失败仍由唯一 owner destroy 非空 C handle。
      icm42670_destroy(handle_);
      handle_ = nullptr;
      terminal_ = true;
      RecordFailure();
      FinishPublication();
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
      return !failed_.load(std::memory_order_acquire) &&
             !stop_failed_.load(std::memory_order_acquire);
    }
    if (handle_ != nullptr && start_attempted_ && running_ &&
        icm42670_is_running(handle_) == 0) {
      RecordFailure();
    }
    terminal_ = true;
    if (handle_ != nullptr && start_attempted_) {
      // Keep callback admission and the publication owner alive throughout
      // blocking STOP, including its terminal scans and ACK processing.
      const int stop_status = icm42670_stop(handle_);
      CaptureFinalHealth(stop_status);
    }
    admission_.store(false, std::memory_order_release);
    if (handle_ != nullptr) {
      icm42670_destroy(handle_);
      handle_ = nullptr;
    }
    FinishPublication();
    if (runtime_health_valid_ &&
        (runtime_health_.published_samples != admitted_samples_ ||
         published_samples_ != admitted_samples_)) {
      stop_failed_.store(true, std::memory_order_release);
      RecordFailure();
    }
    running_ = false;
    sink_ = {};
    notifier_ = {};
    return !failed_.load(std::memory_order_acquire) &&
           !stop_failed_.load(std::memory_order_acquire);
  }

  bool failed() const noexcept { return failed_.load(std::memory_order_acquire); }
  bool CheckProducerHealth() noexcept {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!terminal_ && handle_ != nullptr && start_attempted_ && running_ &&
        icm42670_is_running(handle_) == 0) {
      running_ = false;
      terminal_ = true;
      RecordFailure();
    }
    return !failed_.load(std::memory_order_acquire);
  }


 private:
  static void SampleTrampoline(const icm42670_sample_t* sample, void* user_data) noexcept {
    auto* owner = static_cast<ImuLifecycleCore*>(user_data);
    if (owner == nullptr || !owner->admission_.load(std::memory_order_acquire)) {
      return;
    }
    try {
      if (sample == nullptr || sample->struct_size != sizeof(*sample) ||
          !sample->session_generation || !sample->sample_sequence ||
          (owner->sample_generation_ && owner->sample_generation_ != sample->session_generation)) {
        throw std::runtime_error("invalid ICM sample");
      }
      owner->sample_generation_ = sample->session_generation;
      owner->last_sample_sequence_ = sample->sample_sequence;
      // 默认关闭时不读取clock、不进入metrics mutex/atomic；启用后在ROS sink前记录producer输入。
      if (owner->metrics_enabled_) {
        owner->metrics_.RecordSource(sample->sample_sequence,
                                      sample->sample_monotonic_ns,
                                      SteadyNowNs());
      }
      owner->Enqueue(*sample);
      ++owner->admitted_samples_;
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

  bool RuntimeHealthValid() noexcept {
    std::lock_guard<std::mutex> lock(mutex_);
    return runtime_health_valid_;
  }

 private:
  void CaptureFinalHealth(int stop_status) noexcept {
    runtime_health_ = ICM42670_RUNTIME_HEALTH_INIT;
    const int health_status = icm42670_get_runtime_health(handle_, &runtime_health_);
    runtime_health_valid_ = health_status == ICM42670_STATUS_OK &&
        runtime_health_.struct_size == sizeof(runtime_health_) &&
        runtime_health_.session_generation != 0U;
    const auto& h = runtime_health_;
    if (stop_status != ICM42670_STATUS_OK || !runtime_health_valid_ ||
        h.transport_complete != 1U || h.fatal_reason || h.fatal_errno ||
        h.fifo_overflow_count ||
        (sample_drop_policy_ == ICM42670_SAMPLE_DROP_POLICY_STRICT && h.invalid_payload_count) ||
        h.scan_push_failure_count || h.evidence_full_count || h.quiesce_errno ||
        h.records_consumed != h.records_acked ||
        h.consumed_scans < h.published_samples ||
        h.consumed_scans - h.published_samples != h.invalid_payload_count ||
        h.last_packet_sequence != h.consumed_scans ||
        h.tail_packet_count > h.last_packet_sequence ||
        h.timestamp_service_burst_count > h.records_consumed ||
        (sample_generation_ && sample_generation_ != h.session_generation) ||
        last_sample_sequence_ > h.last_packet_sequence) {
      stop_failed_.store(true, std::memory_order_release);
      RecordFailure();
    }
  }

  // Only bounded sample copies run on the producer; DDS runs on another owner.
  using Pending = icm42670_sample_t;
  void Enqueue(const Pending& value) {
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
        sink_(job);
        ++published_samples_;
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
  FailureNotifier notifier_;
  std::atomic<bool> admission_{false};
  std::atomic<bool> failed_{false};
  std::atomic<bool> stop_failed_{false};
  robobaton_4p_ros2_demo::PublicationRateMetrics metrics_{
      robobaton_4p_ros2_demo::SequenceWidth::k64Bit};
  icm42670_runtime_health_t runtime_health_ = ICM42670_RUNTIME_HEALTH_INIT;
  bool runtime_health_valid_ = false;
  // Each serial owner writes its count; STOP joins both before comparison.
  uint64_t admitted_samples_ = 0U;
  uint64_t published_samples_ = 0U;
  uint32_t sample_generation_ = 0U;
  uint64_t last_sample_sequence_ = 0U;
  uint32_t sample_drop_policy_ = ICM42670_SAMPLE_DROP_POLICY_ALLOW_COUNTED;
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
  config.buffer_capacity = harness->config.buffer_capacity;
  config.sample_drop_policy = harness->config.sample_drop_policy;
  config.timestamp_mode = harness->config.timestamp_mode;
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

bool ImuHarnessRuntimeHealthValid(void* opaque) {
  return static_cast<ImuHarness*>(opaque)->core.RuntimeHealthValid();
}

}  // namespace release008_test

#else

namespace robobaton_4p_ros2_demo {
namespace {
constexpr char kImuTopic[] = "/robobaton/imu/data";
constexpr char kTemperatureTopic[] = "/robobaton/imu/temperature";
constexpr char kTimingTopic[] = "/robobaton/imu/timing";

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
    timing_pub_ = node_->create_publisher<std_msgs::msg::String>(
        kTimingTopic, rclcpp::QoS(256).reliable());
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
    core_config.buffer_capacity = config_.buffer_capacity;
    core_config.sample_drop_policy = config_.sample_drop_policy;
    core_config.timestamp_mode = config_.timestamp_mode;
    core_config.rate_metrics_enabled = config_.rate_metrics_enabled;
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
    // Sink success is measured only with diagnostics; producer health is a separate population.
    char publication_count[32] = "unavailable";
    if (config_.rate_metrics_enabled) {
      const auto publications = core_.Snapshot(SteadyNowNs());
      std::snprintf(publication_count, sizeof(publication_count), "%llu",
                    static_cast<unsigned long long>(publications.publish_count));
    }
    const bool health_valid = core_.RuntimeHealthValid();
    RCLCPP_INFO(node_->get_logger(),
        "IMU_RUNTIME_HEALTH final=yes health_valid=%s stop_ok=%s session_generation=%u published_samples=%llu imu_published_samples=%s imu_publish_count_valid=%s consumed_scans=%llu records_consumed=%llu records_acked=%llu last_packet_sequence=%llu irq_wakeup_count=%llu timer_service_count=%llu fatal_reason=%u fatal_errno=%u fifo_overflow_count=%u invalid_payload_count=%u scan_push_failure_count=%u evidence_full_count=%u timestamp_service_burst_count=%u tail_packet_count=%u unknown_hardware_tail=%u quiesce_errno=%u transport_complete=%u",
        health_valid ? "yes" : "no", stop_ok ? "yes" : "no", health.session_generation,
        static_cast<unsigned long long>(health.published_samples),
        publication_count, config_.rate_metrics_enabled ? "yes" : "no",
        static_cast<unsigned long long>(health.consumed_scans),
        static_cast<unsigned long long>(health.records_consumed),
        static_cast<unsigned long long>(health.records_acked),
        static_cast<unsigned long long>(health.last_packet_sequence),
        static_cast<unsigned long long>(health.irq_wakeup_count),
        static_cast<unsigned long long>(health.timer_service_count),
        health.fatal_reason, health.fatal_errno, health.fifo_overflow_count,
        health.invalid_payload_count, health.scan_push_failure_count, health.evidence_full_count,
        health.timestamp_service_burst_count, health.tail_packet_count,
        health.unknown_hardware_tail, health.quiesce_errno, health.transport_complete);
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
    const auto output = realtime_mapper_.Map(sample);
    char json[1536];
    const auto size = robobaton_demo::FormatImuTiming(output, json, sizeof(json));
    std_msgs::msg::String timing;
    timing.data.assign(json, size);
    sensor_msgs::msg::Imu imu;
    AssignStamp(imu.header.stamp, ToRosStampParts(output.realtime_ns));
    imu.header.frame_id = config_.frame_id;
    imu.orientation_covariance[0] = -1.0;
    imu.angular_velocity.x = sample.gyro_rps[0];
    imu.angular_velocity.y = sample.gyro_rps[1];
    imu.angular_velocity.z = sample.gyro_rps[2];
    imu.linear_acceleration.x = sample.accel_mps2[0];
    imu.linear_acceleration.y = sample.accel_mps2[1];
    imu.linear_acceleration.z = sample.accel_mps2[2];
    // Every sample also carries explicit native-clock and bridge evidence.
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
      timing_pub_->publish(timing);
    } catch (...) {
      core_.RecordPublishFailure();
      throw;
    }
  }

  rclcpp::Node* node_;
  Config config_;
  ImuLifecycleCore core_;
  robobaton_demo::ImuRealtimeMapper realtime_mapper_;
  bool started_ = false;
  bool finalized_ = false;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr temperature_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr timing_pub_;
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
