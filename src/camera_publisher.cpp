#ifdef RELEASE008_CONTRACT_TEST
#include "release008_fake_producers.hpp"
#else
#include "robobaton_4p_ros2_demo/camera_publisher.hpp"
#include "robobaton_4p_ros2_demo/cam_demo_config.h"
#include <sensor_msgs/msg/camera_info.hpp>
#include "robobaton_4p_ros2_demo/publication_rate_metrics.hpp"
#include <rclcpp/create_timer.hpp>
#include <sensor_msgs/msg/image.hpp>
#endif

#include "robobaton_4p_ros2_demo/sc132camera.h"

#include <array>
#include <chrono>
#include <atomic>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <exception>
#include <functional>
#include <limits>
#include <memory>
#include <mutex>
#include <new>
#include <stdexcept>
#include <string>
#include <system_error>
#include <thread>
#include <utility>

#ifndef RELEASE008_CONTRACT_TEST
namespace robobaton_4p_ros2_demo {
void RecordProcessFailure() noexcept;
}
#endif

namespace {

constexpr std::size_t kMaxCameras = SC132_FRAME_SET_MAX_CAMERAS;

void RecordContractEvent(const char* event) {
#if defined(RELEASE008_CONTRACT_TEST) && !defined(ROBOBATON_RATE_METRICS_TEST)
  release008_test::RecordEvent(event);
#else
  (void)event;
#endif
}

struct CameraCoreConfig {
  uint32_t camera_mask = 1U;
  uint32_t fps = 30U;
  uint32_t rotation = 90U;
  uint32_t width = SC132_NATIVE_OUTPUT_WIDTH;
  uint32_t height = SC132_NATIVE_OUTPUT_HEIGHT;
  uint32_t timeout_ms = 100U;
  uint64_t max_skew_ns = SC132_FRAME_SET_DEFAULT_MAX_SKEW_NS;
  std::size_t queue_capacity = 2U;
  bool drop_newest = false;
  bool rate_metrics_enabled = false;
};

class RetainedFrameJob {
 public:
  RetainedFrameJob() = default;
  RetainedFrameJob(sc132_frame_t* frame, const sc132_frame_info_t& info)
      : frame_(frame), info_(info) {}
  ~RetainedFrameJob() { Reset(); }

  RetainedFrameJob(const RetainedFrameJob&) = delete;
  RetainedFrameJob& operator=(const RetainedFrameJob&) = delete;

  RetainedFrameJob(RetainedFrameJob&& other) noexcept { MoveFrom(other); }
  RetainedFrameJob& operator=(RetainedFrameJob&& other) noexcept {
    if (this != &other) {
      Reset();
      MoveFrom(other);
    }
    return *this;
  }

  const sc132_frame_info_t& info() const { return info_; }
  bool owns_frame() const { return frame_ != nullptr; }

  void Reset() noexcept {
    if (frame_ != nullptr) {
      sc132_frame_release(frame_);
      frame_ = nullptr;
    }
  }

 private:
  void MoveFrom(RetainedFrameJob& other) noexcept {
    frame_ = other.frame_;
    info_ = other.info_;
    other.frame_ = nullptr;
    other.info_ = {};
  }

  sc132_frame_t* frame_ = nullptr;
  sc132_frame_info_t info_{};
};

class FrameQueue {
 public:
  FrameQueue(std::size_t capacity, bool drop_newest)
      : capacity_(capacity), drop_newest_(drop_newest) {}

  FrameQueue(const FrameQueue&) = delete;
  FrameQueue& operator=(const FrameQueue&) = delete;

  bool Push(RetainedFrameJob job) {
    std::unique_lock<std::mutex> lock(mutex_);
    if (stopped_) {
      return false;
    }
#ifdef RELEASE008_CONTRACT_TEST
    if (fail_next_allocation_) {
      fail_next_allocation_ = false;
      throw std::bad_alloc();
    }
#endif
    if (drop_newest_ && queue_.size() >= capacity_) {
      return false;
    }
    not_full_.wait(lock, [this] { return stopped_ || queue_.size() < capacity_; });
    if (stopped_) {
      return false;
    }
    queue_.push_back(std::move(job));
    lock.unlock();
    not_empty_.notify_one();
    return true;
  }

  bool Pop(RetainedFrameJob* output) {
    std::unique_lock<std::mutex> lock(mutex_);
    not_empty_.wait(lock, [this] { return stopped_ || !queue_.empty(); });
    if (stopped_ || queue_.empty()) {
      return false;
    }
    *output = std::move(queue_.front());
    queue_.pop_front();
    lock.unlock();
    not_full_.notify_one();
    return true;
  }

  void CloseAdmission() noexcept {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      stopped_ = true;
    }
    not_empty_.notify_all();
    not_full_.notify_all();
  }

  void StopAndDetach(std::deque<RetainedFrameJob>* detached) noexcept {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      stopped_ = true;
      detached->swap(queue_);
    }
    not_empty_.notify_all();
    not_full_.notify_all();
  }

#ifdef RELEASE008_CONTRACT_TEST
  void FailNextAllocation() {
    std::lock_guard<std::mutex> lock(mutex_);
    fail_next_allocation_ = true;
  }
#endif

 private:
  const std::size_t capacity_;
  const bool drop_newest_;
  std::mutex mutex_;
  std::condition_variable not_empty_;
  std::condition_variable not_full_;
  std::deque<RetainedFrameJob> queue_;
  bool stopped_ = false;
#ifdef RELEASE008_CONTRACT_TEST
  bool fail_next_allocation_ = false;
#endif
};

class CameraLifecycleCore {
 public:
  using Sink = std::function<void(const RetainedFrameJob&)>;
  using FailureNotifier = std::function<void()>;

  CameraLifecycleCore() : bridge_(new CallbackBridge()) { bridge_->owner.store(this); }
  ~CameraLifecycleCore() {
#ifdef RELEASE008_CONTRACT_TEST
    ForceJoinForTest();
    bridge_->owner.store(nullptr, std::memory_order_release);
    // production bridge 与进程同寿命；主机 harness 显式回收以支持 LeakSanitizer。
    delete bridge_;
#endif
  }

  CameraLifecycleCore(const CameraLifecycleCore&) = delete;
  CameraLifecycleCore& operator=(const CameraLifecycleCore&) = delete;

  bool Start(const CameraCoreConfig& config, Sink sink, FailureNotifier notifier) {
    std::lock_guard<std::mutex> lock(lifecycle_mutex_);
    if (start_claimed_ || terminal_) {
      return false;
    }
    start_claimed_ = true;
    config_ = config;
    sink_ = std::move(sink);
    failure_notifier_ = std::move(notifier);
    if (!ValidateConfig()) {
      RecordFailure();
      (void)CleanupLocked();
      return false;
    }

    try {
      InitializeQueues();
      bridge_->owner.store(this, std::memory_order_release);
      bridge_->accepting.store(true, std::memory_order_release);
      if (sc132_set_fps(config_.fps) != SC132_STATUS_OK ||
          sc132_set_output_rotation(config_.rotation) != SC132_STATUS_OK) {
        throw std::runtime_error("SC132 configuration failed");
      }
      StartWorkers();
      sc132_frame_set_config_t producer_config = SC132_FRAME_SET_CONFIG_INIT;
      producer_config.callback = &CameraLifecycleCore::FrameSetTrampoline;
      producer_config.user_data = bridge_;
      producer_config.camera_count = EnabledCameraCount();
      producer_config.width = config_.width;
      producer_config.height = config_.height;
      producer_config.timeout_ms = config_.timeout_ms;
      producer_config.max_skew_ns = config_.max_skew_ns;
      start_attempted_ = true;
      const int32_t status = sc132_start_frame_set(&producer_config, config_.camera_mask);
      if (status != SC132_STATUS_OK) {
        throw std::runtime_error("SC132 frame-set startup failed");
      }
      running_ = true;
      return true;
    } catch (...) {
      RecordFailure();
      (void)CleanupLocked();
      return false;
    }
  }

  bool Cleanup() noexcept {
    std::lock_guard<std::mutex> lock(lifecycle_mutex_);
    return CleanupLocked();
  }
  // ROS worker通过单一公共入口提交publish结果，core内部持有每相机metrics并校验边界。
  void RecordPublishSuccess(uint32_t camera_id, uint64_t latency_ns) noexcept {
    if (config_.rate_metrics_enabled && camera_id < kMaxCameras) {
      camera_metrics_[camera_id].RecordPublishSuccess(latency_ns);
    }
  }
  // 主/辅助ROS publish异常统一计入同一门禁，并由调用栈继续触发既有fail-closed。
  void RecordPublishFailure(uint32_t camera_id) noexcept {
    if (config_.rate_metrics_enabled && camera_id < kMaxCameras) {
      camera_metrics_[camera_id].RecordPublishFailure();
    }
  }


  // CameraInfo是辅助topic，单独提交成功计数，避免污染Image主topic频率。
  void RecordAuxiliaryPublishSuccess(uint32_t camera_id) noexcept {
    if (config_.rate_metrics_enabled && camera_id < kMaxCameras) {
      camera_metrics_[camera_id].RecordAuxiliaryPublishSuccess();
    }
  }

  // timer仅读取累计atomic snapshot；返回值按值复制，避免把metrics锁或生命周期暴露到ROS层。
  robobaton_4p_ros2_demo::PublicationRateSnapshot Snapshot(
      uint32_t camera_id, uint64_t monotonic_ns) const noexcept {
    if (camera_id >= kMaxCameras) {
      return {};
    }
    return camera_metrics_[camera_id].Snapshot(monotonic_ns);
  }


  bool failed() const noexcept { return failed_.load(std::memory_order_acquire); }

#ifdef RELEASE008_CONTRACT_TEST
  void FailNextQueueAllocation() {
    for (auto& queue : queues_) {
      if (queue) {
        queue->FailNextAllocation();
      }
    }
  }
  void ThrowInCallback() noexcept { throw_in_callback_.store(true); }
  void FailJoin() noexcept { fail_join_.store(true); }
#endif

 private:
  struct CallbackBridge {
    std::atomic<CameraLifecycleCore*> owner{nullptr};
    std::atomic<bool> accepting{false};
  };

  bool ValidateConfig() const noexcept {
    const uint32_t valid_mask = (1U << kMaxCameras) - 1U;
    return config_.camera_mask != 0U && (config_.camera_mask & ~valid_mask) == 0U &&
           config_.queue_capacity > 0U && config_.width > 0U && config_.height > 0U &&
           config_.timeout_ms > 0U && config_.max_skew_ns > 0U;
  }

  uint32_t EnabledCameraCount() const noexcept {
    uint32_t count = 0U;
    for (uint32_t camera_id = 0; camera_id < kMaxCameras; ++camera_id) {
      if (CameraEnabled(camera_id)) {
        ++count;
      }
    }
    return count;
  }

  bool CameraEnabled(uint32_t camera_id) const noexcept {
    return camera_id < kMaxCameras && (config_.camera_mask & (1U << camera_id)) != 0U;
  }

  void InitializeQueues() {
    for (uint32_t camera_id = 0; camera_id < kMaxCameras; ++camera_id) {
      if (CameraEnabled(camera_id)) {
        queues_[camera_id] =
            std::make_unique<FrameQueue>(config_.queue_capacity, config_.drop_newest);
      }
    }
  }

  void StartWorkers() {
    for (uint32_t camera_id = 0; camera_id < kMaxCameras; ++camera_id) {
      if (CameraEnabled(camera_id)) {
        workers_[camera_id] = std::thread(&CameraLifecycleCore::WorkerLoop, this, camera_id);
      }
    }
  }

  static void FrameSetTrampoline(const sc132_frame_set_t* frame_set, void* user_data) noexcept {
    auto* bridge = static_cast<CallbackBridge*>(user_data);
    if (bridge == nullptr || !bridge->accepting.load(std::memory_order_acquire)) {
      return;
    }
    CameraLifecycleCore* owner = bridge->owner.load(std::memory_order_acquire);
    if (owner == nullptr) {
      return;
    }
    try {
      owner->HandleFrameSet(frame_set);
    } catch (...) {
      owner->CloseAdmissionAndRequestStop();
      owner->RecordFailure();
      RecordContractEvent("consumer.sc.callback_failure");
    }
  }

  void HandleFrameSet(const sc132_frame_set_t* frame_set) {
#ifdef RELEASE008_CONTRACT_TEST
    if (throw_in_callback_.exchange(false)) {
      throw std::runtime_error("injected callback exception");
    }
#endif
    if (frame_set == nullptr || frame_set->struct_size != sizeof(*frame_set) ||
        frame_set->camera_count != EnabledCameraCount() || frame_set->camera_count == 0U ||
        frame_set->camera_count > kMaxCameras) {
      throw std::runtime_error("invalid frame-set header");
    }
    std::array<bool, kMaxCameras> seen{};
    for (uint32_t index = 0; index < frame_set->camera_count; ++index) {
      const sc132_frame_set_item_t& item = frame_set->items[index];
      const uint32_t camera_id = item.camera_id;
      if (!CameraEnabled(camera_id) || seen[camera_id] || item.frame == nullptr) {
        throw std::runtime_error("invalid frame-set item");
      }
      seen[camera_id] = true;
      sc132_frame_info_t info{};
      info.struct_size = sizeof(info);
      if (sc132_frame_get_info(item.frame, &info) != SC132_STATUS_OK ||
          info.struct_size != sizeof(info) || info.camera_id != camera_id ||
          info.y_data == nullptr || info.uv_data == nullptr || info.y_size == 0U ||
          info.uv_size == 0U || info.width == 0U || info.height == 0U ||
          info.stride == 0U || info.vstride == 0U) {
        throw std::runtime_error("invalid frame metadata");
      }
      // 指标定义为最终 callback 所见的 producer 连续性；
      // 记录真实 sequence，使聚合器丢弃过的 producer 帧继续表现为 source gap，而非被 group_id 掩盖。
      if (config_.rate_metrics_enabled) {
        camera_metrics_[camera_id].RecordSource(
            info.sequence, info.timestamp_ns, SteadyNowNs());

      }
      sc132_frame_t* frame = item.frame;
      const int32_t retain_status = sc132_frame_retain(frame);  // RELEASE008_RETAIN_CALL
      if (retain_status != SC132_STATUS_OK) {
        throw std::runtime_error("frame retain failed");
      }
      RetainedFrameJob job(frame, info);
      if (!queues_[camera_id]->Push(std::move(job))) {
        if (config_.rate_metrics_enabled) {
          camera_metrics_[camera_id].RecordDrop();
        }
        continue;
      }
    }
  }

  void WorkerLoop(uint32_t camera_id) noexcept {
    RecordContractEvent("consumer.sc.worker_start");
    try {
      RetainedFrameJob job;
      while (queues_[camera_id]->Pop(&job)) {
        sink_(job);
        job.Reset();
      }
    } catch (...) {
      RecordContractEvent("consumer.sc.worker_failure");
      RecordFailure();
      CloseAdmissionAndRequestStop();
    }
  }

  // 统一使用CLOCK_MONOTONIC同源的steady_clock纳秒，避免ROS/wall时间调整污染耗时证据。
  static uint64_t SteadyNowNs() noexcept {
    return static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
                                     std::chrono::steady_clock::now().time_since_epoch())
                                     .count());
  }


  void RecordFailure() noexcept {
    const bool first = !failed_.exchange(true, std::memory_order_acq_rel);
    if (first && failure_notifier_) {
      try {
        failure_notifier_();
      } catch (...) {
      }
    }
  }

  void CloseAdmissionAndRequestStop() noexcept {
    bridge_->accepting.store(false, std::memory_order_release);
    if (!accepting_closed_.exchange(true, std::memory_order_acq_rel)) {
      RecordContractEvent("consumer.sc.accepting_false");
    }
    if (!stop_requested_.exchange(true, std::memory_order_acq_rel)) {
      sc132_request_stop();
    }
    for (auto& queue : queues_) {
      if (queue) {
        queue->CloseAdmission();
      }
    }
  }

  bool CleanupLocked() noexcept {
    if (!start_claimed_) {
      return true;
    }
    if (cleanup_complete_) {
      return cleanup_quiesced_;
    }
    terminal_ = true;
    CloseAdmissionAndRequestStop();

    RecordContractEvent("consumer.sc.detach_queues");
    std::array<std::deque<RetainedFrameJob>, kMaxCameras> detached;
    for (std::size_t camera_id = 0; camera_id < kMaxCameras; ++camera_id) {
      if (queues_[camera_id]) {
        queues_[camera_id]->StopAndDetach(&detached[camera_id]);
      }
    }
    // 先在各队列锁内 detach，再在所有队列锁之外统一归还 retained frames。
    for (auto& batch : detached) {
      batch.clear();
    }

    RecordContractEvent("consumer.sc.join_workers");
#ifdef RELEASE008_CONTRACT_TEST
    if (fail_join_.load(std::memory_order_acquire)) {
      RecordContractEvent("consumer.sc.join_failure");
      cleanup_quiesced_ = false;
      RecordFailure();
      return false;
    }
#endif
    try {
      for (auto& worker : workers_) {
        if (worker.joinable()) {
          worker.join();
        }
      }
    } catch (const std::system_error&) {
      cleanup_quiesced_ = false;
      RecordFailure();
      return false;
    }

    // 同一 owner 连续调用两次 stop，第二次用于幂等确认或重试 STOPPING cleanup。
    sc132_stop();  // RELEASE008_SC_STOP_CALL_1
    sc132_stop();  // RELEASE008_SC_STOP_CALL_2
    running_ = false;
    cleanup_quiesced_ = true;
    cleanup_complete_ = true;
    sink_ = {};
    failure_notifier_ = {};
    return true;
  }

#ifdef RELEASE008_CONTRACT_TEST
  void ForceJoinForTest() noexcept {
    std::array<std::deque<RetainedFrameJob>, kMaxCameras> detached;
    for (std::size_t camera_id = 0; camera_id < kMaxCameras; ++camera_id) {
      if (queues_[camera_id]) {
        queues_[camera_id]->CloseAdmission();
        queues_[camera_id]->StopAndDetach(&detached[camera_id]);
      }
    }
    for (auto& batch : detached) {
      batch.clear();
    }
    for (auto& worker : workers_) {
      if (worker.joinable()) {
        worker.join();
      }
    }
    bridge_->accepting.store(false, std::memory_order_release);
  }
#endif

  CameraCoreConfig config_{};
  CallbackBridge* const bridge_;
  Sink sink_;
  FailureNotifier failure_notifier_;
  std::array<std::unique_ptr<FrameQueue>, kMaxCameras> queues_{};
  std::array<std::thread, kMaxCameras> workers_{};
  std::array<robobaton_4p_ros2_demo::PublicationRateMetrics, kMaxCameras> camera_metrics_{
      robobaton_4p_ros2_demo::PublicationRateMetrics(robobaton_4p_ros2_demo::SequenceWidth::k64Bit),
      robobaton_4p_ros2_demo::PublicationRateMetrics(robobaton_4p_ros2_demo::SequenceWidth::k64Bit),
      robobaton_4p_ros2_demo::PublicationRateMetrics(robobaton_4p_ros2_demo::SequenceWidth::k64Bit),
      robobaton_4p_ros2_demo::PublicationRateMetrics(robobaton_4p_ros2_demo::SequenceWidth::k64Bit)};
  std::mutex lifecycle_mutex_;
  std::atomic<bool> failed_{false};
  std::atomic<bool> accepting_closed_{false};
  std::atomic<bool> stop_requested_{false};
  bool start_claimed_ = false;
  bool start_attempted_ = false;
  bool running_ = false;
  bool terminal_ = false;
  bool cleanup_complete_ = false;
  bool cleanup_quiesced_ = false;
#ifdef RELEASE008_CONTRACT_TEST
  std::atomic<bool> throw_in_callback_{false};
  std::atomic<bool> fail_join_{false};
#endif
};

}  // namespace

#ifdef RELEASE008_CONTRACT_TEST

namespace release008_test {

struct CameraHarness {
  CameraLifecycleCore core;
  CameraHarnessConfig config;
  std::atomic<bool> throw_in_worker{false};
  std::atomic<bool> throw_in_auxiliary_publish{false};
};

void* CameraHarnessCreate(const CameraHarnessConfig& config) {
  auto* harness = new CameraHarness();
  harness->config = config;
  return harness;
}

bool CameraHarnessStart(void* opaque) {
  auto* harness = static_cast<CameraHarness*>(opaque);
  CameraCoreConfig config;
  config.camera_mask = harness->config.camera_mask;
  config.fps = harness->config.fps;
  config.rotation = harness->config.rotation;
  config.width = harness->config.width;
  config.height = harness->config.height;
  config.timeout_ms = harness->config.timeout_ms;
  config.max_skew_ns = harness->config.max_skew_ns;
  config.queue_capacity = harness->config.queue_capacity;
  config.rate_metrics_enabled = harness->config.rate_metrics_enabled;
  return harness->core.Start(
      config,
      [harness](const RetainedFrameJob& job) {
        const uint32_t camera_id = job.info().camera_id;
        // host harness按production顺序模拟主/辅助发布，验证任一异常都会计数后rethrow。
        try {
          if (harness->throw_in_worker.exchange(false)) {
            throw std::runtime_error("injected image publish failure");
          }
          harness->core.RecordPublishSuccess(camera_id, 25U);
          if (harness->throw_in_auxiliary_publish.exchange(false)) {
            throw std::runtime_error("injected camera info publish failure");
          }
          harness->core.RecordAuxiliaryPublishSuccess(camera_id);
        } catch (...) {
          harness->core.RecordPublishFailure(camera_id);
          throw;
        }
      },
      [] {});
}

bool CameraHarnessStop(void* opaque) {
  return static_cast<CameraHarness*>(opaque)->core.Cleanup();
}

void CameraHarnessDestroy(void* opaque) { delete static_cast<CameraHarness*>(opaque); }

void CameraHarnessFailNextQueueAllocation(void* opaque) {
  static_cast<CameraHarness*>(opaque)->core.FailNextQueueAllocation();
}

void CameraHarnessThrowInCallback(void* opaque) {
  static_cast<CameraHarness*>(opaque)->core.ThrowInCallback();
}

void CameraHarnessThrowInWorker(void* opaque) {
  static_cast<CameraHarness*>(opaque)->throw_in_worker.store(true);
}

void CameraHarnessThrowInAuxiliaryPublish(void* opaque) {
  static_cast<CameraHarness*>(opaque)->throw_in_auxiliary_publish.store(true);
}

void CameraHarnessFailJoin(void* opaque) {
  static_cast<CameraHarness*>(opaque)->core.FailJoin();
}

bool CameraHarnessFailed(void* opaque) {
  return static_cast<CameraHarness*>(opaque)->core.failed();
}

robobaton_4p_ros2_demo::PublicationRateSnapshot CameraHarnessSnapshot(
    void* opaque, uint32_t camera_id, uint64_t monotonic_ns) {
  return static_cast<CameraHarness*>(opaque)->core.Snapshot(camera_id, monotonic_ns);
}

}  // namespace release008_test

#else

namespace robobaton_4p_ros2_demo {
namespace {

constexpr char kImageTopicSuffix[] = "/image_raw";
constexpr char kCameraInfoTopicSuffix[] = "/camera_info";

std::string CameraTopic(uint32_t camera_id, const char* suffix) {
  return "/robobaton/cam" + std::to_string(camera_id) + suffix;
}

std::string CameraFrameId(const std::string& prefix, uint32_t camera_id) {
  return prefix + std::to_string(camera_id) + "_optical_frame";
}

CameraLifecycleCore& ProcessCameraCore() {
  // camera core 与进程同寿命，完成关闭后禁止同进程重启。
  static CameraLifecycleCore* core = new CameraLifecycleCore();
  return *core;
}

}  // namespace

class CameraPublisher::Impl {
 public:
  Impl(rclcpp::Node* node, Config config)
      : node_(node), config_(std::move(config)), core_(ProcessCameraCore()) {
    if (node_ == nullptr || config_.queue_capacity == 0U || config_.image_encoding != "nv12" ||
        (config_.rate_metrics_enabled && config_.rate_log_period_ms == 0U)) {
      throw std::invalid_argument("invalid camera publisher configuration");
    }
    auto image_qos = rclcpp::SensorDataQoS();
    image_qos.keep_last(2);
    auto info_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    for (uint32_t camera_id = 0; camera_id < kMaxCameras; ++camera_id) {
      if (!robobaton_demo::CameraMaskContains(config_.options.camera_mask,
                                               static_cast<int>(camera_id))) {
        continue;
      }
      image_publishers_[camera_id] = node_->create_publisher<sensor_msgs::msg::Image>(
          CameraTopic(camera_id, kImageTopicSuffix), image_qos);
      if (config_.publish_camera_info) {
        camera_info_publishers_[camera_id] =
            node_->create_publisher<sensor_msgs::msg::CameraInfo>(
                CameraTopic(camera_id, kCameraInfoTopicSuffix), info_qos);
      }
    }
  }

  ~Impl() { Stop(); }

  void Start() {
    robobaton_demo::ConfigureSc132TriggerMode(config_.options);
    robobaton_demo::ConfigureSc132SensorProfile(config_.options);
    CameraCoreConfig core_config;
    core_config.camera_mask = config_.options.camera_mask;
    core_config.fps = static_cast<uint32_t>(config_.options.fps);
    core_config.rotation =
        static_cast<uint32_t>(robobaton_demo::InternalRotateDegrees(config_.options));
    core_config.width = robobaton_demo::Sc132OutputWidth(config_.options);
    core_config.height = robobaton_demo::Sc132OutputHeight(config_.options);
    core_config.timeout_ms = config_.options.frame_set_timeout_ms;
    core_config.max_skew_ns = config_.options.frame_set_max_skew_ns;
    core_config.queue_capacity = config_.queue_capacity;
    core_config.drop_newest = config_.queue_policy == QueuePolicy::kDropNewest;
    core_config.rate_metrics_enabled = config_.rate_metrics_enabled;
    if (!core_.Start(
            core_config,
            [this](const RetainedFrameJob& job) { PublishFrame(job); },
            [this] {
              robobaton_4p_ros2_demo::RecordProcessFailure();
              RCLCPP_ERROR(node_->get_logger(), "SC132 lifecycle failure; shutting down");
              rclcpp::shutdown();
            })) {
      throw std::runtime_error("SC132 publisher start failed or restart was rejected");
    }
    started_ = true;
    RCLCPP_INFO(node_->get_logger(), "Started SC132 camera publisher mask=0x%X",
                config_.options.camera_mask);
    if (config_.rate_metrics_enabled) {
      previous_snapshots_.fill({});
      const auto period = std::chrono::milliseconds(config_.rate_log_period_ms);
      rate_timer_ = rclcpp::create_wall_timer(
          period, [this] { LogRateMetrics(); }, nullptr, node_->get_node_base_interface().get(),
          node_->get_node_timers_interface().get());
    }
  }

  void Stop() noexcept {
    rate_timer_.reset();
    if (!started_ && !core_.failed()) {
      return;
    }
    if (!core_.Cleanup()) {
      robobaton_4p_ros2_demo::RecordProcessFailure();
      RCLCPP_FATAL(node_->get_logger(), "SC132 worker join failed; terminating fail-closed");
      std::_Exit(1);
    }
    if (core_.failed()) {
      robobaton_4p_ros2_demo::RecordProcessFailure();
    }
    started_ = false;
  }

 private:
  // 固定schema的单行JSON直接进入node stderr证据；默认关闭时不会创建timer或产生额外日志。
  void LogRateMetrics() {
    const uint64_t now_ns = SteadyNowNs();
    for (uint32_t camera_id = 0; camera_id < kMaxCameras; ++camera_id) {
      if (!robobaton_demo::CameraMaskContains(config_.options.camera_mask,
                                               static_cast<int>(camera_id))) {
        continue;
      }
      const auto current = core_.Snapshot(camera_id, now_ns);
      const auto delta = robobaton_4p_ros2_demo::PublicationRateMetrics::Delta(
          previous_snapshots_[camera_id], current);
      previous_snapshots_[camera_id] = current;

      // 首个窗口从producer首样本开始，之后严格使用相邻timer边界；保留原始count和ns供主机复算。
      const uint64_t interval_start = delta.interval_start_monotonic_ns == 0U
                                          ? current.interval_start_monotonic_ns
                                          : delta.interval_start_monotonic_ns;
      RCLCPP_INFO(
          node_->get_logger(),
          "ROB2_RATE {\"schema\":\"robobaton-rate-v1\",\"run_id\":\"%s\",\"kind\":\"camera\",\"id\":%u,\"interval_start_monotonic_ns\":%llu,\"interval_end_monotonic_ns\":%llu,\"source\":%llu,\"published\":%llu,\"auxiliary_published\":%llu,\"dropped\":%llu,\"publish_failures\":%llu,\"sequence_gaps\":%llu,\"sequence_duplicates\":%llu,\"sequence_regressions\":%llu,\"timestamp_duplicates\":%llu,\"timestamp_regressions\":%llu,\"publish_latency_count\":%llu,\"publish_latency_sum_ns\":%llu,\"publish_latency_max_ns\":%llu}",
          config_.rate_run_id.c_str(), camera_id,
          static_cast<unsigned long long>(interval_start),
          static_cast<unsigned long long>(delta.interval_end_monotonic_ns),
          static_cast<unsigned long long>(delta.source_count),
          static_cast<unsigned long long>(delta.publish_count),
          static_cast<unsigned long long>(delta.auxiliary_publish_count),
          static_cast<unsigned long long>(delta.drop_count),
          static_cast<unsigned long long>(delta.publish_failure_count),
          static_cast<unsigned long long>(delta.sequence_gap_count),
          static_cast<unsigned long long>(delta.sequence_duplicate_count),
          static_cast<unsigned long long>(delta.sequence_regression_count),
          static_cast<unsigned long long>(delta.timestamp_duplicate_count),
          static_cast<unsigned long long>(delta.timestamp_regression_count),
          static_cast<unsigned long long>(delta.publish_latency_count),
          static_cast<unsigned long long>(delta.publish_latency_sum_ns),
          static_cast<unsigned long long>(delta.publish_latency_max_ns));
    }
  }

  // ROS publish耗时采用steady clock，不受ROS仿真时间或wall clock校时影响。
  static uint64_t SteadyNowNs() noexcept {
    return static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
                                     std::chrono::steady_clock::now().time_since_epoch())
                                     .count());
  }

  void PublishFrame(const RetainedFrameJob& job) {
    const auto& info = job.info();
    if (!job.owns_frame() || info.y_size > std::numeric_limits<std::size_t>::max() ||
        info.uv_size > std::numeric_limits<std::size_t>::max() ||
        info.y_size > std::numeric_limits<std::size_t>::max() - info.uv_size) {
      throw std::runtime_error("invalid SC132 frame size");
    }
    const uint32_t camera_id = info.camera_id;
    if (camera_id >= kMaxCameras || !image_publishers_[camera_id]) {
      throw std::runtime_error("invalid SC132 camera id");
    }
    sensor_msgs::msg::Image image;
    image.header.stamp = node_->now();
    image.header.frame_id = CameraFrameId(config_.frame_id_prefix, camera_id);
    image.height = info.height;
    image.width = info.width;
    image.encoding = config_.image_encoding;
    image.is_bigendian = false;
    image.step = info.stride;
    const std::size_t y_size = static_cast<std::size_t>(info.y_size);
    const std::size_t uv_size = static_cast<std::size_t>(info.uv_size);
    image.data.resize(y_size + uv_size);
    std::memcpy(image.data.data(), info.y_data, y_size);
    std::memcpy(image.data.data() + y_size, info.uv_data, uv_size);
    // 主Image或辅助CameraInfo任一publish异常都先计数再rethrow，保留worker既有fail-closed语义。
    try {
      if (config_.rate_metrics_enabled) {
        const uint64_t publish_start_ns = SteadyNowNs();
        image_publishers_[camera_id]->publish(image);
        core_.RecordPublishSuccess(camera_id, SteadyNowNs() - publish_start_ns);
      } else {
        image_publishers_[camera_id]->publish(image);
      }
      if (camera_info_publishers_[camera_id]) {
        sensor_msgs::msg::CameraInfo camera_info;
        camera_info.header = image.header;
        camera_info.width = image.width;
        camera_info.height = image.height;
        camera_info_publishers_[camera_id]->publish(camera_info);
        if (config_.rate_metrics_enabled) {
          core_.RecordAuxiliaryPublishSuccess(camera_id);
        }
      }
    } catch (...) {
      core_.RecordPublishFailure(camera_id);
      throw;
    }

  }

  rclcpp::Node* node_;
  Config config_;
  CameraLifecycleCore& core_;
  bool started_ = false;
  std::array<rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr, kMaxCameras>
      image_publishers_{};
  std::array<rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr, kMaxCameras>
      camera_info_publishers_{};
  rclcpp::TimerBase::SharedPtr rate_timer_;
  std::array<robobaton_4p_ros2_demo::PublicationRateSnapshot, kMaxCameras>
      previous_snapshots_{};
};

CameraPublisher::CameraPublisher(rclcpp::Node* node, Config config)
    : impl_(std::make_unique<Impl>(node, std::move(config))) {}
CameraPublisher::~CameraPublisher() = default;
void CameraPublisher::Start() { impl_->Start(); }
void CameraPublisher::Stop() { impl_->Stop(); }

}  // namespace robobaton_4p_ros2_demo

#endif
