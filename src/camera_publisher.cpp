#include "robobaton_4p_ros2_demo/camera_publisher.hpp"

#include <array>
#include <atomic>
#include <exception>
#include <condition_variable>
#include <cstdint>
#include <cstring>
#include <cstddef>
#include <deque>
#include <mutex>
#include <stdexcept>
#include <memory>
#include <string>
#include <thread>
#include <utility>

#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

extern "C" {
#include "hb_mem_mgr.h"
#include "robobaton_4p_ros2_demo/sc132camera.h"
}

#include "robobaton_4p_ros2_demo/cam_demo_config.h"

namespace robobaton_4p_ros2_demo {

namespace {

using robobaton_demo::CameraMaskContains;
using robobaton_demo::CameraMaskPopCount;
using robobaton_demo::InternalRotateDegrees;
using robobaton_demo::IsSupportedCameraMask;
using robobaton_demo::SteadyClockNowNs;

constexpr int kMaxCameras = robobaton_demo::kMaxChannels;
constexpr char kImageTopicSuffix[] = "/image_raw";
constexpr char kCameraInfoTopicSuffix[] = "/camera_info";

struct CameraFrameJob {
  sc132_frame_t* frame = nullptr;
  uint32_t camera_id = 0;
  uint64_t sequence = 0;
  uint32_t frame_id = 0;
  uint64_t group_id = 0;
  uint64_t sensor_timestamp_ns = 0;
  uint64_t enqueue_timestamp_ns = 0;
  void* y_data = nullptr;
  void* uv_data = nullptr;
  uint64_t y_size = 0;
  uint64_t uv_size = 0;
  int width = 0;
  int height = 0;
  int stride = 0;
  int vstride = 0;
};

void ReleaseFrameJob(CameraFrameJob* job) {
  if (job != nullptr && job->frame != nullptr) {
    sc132_frame_release(job->frame);
    job->frame = nullptr;
  }
}

std::string CameraTopic(int camera_id, const char* suffix) {
  return "/robobaton/cam" + std::to_string(camera_id) + suffix;
}

std::string CameraFrameId(const std::string& prefix, int camera_id) {
  return prefix + std::to_string(camera_id) + "_optical_frame";
}

void ValidateCameraConfig(const CameraPublisher::Config& config) {
  const auto& options = config.options;
  if (!IsSupportedCameraMask(options.camera_mask)) {
    throw std::invalid_argument("camera.camera_mask supports only one physical camera or all four cameras");
  }
  if (options.channels != CameraMaskPopCount(options.camera_mask)) {
    throw std::invalid_argument("camera channels must match camera_mask popcount");
  }
  if (options.fps != 30 && options.fps != 60) {
    throw std::invalid_argument("camera fps must be 30 or 60");
  }
  if (options.rotate_degrees != 0 && options.rotate_degrees != 90 &&
      options.rotate_degrees != 180 && options.rotate_degrees != 270) {
    throw std::invalid_argument("camera rotate_degrees must be 0, 90, 180, or 270");
  }
  // 2026-07-09 修改原因：沿用 cam_demo 约束，对外 180 度在四路 60fps 下会触发底层慢路径，当前不支持。
  if (InternalRotateDegrees(options) == 270 && options.fps == 60) {
    throw std::invalid_argument("camera rotate 180 is not supported at 60fps; use fps=30 or rotate=0");
  }
  if (options.frame_set_max_skew_ns == 0 || options.frame_set_timeout_ms == 0) {
    throw std::invalid_argument("camera frame-set skew/timeout must be positive");
  }
  if (config.queue_capacity == 0) {
    throw std::invalid_argument("camera queue_capacity must be positive");
  }
  if (config.image_encoding != "nv12") {
    throw std::invalid_argument("camera image_encoding v1 only supports nv12");
  }
}

class FrameQueue {
 public:
  FrameQueue(std::size_t capacity, CameraPublisher::QueuePolicy policy)
      : capacity_(capacity), policy_(policy) {}

  FrameQueue(const FrameQueue&) = delete;
  FrameQueue& operator=(const FrameQueue&) = delete;

  bool Push(CameraFrameJob item) {
    std::unique_lock<std::mutex> lock(mutex_);
    if (stopped_) {
      return false;
    }
    if (policy_ == CameraPublisher::QueuePolicy::kDropNewest && queue_.size() >= capacity_) {
      ++dropped_newest_count_;
      return false;
    }

    if (queue_.size() >= capacity_) {
      ++full_wait_count_;
    }
    // 2026-07-09 修改原因：block 模式沿用非 ROS demo 的背压语义，避免静默丢掉四目同步帧。
    not_full_.wait(lock, [&] { return stopped_ || queue_.size() < capacity_; });
    if (stopped_) {
      return false;
    }

    queue_.push_back(item);
    lock.unlock();
    not_empty_.notify_one();
    return true;
  }

  bool Pop(CameraFrameJob* item) {
    std::unique_lock<std::mutex> lock(mutex_);
    not_empty_.wait(lock, [&] { return stopped_ || !queue_.empty(); });
    if (queue_.empty()) {
      return false;
    }

    *item = queue_.front();
    queue_.pop_front();
    lock.unlock();
    not_full_.notify_one();
    return true;
  }

  void Stop() {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      stopped_ = true;
    }
    not_empty_.notify_all();
    not_full_.notify_all();
  }

  std::size_t size() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return queue_.size();
  }

  uint64_t full_wait_count() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return full_wait_count_;
  }

  uint64_t dropped_newest_count() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return dropped_newest_count_;
  }

 private:
  const std::size_t capacity_;
  const CameraPublisher::QueuePolicy policy_;
  mutable std::mutex mutex_;
  std::condition_variable not_empty_;
  std::condition_variable not_full_;
  std::deque<CameraFrameJob> queue_;
  uint64_t full_wait_count_ = 0;
  uint64_t dropped_newest_count_ = 0;
  bool stopped_ = false;
};

}  // namespace

class CameraPublisher::Impl {
 public:
  Impl(rclcpp::Node* node, Config config) : node_(node), config_(std::move(config)) {
    ValidateCameraConfig(config_);

    auto image_qos = rclcpp::SensorDataQoS();
    image_qos.keep_last(2);
    auto camera_info_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();

    InitializeQueues();
    for (int camera_id = 0; camera_id < kMaxCameras; ++camera_id) {
      if (!CameraMaskContains(config_.options.camera_mask, camera_id)) {
        continue;
      }
      image_publishers_[camera_id] = node_->create_publisher<sensor_msgs::msg::Image>(
          CameraTopic(camera_id, kImageTopicSuffix), image_qos);
      if (config_.publish_camera_info) {
        camera_info_publishers_[camera_id] =
            node_->create_publisher<sensor_msgs::msg::CameraInfo>(
                CameraTopic(camera_id, kCameraInfoTopicSuffix), camera_info_qos);
      }
    }
  }

  ~Impl() { Stop(); }

  void Start() {
    std::lock_guard<std::mutex> lock(lifecycle_mutex_);
    if (started_) {
      return;
    }

    stopping_.store(false);
    ResetQueuesForStart();
    try {
      StartUnlocked();
      started_ = true;
    } catch (...) {
      StopUnlocked();
      throw;
    }
  }

  void Stop() {
    std::lock_guard<std::mutex> lock(lifecycle_mutex_);
    StopUnlocked();
  }

 private:
  void InitializeQueues() {
    for (auto& queue : queues_) {
      queue = std::make_unique<FrameQueue>(config_.queue_capacity, config_.queue_policy);
    }
  }

  void ResetQueuesForStart() {
    // 2026-07-09 修改原因：FrameQueue::Stop() 后不可复用；每次启动前重建队列，避免重启后 Push/Pop 永远停在 stopped 状态。
    InitializeQueues();
  }

  void StopQueues() {
    for (auto& queue : queues_) {
      if (queue) {
        queue->Stop();
      }
    }
  }

  void StartUnlocked() {
    const auto& options = config_.options;
    if (VioCamSetFps(options.fps) != 0) {
      throw std::runtime_error("VioCamSetFps failed");
    }
    if (VioCamSetOutputRotate(InternalRotateDegrees(options)) != 0) {
      throw std::runtime_error("VioCamSetOutputRotate failed");
    }
    robobaton_demo::ConfigureSc132TriggerMode(options);
    robobaton_demo::ConfigureSc132SensorProfile(options);

    // 2026-07-09 修改原因：SC132 输出 DMA 图像依赖 hbmem，必须先打开模块再启动相机链路。
    if (hb_mem_module_open() != 0) {
      throw std::runtime_error("hb_mem_module_open failed");
    }
    hbmem_opened_ = true;

    for (int camera_id = 0; camera_id < kMaxCameras; ++camera_id) {
      if (CameraMaskContains(options.camera_mask, camera_id)) {
        workers_[camera_id] = std::thread(&Impl::WorkerLoop, this, camera_id);
      }
    }

    sc132_frame_set_config_t frame_set_config{};
    frame_set_config.cb = &Impl::FrameSetCallback;
    frame_set_config.user = this;
    frame_set_config.camera_count = CameraMaskPopCount(options.camera_mask);
    frame_set_config.max_skew_ns = options.frame_set_max_skew_ns;
    frame_set_config.timeout_ms = options.frame_set_timeout_ms;
    // 2026-07-09 修改原因：沿用 cam_demo，libsc132 内部必须使用 sensor 原始尺寸，不使用对外输出尺寸。
    frame_set_config.width = robobaton_demo::kSensorInputWidth;
    frame_set_config.height = robobaton_demo::kSensorInputHeight;

    if (VioCamInitmFrameSetMask(&frame_set_config, options.camera_mask) != 0) {
      throw std::runtime_error("VioCamInitmFrameSetMask failed");
    }
    camera_started_ = true;

    RCLCPP_INFO(node_->get_logger(), "Started SC132 ROS2 camera publisher mask=0x%X fps=%d",
                options.camera_mask, options.fps);
  }

  void StopUnlocked() {
    stopping_.store(true);

    StopQueues();

    if (camera_started_) {
      VioCamClose();
      camera_started_ = false;
    }
    for (auto& worker : workers_) {
      if (worker.joinable()) {
        worker.join();
      }
    }

    if (hbmem_opened_) {
      (void)hb_mem_module_close();
      hbmem_opened_ = false;
    }
    if (started_) {
      RCLCPP_INFO(node_->get_logger(), "Stopped SC132 ROS2 camera publisher");
    }
    started_ = false;
  }

  static void FrameSetCallback(const sc132_frame_set_t* frame_set, void* user) {
    if (user == nullptr) {
      return;
    }
    static_cast<Impl*>(user)->HandleFrameSet(frame_set);
  }

  void HandleFrameSet(const sc132_frame_set_t* frame_set) {
    if (frame_set == nullptr || stopping_.load()) {
      return;
    }
    if (frame_set->camera_count > kMaxCameras) {
      invalid_frame_set_count_.fetch_add(1, std::memory_order_relaxed);
      return;
    }

    for (uint32_t i = 0; i < frame_set->camera_count; ++i) {
      const sc132_frame_set_item_t& item = frame_set->items[i];
      const int camera_id = static_cast<int>(item.camera_id);
      if (!CameraMaskContains(config_.options.camera_mask, camera_id) || item.frame == nullptr) {
        continue;
      }
      EnqueueCameraFrame(camera_id, item, *frame_set);
    }
  }

  void EnqueueCameraFrame(int camera_id, const sc132_frame_set_item_t& item,
                          const sc132_frame_set_t& frame_set) {
    sc132_frame_t* frame = item.frame;
    const hb_mem_graphic_buf_t* graph_buf = sc132_frame_get_graphic_buf(frame);
    if (graph_buf == nullptr || graph_buf->virt_addr[0] == nullptr ||
        graph_buf->virt_addr[1] == nullptr || graph_buf->size[0] == 0 ||
        graph_buf->size[1] == 0 || graph_buf->stride <= 0 || graph_buf->vstride <= 0) {
      invalid_dma_frame_counts_[camera_id].fetch_add(1, std::memory_order_relaxed);
      return;
    }

    if (sc132_frame_retain(frame) != 0) {
      retain_failed_counts_[camera_id].fetch_add(1, std::memory_order_relaxed);
      return;
    }

    CameraFrameJob job;
    job.frame = frame;
    job.camera_id = static_cast<uint32_t>(camera_id);
    job.sequence = item.sequence;
    job.frame_id = item.frame_id;
    job.group_id = frame_set.group_id;
    job.sensor_timestamp_ns = item.timestamp_ns;
    job.enqueue_timestamp_ns = SteadyClockNowNs();
    job.y_data = graph_buf->virt_addr[0];
    job.uv_data = graph_buf->virt_addr[1];
    job.y_size = graph_buf->size[0];
    job.uv_size = graph_buf->size[1];
    job.width = item.width > 0 ? item.width : sc132_frame_get_width(frame);
    job.height = item.height > 0 ? item.height : sc132_frame_get_height(frame);
    job.stride = graph_buf->stride;
    job.vstride = graph_buf->vstride;

    // 2026-07-09 修改原因：采集回调只做 retain+metadata+enqueue，大拷贝和 ROS publish 放到 worker。
    if (!queues_[camera_id]->Push(job)) {
      ReleaseFrameJob(&job);
    }
  }

  void HandleWorkerFailure(int camera_id, CameraFrameJob* job, const char* reason) {
    // 2026-07-09 修改原因：worker 不能调用 StopUnlocked 避免自 join；只停队列并请求 ROS shutdown。
    ReleaseFrameJob(job);
    stopping_.store(true);
    StopQueues();
    RCLCPP_ERROR(node_->get_logger(), "cam%d worker failed: %s; shutting down node", camera_id,
                 reason);
    rclcpp::shutdown();
  }

  void WorkerLoop(int camera_id) {
    CameraFrameJob job;
    // 2026-07-09 修改原因：worker 顶层兜底异常，避免 ROS publish 或内存分配异常触发 std::terminate。
    try {
      while (queues_[camera_id]->Pop(&job)) {
        PublishFrame(camera_id, job);
        ReleaseFrameJob(&job);
      }
    } catch (const std::exception& e) {
      HandleWorkerFailure(camera_id, &job, e.what());
    } catch (...) {
      HandleWorkerFailure(camera_id, &job, "unknown exception");
    }
  }

  void PublishFrame(int camera_id, const CameraFrameJob& job) {
    if (job.y_data == nullptr || job.uv_data == nullptr || job.width <= 0 || job.height <= 0 ||
        job.stride <= 0 || job.y_size == 0 || job.uv_size == 0) {
      RCLCPP_WARN(node_->get_logger(), "cam%d invalid queued frame, skip publish", camera_id);
      return;
    }

    auto image_msg = sensor_msgs::msg::Image();
    // 2026-07-09 修改原因：sensor timestamp 时间域未确认，第一版统一使用 ROS 发布时间。
    image_msg.header.stamp = node_->now();
    image_msg.header.frame_id = CameraFrameId(config_.frame_id_prefix, camera_id);
    image_msg.height = static_cast<uint32_t>(job.height);
    image_msg.width = static_cast<uint32_t>(job.width);
    image_msg.encoding = config_.image_encoding;
    image_msg.is_bigendian = false;
    image_msg.step = static_cast<uint32_t>(job.stride);

    const std::size_t y_size = static_cast<std::size_t>(job.y_size);
    const std::size_t uv_size = static_cast<std::size_t>(job.uv_size);
    image_msg.data.resize(y_size + uv_size);
    std::memcpy(image_msg.data.data(), job.y_data, y_size);
    std::memcpy(image_msg.data.data() + y_size, job.uv_data, uv_size);

    image_publishers_[camera_id]->publish(image_msg);
    ++published_counts_[camera_id];

    if (camera_info_publishers_[camera_id]) {
      auto info_msg = sensor_msgs::msg::CameraInfo();
      info_msg.header = image_msg.header;
      info_msg.width = image_msg.width;
      info_msg.height = image_msg.height;
      info_msg.distortion_model = "";
      camera_info_publishers_[camera_id]->publish(info_msg);
    }
  }

  std::atomic<uint64_t> invalid_frame_set_count_{0};
  std::array<std::atomic<uint64_t>, kMaxCameras> invalid_dma_frame_counts_{};
  std::array<std::atomic<uint64_t>, kMaxCameras> retain_failed_counts_{};
  rclcpp::Node* node_ = nullptr;
  Config config_;
  std::mutex lifecycle_mutex_;
  std::atomic<bool> stopping_{false};
  bool started_ = false;
  bool hbmem_opened_ = false;
  bool camera_started_ = false;
  std::array<std::unique_ptr<FrameQueue>, kMaxCameras> queues_{};
  std::array<std::thread, kMaxCameras> workers_{};
  std::array<rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr, kMaxCameras> image_publishers_{};
  std::array<rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr, kMaxCameras>
      camera_info_publishers_{};
  std::array<std::atomic<uint64_t>, kMaxCameras> published_counts_{};
};

CameraPublisher::CameraPublisher(rclcpp::Node* node, Config config)
    : impl_(std::make_unique<Impl>(node, std::move(config))) {}

CameraPublisher::~CameraPublisher() = default;

void CameraPublisher::Start() { impl_->Start(); }

void CameraPublisher::Stop() { impl_->Stop(); }

}  // namespace robobaton_4p_ros2_demo
