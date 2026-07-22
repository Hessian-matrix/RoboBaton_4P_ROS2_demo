#pragma once

#include <atomic>
#include <cstdint>
#include <mutex>

namespace robobaton_4p_ros2_demo {

enum class SequenceWidth : uint8_t {
  kNone = 0,
  k32Bit = 32,
  k64Bit = 64,
};

struct PublicationRateSnapshot {
  uint64_t source_count = 0U;
  uint64_t publish_count = 0U;
  uint64_t auxiliary_publish_count = 0U;
  uint64_t drop_count = 0U;
  uint64_t publish_failure_count = 0U;
  uint64_t sequence_gap_count = 0U;
  uint64_t sequence_duplicate_count = 0U;
  uint64_t sequence_regression_count = 0U;
  uint64_t timestamp_duplicate_count = 0U;
  uint64_t timestamp_regression_count = 0U;
  uint64_t publish_latency_count = 0U;
  uint64_t publish_latency_sum_ns = 0U;
  uint64_t publish_latency_max_ns = 0U;
  uint64_t interval_start_monotonic_ns = 0U;
  uint64_t interval_end_monotonic_ns = 0U;
};

// 统一记录producer输入和ROS发布计数；高频更新不分配内存或输出日志。
class PublicationRateMetrics {
 public:
  explicit PublicationRateMetrics(SequenceWidth sequence_width) noexcept;

  // Camera使用真实sequence；timestamp和monotonic时间单位均固定为ns。
  void RecordSource(uint64_t sequence, uint64_t timestamp_ns,
                    uint64_t monotonic_ns) noexcept;
  // IMU公开ABI没有sequence，单独入口避免在ROS层伪造序号。
  void RecordSourceWithoutSequence(uint64_t timestamp_ns,
                                   uint64_t monotonic_ns) noexcept;
  void RecordPublishSuccess(uint64_t latency_ns = 0U) noexcept;
  void RecordAuxiliaryPublishSuccess() noexcept;
  void RecordDrop() noexcept;
  void RecordPublishFailure() noexcept;

  PublicationRateSnapshot Snapshot(uint64_t monotonic_ns) const noexcept;
  static PublicationRateSnapshot Delta(const PublicationRateSnapshot& before,
                                       const PublicationRateSnapshot& after) noexcept;

 private:
  void RecordTimestampLocked(uint64_t timestamp_ns) noexcept;
  void SetFirstMonotonic(uint64_t monotonic_ns) noexcept;

  const SequenceWidth sequence_width_;
  mutable std::mutex source_mutex_;
  bool sequence_initialized_ = false;
  bool timestamp_initialized_ = false;
  uint64_t last_sequence_ = 0U;
  uint64_t last_timestamp_ns_ = 0U;

  std::atomic<uint64_t> source_count_{0U};
  std::atomic<uint64_t> publish_count_{0U};
  std::atomic<uint64_t> auxiliary_publish_count_{0U};
  std::atomic<uint64_t> drop_count_{0U};
  std::atomic<uint64_t> publish_failure_count_{0U};
  std::atomic<uint64_t> sequence_gap_count_{0U};
  std::atomic<uint64_t> sequence_duplicate_count_{0U};
  std::atomic<uint64_t> sequence_regression_count_{0U};
  std::atomic<uint64_t> timestamp_duplicate_count_{0U};
  std::atomic<uint64_t> timestamp_regression_count_{0U};
  std::atomic<uint64_t> publish_latency_count_{0U};
  std::atomic<uint64_t> publish_latency_sum_ns_{0U};
  std::atomic<uint64_t> publish_latency_max_ns_{0U};
  std::atomic<uint64_t> first_monotonic_ns_{0U};
};

}  // namespace robobaton_4p_ros2_demo
