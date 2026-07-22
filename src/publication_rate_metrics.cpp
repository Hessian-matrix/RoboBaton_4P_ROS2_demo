#include "robobaton_4p_ros2_demo/publication_rate_metrics.hpp"

#include <limits>

namespace robobaton_4p_ros2_demo {
namespace {

// 相邻snapshot可能来自不同生命周期或损坏证据；计数倒退时饱和到0，禁止无符号下溢伪造巨量事件。
uint64_t SaturatingSubtract(uint64_t after, uint64_t before) noexcept {
  return after >= before ? after - before : 0U;
}

// 按producer真实序号位宽计算唯一后继；最大值到0属于合法wrap，不记为gap或回退。
uint64_t ExpectedNext(uint64_t sequence, SequenceWidth width) noexcept {
  if (width == SequenceWidth::k32Bit) {
    return static_cast<uint32_t>(static_cast<uint32_t>(sequence) + 1U);
  }
  return sequence == std::numeric_limits<uint64_t>::max() ? 0U : sequence + 1U;
}

// SC132 frame_id按32位合同比较；进入共享状态前截断高位，避免调用方扩展污染wrap判定。
uint64_t NormalizeSequence(uint64_t sequence, SequenceWidth width) noexcept {
  return width == SequenceWidth::k32Bit ? static_cast<uint32_t>(sequence) : sequence;
}

}  // namespace

PublicationRateMetrics::PublicationRateMetrics(SequenceWidth sequence_width) noexcept
    : sequence_width_(sequence_width) {}

// 首个非零monotonic时间只允许一次发布；原子CAS避免多路camera callback竞争覆盖统计起点。
void PublicationRateMetrics::SetFirstMonotonic(uint64_t monotonic_ns) noexcept {
  if (monotonic_ns == 0U) {
    return;
  }
  uint64_t expected = 0U;
  (void)first_monotonic_ns_.compare_exchange_strong(
      expected, monotonic_ns, std::memory_order_release, std::memory_order_relaxed);
}

// timestamp历史与sequence历史共用source_mutex_串行更新，保证duplicate/regression相对同一到达顺序判断。
void PublicationRateMetrics::RecordTimestampLocked(uint64_t timestamp_ns) noexcept {
  // 相等单独统计duplicate；只有严格变小才是regression，正常抖动和较大间隔不在本层推断掉样。
  if (timestamp_initialized_) {
    if (timestamp_ns == last_timestamp_ns_) {
      timestamp_duplicate_count_.fetch_add(1U, std::memory_order_relaxed);
    } else if (timestamp_ns < last_timestamp_ns_) {
      timestamp_regression_count_.fetch_add(1U, std::memory_order_relaxed);
    }
  }
  timestamp_initialized_ = true;
  last_timestamp_ns_ = timestamp_ns;
}

// 记录带真实sequence的producer输入；计数器走atomic热路径，跨样本关联状态由短临界区保护。
void PublicationRateMetrics::RecordSource(uint64_t sequence, uint64_t timestamp_ns,
                                          uint64_t monotonic_ns) noexcept {
  SetFirstMonotonic(monotonic_ns);
  source_count_.fetch_add(1U, std::memory_order_relaxed);
  // 锁只覆盖last sequence/timestamp复合状态；publish/drop独立atomic，避免ROS发布线程和采集线程互相阻塞。
  std::lock_guard<std::mutex> lock(source_mutex_);
  const uint64_t normalized = NormalizeSequence(sequence, sequence_width_);
  if (sequence_width_ != SequenceWidth::kNone && sequence_initialized_) {
    const uint64_t expected = ExpectedNext(last_sequence_, sequence_width_);
    // 32位序号跨越最大值时normalized会变小；仅“上一值在高半区且新值在低半区”视为向前wrap。
    if (normalized == last_sequence_) {
      sequence_duplicate_count_.fetch_add(1U, std::memory_order_relaxed);
    } else if (normalized != expected) {
      const bool wrapped_forward =
          sequence_width_ == SequenceWidth::k32Bit && last_sequence_ > UINT32_MAX / 2U &&
          normalized < UINT32_MAX / 2U;
      // 非期望后继且向前推进记一个gap事件，不在这里估算缺失帧数量；向后移动记regression。
      if (normalized > expected || wrapped_forward) {
        sequence_gap_count_.fetch_add(1U, std::memory_order_relaxed);
      } else {
        sequence_regression_count_.fetch_add(1U, std::memory_order_relaxed);
      }
    }
  }
  sequence_initialized_ = sequence_width_ != SequenceWidth::kNone;
  last_sequence_ = normalized;
  RecordTimestampLocked(timestamp_ns);
}

// IMU C ABI没有sequence；只复用同一timestamp顺序锁，禁止ROS层生成无法审计的假序号。
void PublicationRateMetrics::RecordSourceWithoutSequence(uint64_t timestamp_ns,
                                                         uint64_t monotonic_ns) noexcept {
  SetFirstMonotonic(monotonic_ns);
  source_count_.fetch_add(1U, std::memory_order_relaxed);
  std::lock_guard<std::mutex> lock(source_mutex_);
  RecordTimestampLocked(timestamp_ns);
}

// ROS主topic成功发布后记录调用耗时；max通过CAS原子提升，热路径不引入mutex或分配。
void PublicationRateMetrics::RecordPublishSuccess(uint64_t latency_ns) noexcept {
  publish_count_.fetch_add(1U, std::memory_order_relaxed);
  publish_latency_count_.fetch_add(1U, std::memory_order_relaxed);
  publish_latency_sum_ns_.fetch_add(latency_ns, std::memory_order_relaxed);

  // 并发camera worker只允许增大最大值；CAS失败时复用已刷新expected，直到无需更新或成功。
  uint64_t expected = publish_latency_max_ns_.load(std::memory_order_relaxed);
  while (latency_ns > expected &&
         !publish_latency_max_ns_.compare_exchange_weak(
             expected, latency_ns, std::memory_order_relaxed, std::memory_order_relaxed)) {
  }
}

// CameraInfo和Temperature属于辅助topic，独立计数避免把主Image/IMU频率翻倍。
void PublicationRateMetrics::RecordAuxiliaryPublishSuccess() noexcept {
  auxiliary_publish_count_.fetch_add(1U, std::memory_order_relaxed);
}

void PublicationRateMetrics::RecordDrop() noexcept {
  drop_count_.fetch_add(1U, std::memory_order_relaxed);
}

void PublicationRateMetrics::RecordPublishFailure() noexcept {
  publish_failure_count_.fetch_add(1U, std::memory_order_relaxed);
}

// snapshot是无锁近似一致观测；各累计atomic可审计，分类器只比较区间delta，不要求全字段同一CPU指令时刻。
PublicationRateSnapshot PublicationRateMetrics::Snapshot(uint64_t monotonic_ns) const noexcept {
  PublicationRateSnapshot value;
  // acquire读取与首时间的release发布配对；普通计数虽用relaxed更新，仍保证每个字段自身单调且无数据竞争。
  value.source_count = source_count_.load(std::memory_order_acquire);
  value.publish_count = publish_count_.load(std::memory_order_acquire);
  value.auxiliary_publish_count =
      auxiliary_publish_count_.load(std::memory_order_acquire);
  value.drop_count = drop_count_.load(std::memory_order_acquire);
  value.publish_failure_count = publish_failure_count_.load(std::memory_order_acquire);
  value.sequence_gap_count = sequence_gap_count_.load(std::memory_order_acquire);
  value.sequence_duplicate_count =
      sequence_duplicate_count_.load(std::memory_order_acquire);
  value.sequence_regression_count =
      sequence_regression_count_.load(std::memory_order_acquire);
  value.timestamp_duplicate_count =
      timestamp_duplicate_count_.load(std::memory_order_acquire);
  value.timestamp_regression_count =
      timestamp_regression_count_.load(std::memory_order_acquire);
  value.publish_latency_count = publish_latency_count_.load(std::memory_order_acquire);
  value.publish_latency_sum_ns = publish_latency_sum_ns_.load(std::memory_order_acquire);
  value.publish_latency_max_ns = publish_latency_max_ns_.load(std::memory_order_acquire);
  value.interval_start_monotonic_ns =
      first_monotonic_ns_.load(std::memory_order_acquire);
  value.interval_end_monotonic_ns = monotonic_ns;
  return value;
}

// 把两个累计snapshot转换为区间数据；所有计数饱和相减，时间边界严格取before结束到after结束。
PublicationRateSnapshot PublicationRateMetrics::Delta(
    const PublicationRateSnapshot& before,
    const PublicationRateSnapshot& after) noexcept {
  PublicationRateSnapshot value;
  // 逐字段保留原始可审计语义，不合并sequence/timestamp异常，便于分类器定位具体producer合同。
  value.source_count = SaturatingSubtract(after.source_count, before.source_count);
  value.publish_count = SaturatingSubtract(after.publish_count, before.publish_count);
  value.auxiliary_publish_count =
      SaturatingSubtract(after.auxiliary_publish_count, before.auxiliary_publish_count);
  value.drop_count = SaturatingSubtract(after.drop_count, before.drop_count);
  value.publish_failure_count =
      SaturatingSubtract(after.publish_failure_count, before.publish_failure_count);
  value.sequence_gap_count =
      SaturatingSubtract(after.sequence_gap_count, before.sequence_gap_count);
  value.sequence_duplicate_count =
      SaturatingSubtract(after.sequence_duplicate_count, before.sequence_duplicate_count);
  value.sequence_regression_count =
      SaturatingSubtract(after.sequence_regression_count, before.sequence_regression_count);
  value.timestamp_duplicate_count =
      SaturatingSubtract(after.timestamp_duplicate_count, before.timestamp_duplicate_count);
  value.timestamp_regression_count =
      SaturatingSubtract(after.timestamp_regression_count, before.timestamp_regression_count);
  value.publish_latency_count =
      SaturatingSubtract(after.publish_latency_count, before.publish_latency_count);
  value.publish_latency_sum_ns =
      SaturatingSubtract(after.publish_latency_sum_ns, before.publish_latency_sum_ns);
  value.publish_latency_max_ns = after.publish_latency_max_ns;
  // 即使after早于before也保留实际时间值，后续窗口校验据此判INCONCLUSIVE而非伪造合法区间。
  value.interval_start_monotonic_ns = before.interval_end_monotonic_ns;
  value.interval_end_monotonic_ns = after.interval_end_monotonic_ns;
  return value;
}

}  // namespace robobaton_4p_ros2_demo
