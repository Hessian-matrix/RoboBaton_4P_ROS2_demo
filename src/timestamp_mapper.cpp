#include "robobaton_4p_ros2_demo/timestamp_mapper.hpp"

#include <limits>
#include <stdexcept>

namespace robobaton_4p_ros2_demo {

RosStampParts ToRosStampParts(uint64_t timestamp_ns) {
  constexpr uint64_t kNanosecondsPerSecond = 1000000000ULL;
  const uint64_t seconds = timestamp_ns / kNanosecondsPerSecond;
  if (seconds > static_cast<uint64_t>(std::numeric_limits<int32_t>::max())) {
    throw std::overflow_error("ROS timestamp seconds exceed int32 range");
  }
  RosStampParts parts;
  parts.sec = static_cast<int32_t>(seconds);
  parts.nanosec = static_cast<uint32_t>(timestamp_ns % kNanosecondsPerSecond);
  return parts;
}

}  // namespace robobaton_4p_ros2_demo
