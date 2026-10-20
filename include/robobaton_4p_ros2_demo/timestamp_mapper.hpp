#pragma once

#include <cstdint>

namespace robobaton_4p_ros2_demo {

struct RosStampParts {
  int32_t sec = 0;
  uint32_t nanosec = 0U;
};

// Splits nanoseconds without changing the input clock domain or epoch.
RosStampParts ToRosStampParts(uint64_t timestamp_ns);

template <typename RosTimeMessage>
void AssignStamp(RosTimeMessage& stamp, const RosStampParts& parts) noexcept {
  stamp.sec = parts.sec;
  stamp.nanosec = parts.nanosec;
}

}  // namespace robobaton_4p_ros2_demo
