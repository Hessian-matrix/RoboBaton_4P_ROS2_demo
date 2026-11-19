#ifndef ROBOBATON_4P_ROS2_DEMO_X5_JPEG_OUTPUT_HPP_
#define ROBOBATON_4P_ROS2_DEMO_X5_JPEG_OUTPUT_HPP_

#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <vector>

namespace robobaton_4p_ros2_demo {
namespace detail {

inline std::size_t FindJpegEnd(const unsigned char* data, std::size_t capacity) {
  if (data == nullptr || capacity < 4U || data[0] != 0xFFU || data[1] != 0xD8U) {
    throw std::runtime_error("X5 JPEG output is missing SOI marker");
  }
  for (std::size_t index = capacity; index >= 2U; --index) {
    if (data[index - 2U] == 0xFFU && data[index - 1U] == 0xD9U) {
      return index;
    }
  }
  throw std::runtime_error("X5 JPEG output is missing EOI marker");
}

inline std::vector<uint8_t> CopyJpegForInterchange(const unsigned char* data,
                                                std::size_t capacity) {
  const std::size_t size = FindJpegEnd(data, capacity);
  // CodaJ12 JpgEncEncodeHeader emits APP9(length=4, frameIdx[15:0]) directly
  // after SOI. It has no application identifier and is private encoder state,
  // not image metadata. T.81 B.2.4.6 recommends removing private APP segments
  // across application environments; FFmpeg's APP parser requires length >= 6.
  // Other APP segments, all coding tables and the entropy stream stay intact.
  const bool has_frame_index = size >= 10U && data[2] == 0xFFU &&
      data[3] == 0xE9U && data[4] == 0U && data[5] == 4U && data[8] == 0xFFU;
  if (!has_frame_index) {
    return std::vector<uint8_t>(data, data + size);
  }
  std::vector<uint8_t> jpeg;
  jpeg.reserve(size - 6U);
  jpeg.push_back(0xFFU);
  jpeg.push_back(0xD8U);
  jpeg.insert(jpeg.end(), data + 8U, data + size);
  return jpeg;
}

}  // namespace detail
}  // namespace robobaton_4p_ros2_demo

#endif  // ROBOBATON_4P_ROS2_DEMO_X5_JPEG_OUTPUT_HPP_
