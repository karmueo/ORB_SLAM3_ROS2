/**
 * @file rgbd-raw-data-converter.cpp
 * @brief 实现 packed RGB-D raw 图像到标准 ROS 图像消息的转换逻辑。
 */

#include "rgbd-raw-data-converter.hpp"

#include <cstring>
#include <limits>

bool BuildRgbdImages(
  const sensor_msgs::msg::Image& raw_image,
  const std::string& rgb_encoding,
  sensor_msgs::msg::Image& rgb_image,
  sensor_msgs::msg::Image& depth_image)
{
  /** @brief packed raw 图像中每个像素的 RGB 字节数。 */
  const size_t rgb_channels = 3U;
  /** @brief packed raw 图像中每个像素的深度字节数。 */
  const size_t depth_bytes = sizeof(float);
  /** @brief packed raw 图像中每个像素的总字节数。 */
  const size_t packed_pixel_bytes = rgb_channels + depth_bytes;

  if (raw_image.height == 0U || raw_image.width == 0U) {
    return false;
  }

  if (rgb_encoding != "rgb8" && rgb_encoding != "bgr8") {
    return false;
  }

  /** @brief 输入图像每行期望字节数。 */
  const size_t expected_raw_step = static_cast<size_t>(raw_image.width) * packed_pixel_bytes;
  if (expected_raw_step > std::numeric_limits<uint32_t>::max()) {
    return false;
  }

  if (raw_image.step != static_cast<uint32_t>(expected_raw_step)) {
    return false;
  }

  /** @brief 输入图像期望最小字节数。 */
  const size_t expected_raw_size = static_cast<size_t>(raw_image.height) * expected_raw_step;
  if (expected_raw_step != 0U &&
      expected_raw_size / expected_raw_step != static_cast<size_t>(raw_image.height)) {
    return false;
  }

  if (raw_image.data.size() < expected_raw_size) {
    return false;
  }

  /** @brief 图像像素总数。 */
  const size_t pixel_count = static_cast<size_t>(raw_image.width) * static_cast<size_t>(raw_image.height);
  if (pixel_count / static_cast<size_t>(raw_image.width) != static_cast<size_t>(raw_image.height)) {
    return false;
  }

  if (pixel_count > std::numeric_limits<size_t>::max() / rgb_channels ||
      pixel_count > std::numeric_limits<size_t>::max() / depth_bytes) {
    return false;
  }

  /** @brief 构造中的彩色图像，验证通过后再赋值给输出参数。 */
  sensor_msgs::msg::Image next_rgb_image;
  next_rgb_image.header = raw_image.header;
  next_rgb_image.height = raw_image.height;
  next_rgb_image.width = raw_image.width;
  next_rgb_image.encoding = rgb_encoding;
  next_rgb_image.is_bigendian = 0U;
  next_rgb_image.step = raw_image.width * static_cast<uint32_t>(rgb_channels);
  next_rgb_image.data.resize(pixel_count * rgb_channels);

  /** @brief 构造中的深度图像，验证通过后再赋值给输出参数。 */
  sensor_msgs::msg::Image next_depth_image;
  next_depth_image.header = raw_image.header;
  next_depth_image.height = raw_image.height;
  next_depth_image.width = raw_image.width;
  next_depth_image.encoding = "32FC1";
  next_depth_image.is_bigendian = 0U;
  next_depth_image.step = raw_image.width * static_cast<uint32_t>(depth_bytes);
  next_depth_image.data.resize(pixel_count * sizeof(float));

  for (uint32_t row = 0U; row < raw_image.height; ++row) {
    for (uint32_t column = 0U; column < raw_image.width; ++column) {
      /** @brief 当前像素在 packed raw 输入中的起始下标。 */
      const size_t raw_offset =
        static_cast<size_t>(row) * raw_image.step +
        static_cast<size_t>(column) * packed_pixel_bytes;
      /** @brief 当前像素在彩色输出图像中的起始下标。 */
      const size_t rgb_offset =
        (static_cast<size_t>(row) * raw_image.width + static_cast<size_t>(column)) * rgb_channels;
      /** @brief 当前像素在深度输出图像中的起始下标。 */
      const size_t depth_offset =
        (static_cast<size_t>(row) * raw_image.width + static_cast<size_t>(column)) * depth_bytes;

      if (rgb_encoding == "bgr8") {
        next_rgb_image.data[rgb_offset] = raw_image.data[raw_offset + 2U];
        next_rgb_image.data[rgb_offset + 1U] = raw_image.data[raw_offset + 1U];
        next_rgb_image.data[rgb_offset + 2U] = raw_image.data[raw_offset];
      } else {
        std::memcpy(&next_rgb_image.data[rgb_offset], &raw_image.data[raw_offset], rgb_channels);
      }

      std::memcpy(
        &next_depth_image.data[depth_offset],
        &raw_image.data[raw_offset + rgb_channels],
        depth_bytes);
    }
  }

  rgb_image = next_rgb_image;
  depth_image = next_depth_image;
  return true;
}
