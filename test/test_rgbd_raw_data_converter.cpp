/**
 * @file test_rgbd_raw_data_converter.cpp
 * @brief 测试 packed RGB-D raw 图像转换为标准 Image 消息的核心逻辑。
 */

#include "rgbd-raw-data-converter.hpp"

#include <cstring>
#include <vector>

#include "gtest/gtest.h"

namespace
{
/**
 * @brief 从图像字节数组中恢复 float 深度值。
 * @param data 图像消息中的字节数据。
 * @return 按 float32 解释后的深度数组。
 */
std::vector<float> BytesToFloats(const std::vector<uint8_t>& data)
{
  /** @brief 恢复后的 float 深度数组。 */
  std::vector<float> values(data.size() / sizeof(float));
  std::memcpy(values.data(), data.data(), data.size());
  return values;
}

/**
 * @brief 将 float 深度值追加到 packed raw 字节数组。
 * @param value 需要写入的米单位 float32 深度。
 * @param data 输出的 packed raw 字节数组。
 * @return 无返回值；函数会向 data 末尾追加 4 字节。
 */
void AppendFloat(float value, std::vector<uint8_t>& data)
{
  /** @brief 深度值的字节视图。 */
  const auto* bytes = reinterpret_cast<const uint8_t*>(&value);
  data.insert(data.end(), bytes, bytes + sizeof(float));
}
}  // namespace

TEST(RgbdRawDataConverter, ConvertsPackedRawImageToRgbAndDepthImages)
{
  /** @brief 输入消息头，用于验证输出继承时间戳和坐标系。 */
  sensor_msgs::msg::Image raw_image;
  raw_image.header.frame_id = "rgbd_frame";
  raw_image.header.stamp.sec = 12;
  raw_image.header.stamp.nanosec = 34;
  raw_image.height = 1U;
  raw_image.width = 2U;
  raw_image.step = 14U;
  raw_image.data = {1U, 2U, 3U};
  AppendFloat(1.25F, raw_image.data);
  raw_image.data.insert(raw_image.data.end(), {4U, 5U, 6U});
  AppendFloat(2.5F, raw_image.data);

  /** @brief 转换后的彩色图像消息。 */
  sensor_msgs::msg::Image rgb_image;
  /** @brief 转换后的深度图像消息。 */
  sensor_msgs::msg::Image depth_image;

  EXPECT_TRUE(BuildRgbdImages(raw_image, "rgb8", rgb_image, depth_image));

  EXPECT_EQ(rgb_image.header.frame_id, "rgbd_frame");
  EXPECT_EQ(rgb_image.header.stamp.sec, 12);
  EXPECT_EQ(rgb_image.header.stamp.nanosec, 34U);
  EXPECT_EQ(rgb_image.height, 1U);
  EXPECT_EQ(rgb_image.width, 2U);
  EXPECT_EQ(rgb_image.encoding, "rgb8");
  EXPECT_EQ(rgb_image.step, 6U);
  EXPECT_EQ(rgb_image.data, (std::vector<uint8_t>{1U, 2U, 3U, 4U, 5U, 6U}));
  EXPECT_EQ(depth_image.header.frame_id, "rgbd_frame");
  EXPECT_EQ(depth_image.height, 1U);
  EXPECT_EQ(depth_image.width, 2U);
  EXPECT_EQ(depth_image.encoding, "32FC1");
  EXPECT_EQ(depth_image.step, 8U);
  EXPECT_EQ(BytesToFloats(depth_image.data), (std::vector<float>{1.25F, 2.5F}));
}

TEST(RgbdRawDataConverter, SwapsRedAndBlueForBgr8)
{
  /** @brief 单像素 packed raw 图像。 */
  sensor_msgs::msg::Image raw_image;
  raw_image.height = 1U;
  raw_image.width = 1U;
  raw_image.step = 7U;
  raw_image.data = {10U, 20U, 30U};
  AppendFloat(0.5F, raw_image.data);

  /** @brief 转换后的彩色图像消息。 */
  sensor_msgs::msg::Image rgb_image;
  /** @brief 转换后的深度图像消息。 */
  sensor_msgs::msg::Image depth_image;

  EXPECT_TRUE(BuildRgbdImages(raw_image, "bgr8", rgb_image, depth_image));

  /** @brief 预期的 BGR 像素数据。 */
  const std::vector<uint8_t> expected_bgr = {30U, 20U, 10U};
  EXPECT_EQ(rgb_image.encoding, "bgr8");
  EXPECT_EQ(rgb_image.data, expected_bgr);
}

TEST(RgbdRawDataConverter, RejectsInvalidStep)
{
  /** @brief step 与 width * 7 不一致的 packed raw 图像。 */
  sensor_msgs::msg::Image raw_image;
  raw_image.height = 1U;
  raw_image.width = 1U;
  raw_image.step = 8U;
  raw_image.data.resize(8U);

  /** @brief 转换后的彩色图像消息。 */
  sensor_msgs::msg::Image rgb_image;
  /** @brief 转换后的深度图像消息。 */
  sensor_msgs::msg::Image depth_image;

  EXPECT_FALSE(BuildRgbdImages(raw_image, "rgb8", rgb_image, depth_image));
}

TEST(RgbdRawDataConverter, RejectsInvalidInput)
{
  /** @brief 基准 packed raw 图像，后续测试会复制并破坏部分字段。 */
  sensor_msgs::msg::Image raw_image;
  raw_image.height = 1U;
  raw_image.width = 1U;
  raw_image.step = 7U;
  raw_image.data = {1U, 2U, 3U};
  AppendFloat(1.0F, raw_image.data);

  /** @brief 转换后的彩色图像消息。 */
  sensor_msgs::msg::Image rgb_image;
  /** @brief 转换后的深度图像消息。 */
  sensor_msgs::msg::Image depth_image;

  /** @brief 高度为空的非法图像。 */
  auto empty_height_image = raw_image;
  empty_height_image.height = 0U;
  EXPECT_FALSE(BuildRgbdImages(empty_height_image, "rgb8", rgb_image, depth_image));

  /** @brief 宽度为空的非法图像。 */
  auto empty_width_image = raw_image;
  empty_width_image.width = 0U;
  EXPECT_FALSE(BuildRgbdImages(empty_width_image, "rgb8", rgb_image, depth_image));

  /** @brief 数据长度不足一整行的非法图像。 */
  auto short_data_image = raw_image;
  short_data_image.data.pop_back();
  EXPECT_FALSE(BuildRgbdImages(short_data_image, "rgb8", rgb_image, depth_image));

  EXPECT_FALSE(BuildRgbdImages(raw_image, "mono8", rgb_image, depth_image));
}
