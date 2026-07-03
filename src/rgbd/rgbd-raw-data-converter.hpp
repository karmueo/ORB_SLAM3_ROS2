/**
 * @file rgbd-raw-data-converter.hpp
 * @brief 声明 packed RGB-D raw 图像转换为标准 ROS 图像消息的公共接口。
 */

#ifndef RGBD_RAW_DATA_CONVERTER_HPP_
#define RGBD_RAW_DATA_CONVERTER_HPP_

#include <string>

#include "sensor_msgs/msg/image.hpp"

/**
 * @brief 将每像素 RGB + float32 depth 的 packed raw 图像转换为两路 Image 消息。
 * @param raw_image 输入 packed raw 图像，每像素格式为 RGB 三字节加米单位 float32 深度。
 * @param rgb_encoding 彩色图像输出编码，支持 rgb8 或 bgr8。
 * @param rgb_image 输出的彩色 Image 消息。
 * @param depth_image 输出的 32FC1 深度 Image 消息。
 * @return 输入尺寸、step、数据长度和编码合法时返回 true，否则返回 false 且不更新输出消息。
 */
bool BuildRgbdImages(
  const sensor_msgs::msg::Image& raw_image,
  const std::string& rgb_encoding,
  sensor_msgs::msg::Image& rgb_image,
  sensor_msgs::msg::Image& depth_image);

#endif  // RGBD_RAW_DATA_CONVERTER_HPP_
