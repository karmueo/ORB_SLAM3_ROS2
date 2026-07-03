/**
 * @file rgbd-raw-data-converter-node.cpp
 * @brief 提供订阅 packed RGB-D raw 图像并发布标准 RGB-D 图像的 ROS 2 节点入口。
 */

#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

#include "rgbd-raw-data-converter.hpp"

/**
 * @brief 将 packed RGB-D raw 图像拆分发布为标准彩色图像和深度图像的节点。
 */
class RgbdRawDataConverterNode : public rclcpp::Node
{
public:
  /**
   * @brief 构造 RGB-D 原始数据转换节点并创建订阅与发布器。
   */
  RgbdRawDataConverterNode()
  : Node("rgbd_raw_data_converter")
  {
    this->declare_parameter<std::string>("rgb_encoding", "rgb8");

    rgb_encoding_ = this->get_parameter("rgb_encoding").as_string();
    if (rgb_encoding_ != "rgb8" && rgb_encoding_ != "bgr8") {
      RCLCPP_WARN(
        this->get_logger(),
        "Unsupported rgb_encoding '%s', fallback to 'rgb8'",
        rgb_encoding_.c_str());
      rgb_encoding_ = "rgb8";
    }

    rgb_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("camera/rgb", 10);
    depth_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("camera/depth", 10);
    raw_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
      "rgbd/raw/image",
      10,
      std::bind(&RgbdRawDataConverterNode::HandleRawImage, this, std::placeholders::_1));
  }

private:
  /**
   * @brief 处理 packed RGB-D raw 图像，转换成功后发布两路标准图像。
   * @param message packed RGB-D raw 图像消息。
   * @return 无返回值；校验失败时丢弃当前帧并输出 warning。
   */
  void HandleRawImage(const sensor_msgs::msg::Image::SharedPtr message)
  {
    /** @brief 转换后的彩色图像消息。 */
    sensor_msgs::msg::Image rgb_image;
    /** @brief 转换后的深度图像消息。 */
    sensor_msgs::msg::Image depth_image;

    if (!BuildRgbdImages(
        *message,
        rgb_encoding_,
        rgb_image,
        depth_image)) {
      RCLCPP_WARN(
        this->get_logger(),
        "Drop invalid packed RGB-D frame: width=%u height=%u step=%u data_size=%zu encoding=%s",
        message->width,
        message->height,
        message->step,
        message->data.size(),
        rgb_encoding_.c_str());
      return;
    }

    rgb_publisher_->publish(rgb_image);
    depth_publisher_->publish(depth_image);
  }

  /** @brief 彩色图像输出发布器。 */
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr rgb_publisher_;
  /** @brief 深度图像输出发布器。 */
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_publisher_;
  /** @brief packed RGB-D raw 图像订阅器。 */
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr raw_subscription_;
  /** @brief 彩色图像输出编码，支持 rgb8 或 bgr8。 */
  std::string rgb_encoding_;
};

/**
 * @brief 启动 RGB-D 原始数据转换节点。
 * @param argc 命令行参数数量。
 * @param argv 命令行参数数组。
 * @return 进程退出码，正常退出返回 0。
 */
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  /** @brief RGB-D 原始数据转换节点实例。 */
  const auto node = std::make_shared<RgbdRawDataConverterNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
