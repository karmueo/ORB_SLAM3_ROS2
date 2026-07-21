/**
 * @file monocular-feature-mask.hpp
 * @brief 声明纯单目特征掩膜的加载、二值化和尺寸校验接口。
 */

#ifndef ORBSLAM3_ROS2_MONOCULAR_FEATURE_MASK_HPP_
#define ORBSLAM3_ROS2_MONOCULAR_FEATURE_MASK_HPP_

#include <string>

#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>

namespace orbslam3_ros2
{

/**
 * @brief 将单通道 8 位图像归一化为白色允许、黑色排除的二值掩膜。
 * @param raw_mask 待归一化的掩膜图像。
 * @return 仅包含 0 和 255 的 CV_8UC1 掩膜。
 * @throws std::invalid_argument 输入为空、类型错误或不包含允许区域时抛出。
 */
cv::Mat NormalizeFeatureMask(const cv::Mat& raw_mask);

/**
 * @brief 从 PNG 等图像文件加载特征掩膜。
 * @param path 掩膜文件路径；空路径表示禁用掩膜。
 * @return 二值掩膜；禁用时返回空矩阵。
 * @throws std::runtime_error 非空路径无法读取时抛出。
 * @throws std::invalid_argument 图像内容无效时抛出。
 */
cv::Mat LoadFeatureMask(const std::string& path);

/**
 * @brief 使用二值掩膜清除输入图像中的排除区域。
 * @param image 待处理的单通道或多通道输入图像。
 * @param mask 白色允许、黑色排除的 CV_8UC1 二值掩膜；空矩阵表示禁用。
 * @return 应用掩膜后的独立图像；掩膜禁用时返回输入图像的浅拷贝。
 * @throws std::invalid_argument 掩膜类型或尺寸无效时抛出。
 */
cv::Mat ApplyFeatureMask(const cv::Mat& image, const cv::Mat& mask);

/**
 * @brief 检查掩膜尺寸是否与算法缩放前的输入图像一致。
 * @param mask 待检查掩膜；空矩阵表示禁用并视为合法。
 * @param image_size 输入图像尺寸。
 * @param error_message 校验失败时接收中文错误说明，可为空。
 * @return 尺寸一致或掩膜禁用时返回 true。
 */
bool ValidateFeatureMaskSize(
    const cv::Mat& mask,
    const cv::Size& image_size,
    std::string* error_message);

/**
 * @brief 计算掩膜排除像素占全部像素的比例。
 * @param mask 已二值化掩膜；空矩阵表示未排除任何区域。
 * @return 范围为 [0, 1] 的排除比例。
 */
double CalculateExcludedRatio(const cv::Mat& mask);

}  // namespace orbslam3_ros2

#endif  // ORBSLAM3_ROS2_MONOCULAR_FEATURE_MASK_HPP_
