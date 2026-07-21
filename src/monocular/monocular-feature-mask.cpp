/**
 * @file monocular-feature-mask.cpp
 * @brief 实现纯单目特征掩膜的文件加载、二值化、覆盖率统计和尺寸校验。
 */

#include "monocular-feature-mask.hpp"

#include <sstream>
#include <stdexcept>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

namespace orbslam3_ros2
{

/**
 * @brief 将单通道 8 位图像归一化为白色允许、黑色排除的二值掩膜。
 * @param raw_mask 待归一化的掩膜图像。
 * @return 仅包含 0 和 255 的 CV_8UC1 掩膜。
 * @throws std::invalid_argument 输入为空、类型错误或不包含允许区域时抛出。
 */
cv::Mat NormalizeFeatureMask(const cv::Mat& raw_mask)
{
    if (raw_mask.empty())
    {
        throw std::invalid_argument("特征掩膜图像为空");
    }
    if (raw_mask.type() != CV_8UC1)
    {
        throw std::invalid_argument("特征掩膜必须是 CV_8UC1 单通道 8 位图像");
    }

    /** @brief 归一化后的二值掩膜，非零输入统一转换为 255。 */
    cv::Mat binary_mask;
    cv::threshold(raw_mask, binary_mask, 0, 255, cv::THRESH_BINARY);
    if (cv::countNonZero(binary_mask) == 0)
    {
        throw std::invalid_argument("特征掩膜不能为全黑，至少需要保留一个允许像素");
    }
    return binary_mask;
}

/**
 * @brief 从 PNG 等图像文件加载特征掩膜。
 * @param path 掩膜文件路径；空路径表示禁用掩膜。
 * @return 二值掩膜；禁用时返回空矩阵。
 * @throws std::runtime_error 非空路径无法读取时抛出。
 * @throws std::invalid_argument 图像内容无效时抛出。
 */
cv::Mat LoadFeatureMask(const std::string& path)
{
    if (path.empty())
    {
        return cv::Mat();
    }

    /** @brief 以灰度方式读取的原始掩膜。 */
    const cv::Mat raw_mask = cv::imread(path, cv::IMREAD_GRAYSCALE);
    if (raw_mask.empty())
    {
        throw std::runtime_error("无法读取特征掩膜文件: " + path);
    }
    return NormalizeFeatureMask(raw_mask);
}

/**
 * @brief 使用二值掩膜清除输入图像中的排除区域。
 * @param image 待处理的单通道或多通道输入图像。
 * @param mask 白色允许、黑色排除的 CV_8UC1 二值掩膜；空矩阵表示禁用。
 * @return 应用掩膜后的独立图像；掩膜禁用时返回输入图像的浅拷贝。
 * @throws std::invalid_argument 掩膜类型或尺寸无效时抛出。
 */
cv::Mat ApplyFeatureMask(const cv::Mat& image, const cv::Mat& mask)
{
    if (mask.empty())
    {
        return image;
    }
    if (mask.type() != CV_8UC1)
    {
        throw std::invalid_argument("特征掩膜必须是 CV_8UC1 单通道 8 位图像");
    }
    if (mask.size() != image.size())
    {
        throw std::invalid_argument("特征掩膜尺寸必须与输入图像一致");
    }

    /** @brief 排除区域清零后的跟踪图像。 */
    cv::Mat masked_image = cv::Mat::zeros(image.size(), image.type());
    image.copyTo(masked_image, mask);
    return masked_image;
}

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
    std::string* error_message)
{
    if (mask.empty() || mask.size() == image_size)
    {
        return true;
    }

    if (error_message != nullptr)
    {
        /** @brief 包含实际尺寸和期望尺寸的错误信息。 */
        std::ostringstream stream;
        stream << "特征掩膜尺寸 " << mask.cols << "x" << mask.rows
               << " 与输入图像尺寸 " << image_size.width << "x" << image_size.height
               << " 不一致";
        *error_message = stream.str();
    }
    return false;
}

/**
 * @brief 计算掩膜排除像素占全部像素的比例。
 * @param mask 已二值化掩膜；空矩阵表示未排除任何区域。
 * @return 范围为 [0, 1] 的排除比例。
 */
double CalculateExcludedRatio(const cv::Mat& mask)
{
    if (mask.empty())
    {
        return 0.0;
    }
    if (mask.type() != CV_8UC1)
    {
        throw std::invalid_argument("计算排除比例前必须提供 CV_8UC1 特征掩膜");
    }

    /** @brief 掩膜总像素数。 */
    const double total_pixels = static_cast<double>(mask.total());
    /** @brief 允许提取特征的白色像素数。 */
    const double allowed_pixels = static_cast<double>(cv::countNonZero(mask));
    return (total_pixels - allowed_pixels) / total_pixels;
}

}  // namespace orbslam3_ros2
