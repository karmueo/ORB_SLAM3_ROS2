/**
 * @file test_monocular_feature_mask.cpp
 * @brief 测试纯单目静态特征掩膜的加载、应用、二值化、覆盖率和尺寸校验。
 */

#include <stdexcept>
#include <string>

#include <gtest/gtest.h>
#include <opencv2/core/mat.hpp>

#include "monocular-feature-mask.hpp"

namespace
{

/**
 * @brief 验证空路径按禁用掩膜处理。
 */
TEST(MonocularFeatureMaskTest, EmptyPathDisablesMask)
{
    /** @brief 空路径加载结果。 */
    const cv::Mat mask = orbslam3_ros2::LoadFeatureMask("");
    EXPECT_TRUE(mask.empty());
    EXPECT_DOUBLE_EQ(orbslam3_ros2::CalculateExcludedRatio(mask), 0.0);
}

/**
 * @brief 验证包内全白模板可读取且不排除任何像素。
 */
TEST(MonocularFeatureMaskTest, LoadsInstalledTemplateConvention)
{
    /** @brief 源码目录内的默认掩膜模板路径。 */
    const std::string path =
        std::string(ORBSLAM3_TEST_SOURCE_DIR) +
        "/config/masks/XV_RGB_Fisheye_gripper_mask.png";
    /** @brief 从默认模板加载的掩膜。 */
    const cv::Mat mask = orbslam3_ros2::LoadFeatureMask(path);

    EXPECT_EQ(mask.type(), CV_8UC1);
    EXPECT_EQ(mask.cols, 1280);
    EXPECT_EQ(mask.rows, 1280);
    EXPECT_DOUBLE_EQ(orbslam3_ros2::CalculateExcludedRatio(mask), 0.0);
}

/**
 * @brief 验证所有非零灰度值都会归一化为允许像素。
 */
TEST(MonocularFeatureMaskTest, NormalizesEveryNonZeroPixelToWhite)
{
    /** @brief 包含黑色和多种非零灰度值的测试图像。 */
    cv::Mat raw_mask = (cv::Mat_<unsigned char>(2, 2) << 0, 1, 127, 255);
    /** @brief 归一化后的二值掩膜。 */
    const cv::Mat mask = orbslam3_ros2::NormalizeFeatureMask(raw_mask);

    EXPECT_EQ(mask.at<unsigned char>(0, 0), 0);
    EXPECT_EQ(mask.at<unsigned char>(0, 1), 255);
    EXPECT_EQ(mask.at<unsigned char>(1, 0), 255);
    EXPECT_EQ(mask.at<unsigned char>(1, 1), 255);
    EXPECT_DOUBLE_EQ(orbslam3_ros2::CalculateExcludedRatio(mask), 0.25);
}

/**
 * @brief 验证掩膜会清除黑色区域并保留白色区域的图像内容。
 */
TEST(MonocularFeatureMaskTest, ClearsExcludedImageRegion)
{
    /** @brief 包含四个不同灰度值的输入图像。 */
    cv::Mat image = (cv::Mat_<unsigned char>(2, 2) << 10, 20, 30, 40);
    /** @brief 仅允许对角像素参与跟踪的二值掩膜。 */
    cv::Mat mask = (cv::Mat_<unsigned char>(2, 2) << 255, 0, 0, 255);
    /** @brief 应用掩膜后的跟踪图像。 */
    const cv::Mat masked_image = orbslam3_ros2::ApplyFeatureMask(image, mask);

    EXPECT_EQ(masked_image.at<unsigned char>(0, 0), 10);
    EXPECT_EQ(masked_image.at<unsigned char>(0, 1), 0);
    EXPECT_EQ(masked_image.at<unsigned char>(1, 0), 0);
    EXPECT_EQ(masked_image.at<unsigned char>(1, 1), 40);
}

/**
 * @brief 验证无效文件、类型和全黑内容会被拒绝。
 */
TEST(MonocularFeatureMaskTest, RejectsInvalidMasks)
{
    EXPECT_THROW(
        orbslam3_ros2::LoadFeatureMask("/tmp/orbslam3_missing_feature_mask.png"),
        std::runtime_error);

    /** @brief 类型错误的三通道掩膜。 */
    const cv::Mat color_mask(8, 8, CV_8UC3, cv::Scalar(255, 255, 255));
    EXPECT_THROW(
        orbslam3_ros2::NormalizeFeatureMask(color_mask),
        std::invalid_argument);

    /** @brief 不包含任何允许区域的全黑掩膜。 */
    const cv::Mat black_mask = cv::Mat::zeros(8, 8, CV_8UC1);
    EXPECT_THROW(
        orbslam3_ros2::NormalizeFeatureMask(black_mask),
        std::invalid_argument);
}

/**
 * @brief 验证首帧尺寸校验给出明确结果和错误说明。
 */
TEST(MonocularFeatureMaskTest, ValidatesInputImageSize)
{
    /** @brief 模拟原始相机分辨率的掩膜。 */
    const cv::Mat mask(1280, 1280, CV_8UC1, cv::Scalar(255));
    /** @brief 接收尺寸不匹配说明的字符串。 */
    std::string error_message;

    EXPECT_TRUE(orbslam3_ros2::ValidateFeatureMaskSize(
        mask, cv::Size(1280, 1280), &error_message));
    EXPECT_FALSE(orbslam3_ros2::ValidateFeatureMaskSize(
        mask, cv::Size(960, 960), &error_message));
    EXPECT_NE(error_message.find("1280x1280"), std::string::npos);
    EXPECT_NE(error_message.find("960x960"), std::string::npos);
}

}  // namespace
