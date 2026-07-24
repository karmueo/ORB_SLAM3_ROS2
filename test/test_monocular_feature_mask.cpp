/**
 * @file test_monocular_feature_mask.cpp
 * @brief 测试单目特征掩膜工具及 ORB-SLAM3 候选特征过滤行为。
 */

#include <stdexcept>
#include <string>

#include <gtest/gtest.h>
#include <opencv2/core.hpp>

#include "ORBextractor.h"

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
 * @brief 验证包内默认夹爪掩膜可读取且包含允许区和排除区。
 */
TEST(MonocularFeatureMaskTest, LoadsInstalledTemplateConvention)
{
    /** @brief 源码目录内的默认掩膜模板路径。 */
    const std::string path =
        std::string(ORBSLAM3_TEST_SOURCE_DIR) +
        "/config/masks/mask.png";
    /** @brief 从默认模板加载的掩膜。 */
    const cv::Mat mask = orbslam3_ros2::LoadFeatureMask(path);

    EXPECT_EQ(mask.type(), CV_8UC1);
    EXPECT_EQ(mask.cols, 1280);
    EXPECT_EQ(mask.rows, 1280);
    EXPECT_GT(orbslam3_ros2::CalculateExcludedRatio(mask), 0.0);
    EXPECT_LT(orbslam3_ros2::CalculateExcludedRatio(mask), 1.0);
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

/**
 * @brief 验证原生 ORB mask 会排除黑区，同时保留白区边界附近的关键点。
 */
TEST(MonocularFeatureMaskTest, NativeOrbMaskRejectsOnlyBlackPixels)
{
    /** @brief 合成强纹理灰度图像宽度。 */
    constexpr int image_width = 320;
    /** @brief 合成强纹理灰度图像高度。 */
    constexpr int image_height = 240;
    /** @brief 白色允许区域与黑色排除区域的分界横坐标。 */
    constexpr int mask_boundary_x = 160;
    /** @brief 用于确认未对允许区域额外执行腐蚀的边界检查宽度。 */
    constexpr int boundary_check_width = 19;

    /** @brief 用于生成可重复强纹理图像的随机数生成器。 */
    cv::RNG random_generator(12345U);
    /** @brief 在允许区和排除区均包含大量候选角点的灰度图像。 */
    cv::Mat image(image_height, image_width, CV_8UC1);
    random_generator.fill(image, cv::RNG::UNIFORM, 0, 256);

    /** @brief 左半区允许、右半区排除的二值掩膜。 */
    cv::Mat mask(image_height, image_width, CV_8UC1, cv::Scalar(255));
    mask.colRange(mask_boundary_x, image_width).setTo(cv::Scalar(0));

    /** @brief 使用常规单目参数构造的 ORB-SLAM3 特征提取器。 */
    ORB_SLAM3::ORBextractor extractor(1000, 1.2F, 8, 20, 7);
    /** @brief 原生 mask 过滤后保留下来的关键点。 */
    std::vector<cv::KeyPoint> keypoints;
    /** @brief 与保留关键点逐行对应的 ORB 描述子。 */
    cv::Mat descriptors;
    /** @brief 单目模式下使用的重叠区域占位范围。 */
    std::vector<int> lapping_area{0, 0};

    extractor(image, mask, keypoints, descriptors, lapping_area);

    ASSERT_FALSE(keypoints.empty());
    ASSERT_EQ(descriptors.rows, static_cast<int>(keypoints.size()));
    /** @brief 是否存在位于白区边界检查范围内的保留关键点。 */
    bool has_allowed_boundary_keypoint = false;
    for (const cv::KeyPoint& keypoint : keypoints)
    {
        /** @brief 映射回原图的关键点横坐标。 */
        const int keypoint_x = cvRound(keypoint.pt.x);
        /** @brief 映射回原图的关键点纵坐标。 */
        const int keypoint_y = cvRound(keypoint.pt.y);
        EXPECT_NE(mask.at<unsigned char>(keypoint_y, keypoint_x), 0);
        EXPECT_LT(keypoint.pt.x, static_cast<float>(mask_boundary_x));
        if (keypoint.pt.x >=
            static_cast<float>(mask_boundary_x - boundary_check_width))
        {
            has_allowed_boundary_keypoint = true;
        }
    }
    EXPECT_TRUE(has_allowed_boundary_keypoint);
}

/**
 * @brief 验证空 mask 与全白 mask 均保持原有 ORB 特征提取结果。
 */
TEST(MonocularFeatureMaskTest, EmptyMaskPreservesNativeOrbExtraction)
{
    /** @brief 合成强纹理灰度图像宽度。 */
    constexpr int image_width = 320;
    /** @brief 合成强纹理灰度图像高度。 */
    constexpr int image_height = 240;

    /** @brief 用于生成可重复强纹理图像的随机数生成器。 */
    cv::RNG random_generator(67890U);
    /** @brief 用于比较禁用 mask 和全白 mask 行为的灰度图像。 */
    cv::Mat image(image_height, image_width, CV_8UC1);
    random_generator.fill(image, cv::RNG::UNIFORM, 0, 256);
    /** @brief 不排除任何像素的全白 mask。 */
    const cv::Mat white_mask(
        image_height, image_width, CV_8UC1, cv::Scalar(255));
    /** @brief 单目模式下使用的重叠区域占位范围。 */
    std::vector<int> lapping_area{0, 0};

    /** @brief 禁用 mask 时使用的 ORB-SLAM3 特征提取器。 */
    ORB_SLAM3::ORBextractor unmasked_extractor(1000, 1.2F, 8, 20, 7);
    /** @brief 禁用 mask 时得到的关键点。 */
    std::vector<cv::KeyPoint> unmasked_keypoints;
    /** @brief 禁用 mask 时得到的描述子。 */
    cv::Mat unmasked_descriptors;
    unmasked_extractor(
        image,
        cv::Mat(),
        unmasked_keypoints,
        unmasked_descriptors,
        lapping_area);

    /** @brief 使用全白 mask 时使用的 ORB-SLAM3 特征提取器。 */
    ORB_SLAM3::ORBextractor white_mask_extractor(1000, 1.2F, 8, 20, 7);
    /** @brief 使用全白 mask 时得到的关键点。 */
    std::vector<cv::KeyPoint> white_mask_keypoints;
    /** @brief 使用全白 mask 时得到的描述子。 */
    cv::Mat white_mask_descriptors;
    white_mask_extractor(
        image,
        white_mask,
        white_mask_keypoints,
        white_mask_descriptors,
        lapping_area);

    ASSERT_EQ(unmasked_keypoints.size(), white_mask_keypoints.size());
    ASSERT_EQ(unmasked_descriptors.size(), white_mask_descriptors.size());
    EXPECT_EQ(
        cv::countNonZero(unmasked_descriptors != white_mask_descriptors),
        0);
    for (std::size_t index = 0; index < unmasked_keypoints.size(); ++index)
    {
        EXPECT_EQ(
            unmasked_keypoints[index].pt,
            white_mask_keypoints[index].pt);
        EXPECT_EQ(
            unmasked_keypoints[index].octave,
            white_mask_keypoints[index].octave);
    }
}

}  // namespace
