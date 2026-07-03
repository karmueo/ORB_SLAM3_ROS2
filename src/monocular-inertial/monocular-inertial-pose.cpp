/**
 * @file monocular-inertial-pose.cpp
 * @brief 实现单目惯性节点使用的位姿转换与标定读取辅助函数。
 */

#include "monocular-inertial-pose.hpp"

#include <opencv2/core/core.hpp>

namespace
{
/**
 * @brief 将 OpenCV 4x4 浮点矩阵转换为 Sophus SE3。
 * @param matrix OpenCV 4x4 变换矩阵。
 * @return Sophus SE3 位姿。
 */
Sophus::SE3f CvMatrixToSophus(const cv::Mat& matrix)
{
    /** @brief 按行存储的 Eigen 4x4 变换矩阵，用于匹配 OpenCV 内存布局。 */
    Eigen::Matrix<float, 4, 4, Eigen::RowMajor> eigenMatrix(matrix.ptr<float>(0));
    return Sophus::SE3f(eigenMatrix);
}
}  // namespace

/**
 * @brief 根据 ORB_SLAM3 相机位姿和机体到相机外参计算世界到机体系位姿。
 * @param Tcw 世界系到相机系位姿。
 * @param Tbc 机体系到相机系外参，对应 ORB_SLAM3 配置中的 IMU.T_b_c1 或 Tbc。
 * @return 世界系到机体系位姿。
 */
Sophus::SE3f ComputeBodyPoseFromCameraPose(const Sophus::SE3f& Tcw, const Sophus::SE3f& Tbc)
{
    /** @brief 世界系到相机系位姿的逆变换，即相机系到世界系位姿。 */
    const Sophus::SE3f Twc = Tcw.inverse();
    /** @brief 机体系到相机系外参的逆变换，即相机系到机体系外参。 */
    const Sophus::SE3f Tcb = Tbc.inverse();
    return Twc * Tcb;
}

/**
 * @brief 从 ORB_SLAM3 配置文件读取机体系到相机系外参。
 * @param settingsFile ORB_SLAM3 YAML 配置文件路径。
 * @param Tbc 输出的机体系到相机系外参。
 * @param errorMessage 读取失败时输出错误说明；可为空。
 * @return 读取成功返回 true，否则返回 false。
 */
bool LoadBodyToCameraExtrinsic(
    const std::string& settingsFile,
    Sophus::SE3f& Tbc,
    std::string* errorMessage)
{
    /** @brief ORB_SLAM3 配置文件读取器。 */
    cv::FileStorage fileStorage(settingsFile, cv::FileStorage::READ);
    if (!fileStorage.isOpened())
    {
        if (errorMessage)
        {
            *errorMessage = "cannot open settings file";
        }
        return false;
    }

    /** @brief 新版 ORB_SLAM3 配置中的机体系到相机系外参节点。 */
    cv::FileNode extrinsicNode = fileStorage["IMU.T_b_c1"];
    if (extrinsicNode.empty())
    {
        extrinsicNode = fileStorage["Tbc"];
    }

    if (extrinsicNode.empty())
    {
        if (errorMessage)
        {
            *errorMessage = "missing IMU.T_b_c1 or Tbc";
        }
        return false;
    }

    /** @brief 从 YAML 读取出的机体系到相机系外参矩阵。 */
    const cv::Mat cvTbc = extrinsicNode.mat();
    if (cvTbc.rows != 4 || cvTbc.cols != 4 || cvTbc.type() != CV_32F)
    {
        if (errorMessage)
        {
            *errorMessage = "extrinsic matrix must be 4x4 float";
        }
        return false;
    }

    Tbc = CvMatrixToSophus(cvTbc);
    return true;
}
