/**
 * @file prepare_ego_config_settings_probe.cpp
 * @brief 用真实 ORB_SLAM3 Settings 加载转换器生成的配置，供 pytest 调用。
 */

#include <cmath>
#include <exception>
#include <iostream>

#include "CameraModels/KannalaBrandt8.h"
#include "Settings.h"
#include "System.h"

int main(int argc, char** argv)
{
  if (argc != 2)
  {
    std::cerr << "usage: " << argv[0] << " CONFIG.yaml" << std::endl;
    return 2;
  }

  try
  {
    ORB_SLAM3::Settings settings(argv[1], ORB_SLAM3::System::IMU_STEREO);
    auto* camera1 =
        dynamic_cast<ORB_SLAM3::KannalaBrandt8*>(settings.camera1());
    auto* camera2 =
        dynamic_cast<ORB_SLAM3::KannalaBrandt8*>(settings.camera2());
    if (camera1 == nullptr || camera2 == nullptr)
    {
      std::cerr << "expected two KannalaBrandt8 cameras" << std::endl;
      return 1;
    }

    for (int index = 0; index < 8; ++index)
    {
      if (!std::isfinite(camera1->getParameter(index)) ||
          !std::isfinite(camera2->getParameter(index)))
      {
        std::cerr << "non-finite camera parameter at index " << index
                  << std::endl;
        return 1;
      }
    }

    const float floatFields[] = {
        settings.fps(),
        settings.thDepth(),
        settings.noiseGyro(),
        settings.noiseAcc(),
        settings.gyroWalk(),
        settings.accWalk(),
        settings.imuFrequency(),
        settings.scaleFactor(),
        settings.keyFrameSize(),
        settings.keyFrameLineWidth(),
        settings.graphLineWidth(),
        settings.pointSize(),
        settings.cameraSize(),
        settings.cameraLineWidth(),
        settings.viewPointX(),
        settings.viewPointY(),
        settings.viewPointZ(),
        settings.viewPointF(),
    };
    for (float value : floatFields)
    {
      if (!std::isfinite(value))
      {
        std::cerr << "non-finite float setting" << std::endl;
        return 1;
      }
    }
    if (settings.imuFrequency() <= 0.0F)
    {
      std::cerr << "IMU.Frequency must be positive" << std::endl;
      return 1;
    }
    if (settings.deferLowMotionReset() || settings.deferPreBA2TrackingReset())
    {
      std::cerr << "generated config must leave deferred resets disabled"
                << std::endl;
      return 1;
    }
    if (!settings.Tbc().matrix().allFinite())
    {
      std::cerr << "IMU.T_b_c1 is not finite" << std::endl;
      return 1;
    }

    std::cout << "loaded " << argv[1] << std::endl;
    return 0;
  }
  catch (const std::exception& error)
  {
    std::cerr << "Settings load failed: " << error.what() << std::endl;
    return 1;
  }
  catch (...)
  {
    std::cerr << "Settings load failed with unknown error" << std::endl;
    return 1;
  }
}
