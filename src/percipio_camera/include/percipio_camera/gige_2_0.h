#pragma once

#include "percipio_interface.h"

namespace percipio 
{

class GigE_2_0 : public GigEBase {
  public:
    GigE_2_0(const TY_DEV_HANDLE dev);
    ~GigE_2_0() {};

    virtual std::vector<VideoMode>    getLeftIRVideoModeList();
    virtual std::vector<VideoMode>    getRightIRVideoModeList();
    virtual std::vector<VideoMode>    getColorVideoModeList();
    virtual std::vector<VideoMode>    getDepthVideoModeList();

    virtual TY_STATUS getColorCalibData(TY_CAMERA_CALIB_INFO& calib_data);
    virtual TY_STATUS getDepthCalibData(TY_CAMERA_CALIB_INFO& calib_data);

    virtual TY_STATUS getColorIntrinsic(TY_CAMERA_INTRINSIC& Intrinsic);
    virtual TY_STATUS getDepthIntrinsic(TY_CAMERA_INTRINSIC& Intrinsic);

    virtual TY_STATUS AcquisitionInit();
    virtual TY_STATUS SetImageMode(const SensorType sensorType, const int width, const int height, const std::string& fmt);

    virtual TY_STATUS PreSetting();

    virtual TY_STATUS EnableColorStream(const bool en);
    virtual TY_STATUS EnableDepthStream(const bool en);
    virtual TY_STATUS EnableLeftIRStream(const bool en);
    virtual TY_STATUS EnableRightIRStream(const bool en);

  private:
    TY_COMPONENT_ID ids;
    bool load_default_parameter();
};
}