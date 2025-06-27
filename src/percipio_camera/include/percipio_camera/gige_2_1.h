#pragma once

#include "percipio_interface.h"

namespace percipio 
{

typedef struct _GegE2ImageMode
{
  uint32_t width;
  uint32_t height;
  uint32_t binning;
  uint32_t pixel_fmt;
} GegE2ImageMode;

class GigE_2_1 : public GigEBase {
  public:
    GigE_2_1(const TY_DEV_HANDLE dev);
    ~GigE_2_1() {};

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
    std::vector<GegE2ImageMode>       dump_source_image_mode();
    TY_STATUS getCalibData(TY_CAMERA_CALIB_INFO& calib_data);
};
}