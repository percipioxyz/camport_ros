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

    virtual TY_STATUS getIRLensType(TYLensOpticalType& type);
    virtual TY_STATUS getIRRectificationMode(IRImageRectificationMode& mode);

    virtual TY_STATUS getLeftIRCalibData(TY_CAMERA_CALIB_INFO& calib_data);
    virtual TY_STATUS getLeftIRRotation(TY_CAMERA_ROTATION& rotation);
    virtual TY_STATUS getLeftIRRectifiedIntr(TY_CAMERA_INTRINSIC& rectified_intr);

    virtual TY_STATUS getRightIRCalibData(TY_CAMERA_CALIB_INFO& calib_data);
    virtual TY_STATUS getRightIRRotation(TY_CAMERA_ROTATION& rotation);
    virtual TY_STATUS getRightIRRectifiedIntr(TY_CAMERA_INTRINSIC& rectified_intr);

    virtual TY_STATUS getDepthScaleUnit(float& f_depth_scale);

    virtual TY_STATUS EnableHwIRUndistortion();

    virtual TY_STATUS AcquisitionInit();
    virtual TY_STATUS SetImageMode(const SensorType sensorType, const int width, const int height, const std::string& fmt);

    virtual TY_STATUS PreSetting();

    virtual TY_STATUS LoadParametersFromXML(const percipio_feat& cfg);

    virtual bool is_support_frame_rate_ctrl();
    virtual TY_STATUS  frame_rate_init(const float fps);
    virtual TY_STATUS  enable_trigger_mode(const bool en);

    virtual TY_STATUS  send_soft_trigger_signal();

    virtual TY_STATUS EnableColorStream(const bool en);
    virtual TY_STATUS EnableDepthStream(const bool en);
    virtual TY_STATUS EnableLeftIRStream(const bool en);
    virtual TY_STATUS EnableRightIRStream(const bool en);

    virtual void Reset();

  private:
    TY_COMPONENT_ID ids;
    TY_AEC_ROI_PARAM aec_roi;

    bool load_default_parameter();
    bool try_xml_parameter(const std::string& source, const std::string& feat, const std::string& val_str);
};
}