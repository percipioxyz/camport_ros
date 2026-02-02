#include "percipio_camera/gige_2_0.h"

#include "percipio_camera/crc32.hpp"
#include "percipio_camera/ParametersParse.hpp"
#include "percipio_camera/huffman.h"

namespace percipio 
{

#define MAX_STORAGE_SIZE    (10*1024*1024)

enum EncodingType : uint32_t  
{
  HUFFMAN = 0,
};

extern std::map<SensorType, std::string> SensorDesc;
extern bool MatchPixelFormat(const std::string& fmt_desc, const TYPixFmt& m_fmt);

static std::map<std::string, TY_FEATURE_ID> m_gige_2_0_feature_map = {
    {"TriggerDelay",                              TY_INT_TRIGGER_DELAY_US},

    {"ExposureAuto",                              TY_BOOL_AUTO_EXPOSURE},
    {"ExposureTime",                              TY_INT_EXPOSURE_TIME},
    {"ExposureTargetBrightness",                  TY_INT_AE_TARGET_Y},
    {"AnalogAll",                                 TY_INT_ANALOG_GAIN},
    {"DigitalAll",                                TY_INT_GAIN},
    {"DigitalRed",                                TY_INT_R_GAIN},
    {"DigitalGreen",                              TY_INT_G_GAIN},
    {"DigitalBlue",                               TY_INT_B_GAIN},
    {"BalanceWhiteAuto",                          TY_BOOL_AUTO_AWB},

    {"AutoFunctionAOIOffsetX",                    TY_STRUCT_AEC_ROI},
    {"AutoFunctionAOIOffsetY",                    TY_STRUCT_AEC_ROI},
    {"AutoFunctionAOIWidth",                      TY_STRUCT_AEC_ROI},
    {"AutoFunctionAOIHeight",                     TY_STRUCT_AEC_ROI},

    {"DepthScaleUnit",                            TY_FLOAT_SCALE_UNIT},
    {"DepthSgbmImageNumber",                      TY_INT_SGBM_IMAGE_NUM},
    {"DepthSgbmDisparityNumber",                  TY_INT_SGBM_DISPARITY_NUM},
    {"DepthSgbmDisparityOffset",                  TY_INT_SGBM_DISPARITY_OFFSET},
    {"DepthSgbmMatchWinHeight",                   TY_INT_SGBM_MATCH_WIN_HEIGHT},
    {"DepthSgbmSemiParamP1",                      TY_INT_SGBM_SEMI_PARAM_P1},
    {"DepthSgbmSemiParamP2",                      TY_INT_SGBM_SEMI_PARAM_P2},
    {"DepthSgbmUniqueFactor",                     TY_INT_SGBM_UNIQUE_FACTOR},
    {"DepthSgbmUniqueAbsDiff",                    TY_INT_SGBM_UNIQUE_ABSDIFF},
    {"DepthSgbmUniqueMaxCost",                    TY_INT_SGBM_UNIQUE_MAX_COST},
    {"DepthSgbmHFilterHalfWin",                   TY_BOOL_SGBM_HFILTER_HALF_WIN},
    {"DepthSgbmMatchWinWidth",                    TY_INT_SGBM_MATCH_WIN_WIDTH},
    {"DepthSgbmMedFilter",                        TY_BOOL_SGBM_MEDFILTER},
    {"DepthSgbmLRC",                              TY_BOOL_SGBM_LRC},
    {"DepthSgbmLRCDiff",                          TY_INT_SGBM_LRC_DIFF},
    {"DepthSgbmMedFilterThresh",                  TY_INT_SGBM_MEDFILTER_THRESH},
    {"DepthSgbmSemiParamP1Scale",                 TY_INT_SGBM_SEMI_PARAM_P1_SCALE},
    {"DepthSgpmPhaseNumber",                      TY_INT_SGPM_PHASE_NUM},
    {"DepthSgpmPhaseScale",                       TY_INT_SGPM_NORMAL_PHASE_SCALE},
    {"DepthSgpmPhaseOffset",                      TY_INT_SGPM_NORMAL_PHASE_OFFSET},
    {"DepthSgpmReferencePhaseScale",              TY_INT_SGPM_REF_PHASE_SCALE},
    {"DepthSgpmReferencePhaseOffset",             TY_INT_SGPM_REF_PHASE_OFFSET},
    {"DepthSgpmEpipolarConstraintPatternScale",   TY_INT_SGPM_REF_PHASE_SCALE},
    {"DepthSgpmEpipolarConstraintPatternOffset",  TY_INT_SGPM_REF_PHASE_OFFSET},
    {"DepthSgpmEpipolarConstraintEnable",         TY_BOOL_SGPM_EPI_EN},
    {"DepthSgpmEpipolarConstraintChan0",          TY_INT_SGPM_EPI_CH0},
    {"DepthSgpmEpipolarConstraintChan1",          TY_INT_SGPM_EPI_CH1},
    {"DepthSgpmEpipolarConstraintThresh",         TY_INT_SGPM_EPI_THRESH},
    {"DepthSgpmPhaseOrderFilterEnable",           TY_BOOL_SGPM_ORDER_FILTER_EN},
    {"DepthSgpmPhaseOrderFilterChannel",          TY_INT_SGPM_ORDER_FILTER_CHN},
    {"DepthRangeMin",                             TY_INT_DEPTH_MIN_MM},
    {"DepthRangeMax",                             TY_INT_DEPTH_MAX_MM},
    {"DepthSgbmTextureFilterValueOffset",         TY_INT_SGBM_TEXTURE_OFFSET},
    {"DepthSgbmTextureFilterThreshold",           TY_INT_SGBM_TEXTURE_THRESH},

    {"DepthStreamTofFilterThreshold",             TY_INT_FILTER_THRESHOLD},
    {"DepthStreamTofChannel",                     TY_INT_TOF_CHANNEL},
    {"DepthStreamTofModulationThreshold",         TY_INT_TOF_MODULATION_THRESHOLD},
    {"DepthStreamTofDepthQuality",                TY_ENUM_DEPTH_QUALITY},
    {"DepthStreamTofHdrRatio",                    TY_INT_TOF_HDR_RATIO},
    {"DepthStreamTofJitterThreshold",             TY_INT_TOF_JITTER_THRESHOLD},

    {"DepthStreamTofAntiSunlightIndex",           TY_INT_TOF_ANTI_SUNLIGHT_INDEX},
    {"DepthStreamTofAntiInterference",            TY_BOOL_TOF_ANTI_INTERFERENCE},
    {"DepthStreamTofSpeckleSize",                 TY_INT_MAX_SPECKLE_SIZE},
    {"DepthStreamTofSpeckleDiff",                 TY_INT_MAX_SPECKLE_DIFF}
};

#define INVALID_COMPONENT  0xFFFFFFFF
static TY_COMPONENT_ID SensorTypeToPercipComonentID(const SensorType& type)
{
  switch(type) {
    case SENSOR_IR_LEFT: return TY_COMPONENT_IR_CAM_LEFT;
    case SENSOR_IR_RIGHT: return TY_COMPONENT_IR_CAM_RIGHT;
    case SENSOR_COLOR: return TY_COMPONENT_RGB_CAM;
    case SENSOR_DEPTH: return TY_COMPONENT_DEPTH_CAM;
    case SENSOR_POINT3D: return TY_COMPONENT_DEPTH_CAM;
    default: return INVALID_COMPONENT;
  }
}

static TY_PIXEL_FORMAT StdTYPixFmtToOldPixelFormat(TYPixFmt fmt)
{
  switch(fmt) {
    case TYPixelFormatMono8: return TY_PIXEL_FORMAT_MONO;
    case TYPixelFormatBayerGRBG8: return TY_PIXEL_FORMAT_BAYER8GB;
    case TYPixelFormatBayerRGGB8: return TY_PIXEL_FORMAT_BAYER8BG;
    case TYPixelFormatBayerGBRG8: return TY_PIXEL_FORMAT_BAYER8GR;
    case TYPixelFormatBayerBGGR8: return TY_PIXEL_FORMAT_BAYER8RG;

    case TYPixelFormatMono10 : return TY_PIXEL_FORMAT_CSI_MONO10;
    case TYPixelFormatBayerGRBG10 : return TY_PIXEL_FORMAT_CSI_BAYER10GRBG;
    case TYPixelFormatBayerRGGB10 : return TY_PIXEL_FORMAT_CSI_BAYER10RGGB;
    case TYPixelFormatBayerGBRG10 : return TY_PIXEL_FORMAT_CSI_BAYER10GBRG;
    case TYPixelFormatBayerBGGR10 : return TY_PIXEL_FORMAT_CSI_BAYER10BGGR;

    case TYPixelFormatMono12 : return TY_PIXEL_FORMAT_CSI_MONO12;
    case TYPixelFormatBayerGRBG12 : return TY_PIXEL_FORMAT_CSI_BAYER12GRBG;
    case TYPixelFormatBayerRGGB12 : return TY_PIXEL_FORMAT_CSI_BAYER12RGGB;
    case TYPixelFormatBayerGBRG12 : return TY_PIXEL_FORMAT_CSI_BAYER12GBRG;
    case TYPixelFormatBayerBGGR12 : return TY_PIXEL_FORMAT_CSI_BAYER12BGGR;

    case TYPixelFormatCoord3D_C16 : return TY_PIXEL_FORMAT_DEPTH16;
    case TYPixelFormatYUV422_8 : return TY_PIXEL_FORMAT_YUYV;
    case TYPixelFormatMono16 : return TY_PIXEL_FORMAT_MONO16;

    case TYPixelFormatRGB8 : return TY_PIXEL_FORMAT_RGB;
    case TYPixelFormatBGR8 : return TY_PIXEL_FORMAT_BGR;
    case TYPixelFormatJPEG : return TY_PIXEL_FORMAT_JPEG;

    case TYPixelFormatCoord3D_ABC16 : return TY_PIXEL_FORMAT_XYZ48;

    default: return 0xFFFFFFFF;
  }
}

static TYPixFmt OldPixelFormatToStdTYPixFmt(TY_PIXEL_FORMAT fmt)
{
  switch(fmt) {
    case TY_PIXEL_FORMAT_MONO: return TYPixelFormatMono8;
    case TY_PIXEL_FORMAT_BAYER8GB: return TYPixelFormatBayerGRBG8;
    case TY_PIXEL_FORMAT_BAYER8BG: return TYPixelFormatBayerRGGB8;
    case TY_PIXEL_FORMAT_BAYER8GR: return TYPixelFormatBayerGBRG8;
    case TY_PIXEL_FORMAT_BAYER8RG: return TYPixelFormatBayerBGGR8;

    case TY_PIXEL_FORMAT_CSI_MONO10: return TYPixelFormatMono10;
    case TY_PIXEL_FORMAT_CSI_BAYER10GRBG: return TYPixelFormatBayerGRBG10;
    case TY_PIXEL_FORMAT_CSI_BAYER10RGGB: return TYPixelFormatBayerRGGB10;
    case TY_PIXEL_FORMAT_CSI_BAYER10GBRG: return TYPixelFormatBayerGBRG10;
    case TY_PIXEL_FORMAT_CSI_BAYER10BGGR: return TYPixelFormatBayerBGGR10;

    case TY_PIXEL_FORMAT_CSI_MONO12: return TYPixelFormatMono12;
    case TY_PIXEL_FORMAT_CSI_BAYER12GRBG: return TYPixelFormatBayerGRBG12;
    case TY_PIXEL_FORMAT_CSI_BAYER12RGGB: return TYPixelFormatBayerRGGB12;
    case TY_PIXEL_FORMAT_CSI_BAYER12GBRG: return TYPixelFormatBayerGBRG12;
    case TY_PIXEL_FORMAT_CSI_BAYER12BGGR: return TYPixelFormatBayerBGGR12;

    case TY_PIXEL_FORMAT_DEPTH16: return TYPixelFormatCoord3D_C16;
    case TY_PIXEL_FORMAT_YUYV: return TYPixelFormatYUV422_8;
    case TY_PIXEL_FORMAT_MONO16: return TYPixelFormatMono16;
    case TY_PIXEL_FORMAT_TOF_IR_MONO16: return TYPixelFormatMono16;

    case TY_PIXEL_FORMAT_RGB: return TYPixelFormatRGB8;
    case TY_PIXEL_FORMAT_BGR: return TYPixelFormatBGR8;
    case TY_PIXEL_FORMAT_JPEG: return TYPixelFormatJPEG;

    case TY_PIXEL_FORMAT_XYZ48: return TYPixelFormatCoord3D_ABC16;

    default: return TYPixelFormatInvalid;
  }
}

GigE_2_0::GigE_2_0(const TY_DEV_HANDLE dev) : GigEBase(dev)
{
  ids = 0;
  TY_STATUS  status = TYGetComponentIDs(hDevice, &ids);
  if(status) return ;

  if(ids & TY_COMPONENT_DEPTH_CAM) 
    TYHasFeature(hDevice, TY_COMPONENT_DEPTH_CAM, TY_STRUCT_CAM_DISTORTION, &need_depth_undistortion);
}

std::vector<VideoMode> GigE_2_0::getLeftIRVideoModeList()
{
  TY_STATUS ret;
  if(!(ids & TY_COMPONENT_IR_CAM_LEFT))
    return std::vector<VideoMode>();

  std::vector<TY_ENUM_ENTRY> feature_info;
  get_feature_enum_list(hDevice, TY_COMPONENT_IR_CAM_LEFT, TY_ENUM_IMAGE_MODE, feature_info);

  videos[SENSOR_IR_LEFT].clear();
  for(size_t i = 0; i < feature_info.size(); i++) {
    uint32_t fmt = TYPixelFormat(feature_info[i].value);
    uint32_t width = TYImageWidth(feature_info[i].value);
    uint32_t height = TYImageHeight(feature_info[i].value);
    videos[SENSOR_IR_LEFT].push_back(VideoMode(OldPixelFormatToStdTYPixFmt(fmt), width, height));
  }

  return videos[SENSOR_IR_LEFT];
}

std::vector<VideoMode> GigE_2_0::getRightIRVideoModeList()
{
  TY_STATUS ret;
  if(!(ids & TY_COMPONENT_IR_CAM_RIGHT))
    return std::vector<VideoMode>();

  std::vector<TY_ENUM_ENTRY> feature_info;
  get_feature_enum_list(hDevice, TY_COMPONENT_IR_CAM_RIGHT, TY_ENUM_IMAGE_MODE, feature_info);

  videos[SENSOR_IR_RIGHT].clear();
  for(size_t i = 0; i < feature_info.size(); i++) {
    uint32_t fmt = TYPixelFormat(feature_info[i].value);
    uint32_t width = TYImageWidth(feature_info[i].value);
    uint32_t height = TYImageHeight(feature_info[i].value);
    videos[SENSOR_IR_RIGHT].push_back(VideoMode(OldPixelFormatToStdTYPixFmt(fmt), width, height));
  }

  return videos[SENSOR_IR_RIGHT];
}

std::vector<VideoMode> GigE_2_0::getColorVideoModeList()
{
  TY_STATUS ret;
  if(!(ids & TY_COMPONENT_RGB_CAM))
    return std::vector<VideoMode>();

  std::vector<TY_ENUM_ENTRY> feature_info;
  get_feature_enum_list(hDevice, TY_COMPONENT_RGB_CAM, TY_ENUM_IMAGE_MODE, feature_info);

  videos[SENSOR_COLOR].clear();
  for(size_t i = 0; i < feature_info.size(); i++) {
    uint32_t fmt = TYPixelFormat(feature_info[i].value);
    uint32_t width = TYImageWidth(feature_info[i].value);
    uint32_t height = TYImageHeight(feature_info[i].value);
    videos[SENSOR_COLOR].push_back(VideoMode(OldPixelFormatToStdTYPixFmt(fmt), width, height));
  }

  return videos[SENSOR_COLOR];
}

std::vector<VideoMode> GigE_2_0::getDepthVideoModeList()
{
  TY_STATUS ret;
  if(!(ids & TY_COMPONENT_DEPTH_CAM))
    return std::vector<VideoMode>();

  std::vector<TY_ENUM_ENTRY> feature_info;
  get_feature_enum_list(hDevice, TY_COMPONENT_DEPTH_CAM, TY_ENUM_IMAGE_MODE, feature_info);

  videos[SENSOR_DEPTH].clear();
  for(size_t i = 0; i < feature_info.size(); i++) {
    uint32_t fmt = TYPixelFormat(feature_info[i].value);
    uint32_t width = TYImageWidth(feature_info[i].value);
    uint32_t height = TYImageHeight(feature_info[i].value);
    videos[SENSOR_DEPTH].push_back(VideoMode(OldPixelFormatToStdTYPixFmt(fmt), width, height));
  }

  return videos[SENSOR_DEPTH];
}

TY_STATUS GigE_2_0::getColorCalibData(TY_CAMERA_CALIB_INFO& calib_data)
{
  if(!(ids & TY_COMPONENT_RGB_CAM))
    return TY_STATUS_INVALID_COMPONENT;

  return TYGetStruct(hDevice, TY_COMPONENT_RGB_CAM, TY_STRUCT_CAM_CALIB_DATA, &calib_data, sizeof(calib_data));
}

TY_STATUS GigE_2_0::getDepthCalibData(TY_CAMERA_CALIB_INFO& calib_data)
{
  if(!(ids & TY_COMPONENT_DEPTH_CAM))
    return TY_STATUS_INVALID_COMPONENT;

  return TYGetStruct(hDevice, TY_COMPONENT_DEPTH_CAM, TY_STRUCT_CAM_CALIB_DATA, &calib_data, sizeof(calib_data));
}

TY_STATUS GigE_2_0::getColorIntrinsic(TY_CAMERA_INTRINSIC& Intrinsic)
{
  if(!(ids & TY_COMPONENT_RGB_CAM))
    return TY_STATUS_INVALID_COMPONENT;

  return TYGetStruct(hDevice, TY_COMPONENT_RGB_CAM, TY_STRUCT_CAM_INTRINSIC, &Intrinsic, sizeof(Intrinsic));
}

TY_STATUS GigE_2_0::getDepthIntrinsic(TY_CAMERA_INTRINSIC& Intrinsic)
{
  if(!(ids & TY_COMPONENT_DEPTH_CAM))
    return TY_STATUS_INVALID_COMPONENT;

  return TYGetStruct(hDevice, TY_COMPONENT_DEPTH_CAM, TY_STRUCT_CAM_INTRINSIC, &Intrinsic, sizeof(Intrinsic));
}

TY_STATUS GigE_2_0::getIRLensType(TYLensOpticalType& type)
{
  type = TY_LENS_PINHOLE;
  bool has_lens_type = false;
  TYHasFeature(hDevice, TY_COMPONENT_IR_CAM_LEFT, TY_ENUM_LENS_OPTICAL_TYPE, &has_lens_type);
  if(!has_lens_type) return TY_STATUS_OK;

  return TYGetEnum(hDevice, TY_COMPONENT_IR_CAM_LEFT, TY_ENUM_LENS_OPTICAL_TYPE, (uint32_t*)&type);
}

TY_STATUS GigE_2_0::getIRRectificationMode(IRImageRectificationMode& mode)
{
  mode = DISTORTION_CORRECTION;

  bool rotation = false;
  bool rectified_intr = false;  
  TYHasFeature(hDevice, TY_COMPONENT_IR_CAM_LEFT, TY_STRUCT_CAM_RECTIFIED_ROTATION, &rotation);
  TYHasFeature(hDevice, TY_COMPONENT_IR_CAM_LEFT, TY_STRUCT_CAM_RECTIFIED_INTRI, &rectified_intr);
  if(rotation && rectified_intr) mode = EPIPOLAR_RECTIFICATION;
  return TY_STATUS_OK;
}

TY_STATUS GigE_2_0::getLeftIRCalibData(TY_CAMERA_CALIB_INFO& calib_data)
{
  if(!(ids & TY_COMPONENT_IR_CAM_LEFT)) return TY_STATUS_INVALID_COMPONENT;
  return TYGetStruct(hDevice, TY_COMPONENT_IR_CAM_LEFT, TY_STRUCT_CAM_CALIB_DATA, &calib_data, sizeof(TY_CAMERA_CALIB_INFO));
}

TY_STATUS GigE_2_0::getLeftIRRotation(TY_CAMERA_ROTATION& rotation)
{
  if(!(ids & TY_COMPONENT_IR_CAM_LEFT)) return TY_STATUS_INVALID_COMPONENT;
  return TYGetStruct(hDevice, TY_COMPONENT_IR_CAM_LEFT, TY_STRUCT_CAM_RECTIFIED_ROTATION, &rotation, sizeof(TY_CAMERA_ROTATION));
}

TY_STATUS GigE_2_0::getLeftIRRectifiedIntr(TY_CAMERA_INTRINSIC& rectified_intr)
{
  if(!(ids & TY_COMPONENT_IR_CAM_LEFT)) return TY_STATUS_INVALID_COMPONENT;
  return TYGetStruct(hDevice, TY_COMPONENT_IR_CAM_LEFT, TY_STRUCT_CAM_RECTIFIED_INTRI, &rectified_intr, sizeof(TY_CAMERA_INTRINSIC));
}

TY_STATUS GigE_2_0::getRightIRCalibData(TY_CAMERA_CALIB_INFO& calib_data)
{
  if(!(ids & TY_COMPONENT_IR_CAM_RIGHT)) return TY_STATUS_INVALID_COMPONENT;
  return TYGetStruct(hDevice, TY_COMPONENT_IR_CAM_RIGHT, TY_STRUCT_CAM_CALIB_DATA, &calib_data, sizeof(TY_CAMERA_CALIB_INFO));
}

TY_STATUS GigE_2_0::getRightIRRotation(TY_CAMERA_ROTATION& rotation)
{
  if(!(ids & TY_COMPONENT_IR_CAM_RIGHT)) return TY_STATUS_INVALID_COMPONENT;
  return TYGetStruct(hDevice, TY_COMPONENT_IR_CAM_RIGHT, TY_STRUCT_CAM_RECTIFIED_ROTATION, &rotation, sizeof(TY_CAMERA_ROTATION));
} 

TY_STATUS GigE_2_0::getRightIRRectifiedIntr(TY_CAMERA_INTRINSIC& rectified_intr)
{
  if(!(ids & TY_COMPONENT_IR_CAM_RIGHT)) return TY_STATUS_INVALID_COMPONENT;
  return TYGetStruct(hDevice, TY_COMPONENT_IR_CAM_RIGHT, TY_STRUCT_CAM_RECTIFIED_INTRI, &rectified_intr, sizeof(TY_CAMERA_INTRINSIC));
}

TY_STATUS GigE_2_0::getDepthScaleUnit(float& f_depth_scale)
{
  return TYGetFloat(hDevice, TY_COMPONENT_DEPTH_CAM, TY_FLOAT_SCALE_UNIT, &f_depth_scale);
}

TY_STATUS GigE_2_0::EnableHwIRUndistortion()
{
  bool has = false;
  TY_STATUS status = TYHasFeature(hDevice, TY_COMPONENT_IR_CAM_LEFT, TY_BOOL_UNDISTORTION, &has);
  if(status) return status;
  if(!has) return TY_STATUS_NOT_PERMITTED;
  return TYSetBool(hDevice, TY_COMPONENT_IR_CAM_LEFT, TY_BOOL_UNDISTORTION, true);
}

TY_STATUS GigE_2_0::AcquisitionInit()
{
  if(!(ids & TY_COMPONENT_DEVICE))
    return TY_STATUS_OK;

  TY_STATUS status = TYSetEnum(hDevice, TY_COMPONENT_DEVICE, TY_ENUM_TIME_SYNC_TYPE, TY_TIME_SYNC_TYPE_HOST);
  if(status) {
    ROS_WARN("Failed to set time sync tpye to host, error: %d", status);
  }

  bool hasTrigger = false;
  TYHasFeature(hDevice, TY_COMPONENT_DEVICE, TY_STRUCT_TRIGGER_PARAM, &hasTrigger);
  if (hasTrigger) {
    TY_TRIGGER_PARAM trigger;
    trigger.mode = TY_TRIGGER_MODE_OFF;
    TYSetStruct(hDevice, TY_COMPONENT_DEVICE, TY_STRUCT_TRIGGER_PARAM, &trigger, sizeof(trigger));
  }
  return TY_STATUS_OK;
}

TY_STATUS GigE_2_0::SetImageMode(const SensorType sensorType, const int width, const int height, const std::string& fmt)
{
  TY_STATUS ret;
  TY_COMPONENT_ID component = SensorTypeToPercipComonentID(sensorType);
  if(component == INVALID_COMPONENT) return TY_STATUS_INVALID_COMPONENT;

  if(!(ids & component))
    return TY_STATUS_INVALID_COMPONENT;

  auto it = videos.find(sensorType);
  if (it == videos.end())
    return TY_STATUS_INVALID_COMPONENT;

  if(fmt.length()) {
    for(size_t i = 0; i < videos[sensorType].size(); i++) {
      int m_width = videos[sensorType][i].getResolutionX();
      int m_height = videos[sensorType][i].getResolutionY();
      int m_pixFmt = videos[sensorType][i].getPixelFormat();
      if((width == m_width) && (height == m_height) && MatchPixelFormat(fmt, m_pixFmt)) {
        ret = TYSetEnum(hDevice, component, TY_ENUM_IMAGE_MODE, TYImageMode2(StdTYPixFmtToOldPixelFormat(m_pixFmt), width, height));
        if(ret == TY_STATUS_OK) {
          current_video_mode[sensorType] = videos[sensorType][i];
          ROS_INFO("Set %s image mode : %d x %d, fmt: 0x%08x", SensorDesc[sensorType].c_str(), width, height, StdTYPixFmtToOldPixelFormat(m_pixFmt));
          return TY_STATUS_OK;
        }
      }
    }

  } else {
    for(size_t i = 0; i < videos[sensorType].size(); i++) {
      int m_width = videos[sensorType][i].getResolutionX();
      int m_height = videos[sensorType][i].getResolutionY();
      int m_pixFmt = videos[sensorType][i].getPixelFormat();
      if((width == m_width) && (height == m_height)) {
        ret = TYSetEnum(hDevice, component, TY_ENUM_IMAGE_MODE, TYImageMode2(StdTYPixFmtToOldPixelFormat(m_pixFmt), width, height));
        if(ret == TY_STATUS_OK) {
          current_video_mode[sensorType] = videos[sensorType][i];
          ROS_INFO("Set %s image mode : %d x %d, fmt: 0x%08x", SensorDesc[sensorType].c_str(), width, height, StdTYPixFmtToOldPixelFormat(m_pixFmt));
          return TY_STATUS_OK;
        }
      }
    }
  }

  ROS_WARN("Incorrect image mode : 0x%08x target size: %d x %d - format(%s), using default value.", component, width, height, fmt.c_str());

  uint32_t image_mode;
  ret = TYGetEnum(hDevice, component, TY_ENUM_IMAGE_MODE, &image_mode);
  if(ret == TY_STATUS_OK) {
    current_video_mode[sensorType] = VideoMode(OldPixelFormatToStdTYPixFmt(TYPixelFormat(image_mode)), TYImageWidth(image_mode), TYImageHeight(image_mode));
    ROS_INFO("Set %s image mode : %d x %d, fmt: 0x%08x", SensorDesc[sensorType].c_str(), TYImageWidth(image_mode), TYImageHeight(image_mode), OldPixelFormatToStdTYPixFmt(TYPixelFormat(image_mode)));
  } else {
    ROS_WARN("Read default image mode : 0x%08x failed!", component);
  }

  return ret;
}

TY_STATUS GigE_2_0::PreSetting()
{
  load_default_parameter();

  if(ids & TY_COMPONENT_RGB_CAM) {
    uint32_t image_mode;
    TY_STATUS ret = TYGetEnum(hDevice, TY_COMPONENT_RGB_CAM, TY_ENUM_IMAGE_MODE, &image_mode);
    if(TY_STATUS_OK == ret) {
      //default video mode
      current_video_mode[SENSOR_COLOR] = VideoMode(OldPixelFormatToStdTYPixFmt(TYPixelFormat(image_mode)), TYImageWidth(image_mode), TYImageHeight(image_mode));
    }
  }

  if(ids & TY_COMPONENT_DEVICE) {
    bool b_gsvp_resend_support = false;
    TYHasFeature(hDevice, TY_COMPONENT_DEVICE, TY_BOOL_GVSP_RESEND, &b_gsvp_resend_support);
    if(b_gsvp_resend_support) {
      ROS_INFO("Device Open resend.");
      TYSetBool(hDevice, TY_COMPONENT_DEVICE, TY_BOOL_GVSP_RESEND, true);
    }
  }

  if(ids & TY_COMPONENT_LASER) 
    return TYSetBool(hDevice, TY_COMPONENT_LASER, TY_BOOL_LASER_AUTO_CTRL, true);

  return TY_STATUS_OK;
}

bool GigE_2_0::is_support_frame_rate_ctrl()
{
  TY_TRIGGER_PARAM param;
  param.mode = TY_TRIGGER_MODE_M_PER;
  param.fps = 1;
  TY_STATUS ret = TYSetStruct(hDevice, TY_COMPONENT_DEVICE, TY_STRUCT_TRIGGER_PARAM, (void*)&param, sizeof(param));
  return ret ? false : true;
}

TY_STATUS GigE_2_0::frame_rate_init(const float fps)
{
  TY_TRIGGER_PARAM param;
  param.mode = TY_TRIGGER_MODE_M_PER;
  param.fps = (int8_t)fps;
  TY_STATUS ret = TYSetStruct(hDevice, TY_COMPONENT_DEVICE, TY_STRUCT_TRIGGER_PARAM, (void*)&param, sizeof(param));
  return  ret;
}

TY_STATUS GigE_2_0::enable_trigger_mode(const bool en)
{
  if(!en) return 0;

  TY_TRIGGER_PARAM_EX trigger;
  trigger.mode = TY_TRIGGER_MODE_SLAVE;
  return TYSetStruct(hDevice, TY_COMPONENT_DEVICE, TY_STRUCT_TRIGGER_PARAM_EX, &trigger, sizeof(trigger));
}

TY_STATUS  GigE_2_0::send_soft_trigger_signal()
{
  int err = TY_STATUS_OK;
  while(TY_STATUS_BUSY == (err = TYSendSoftTrigger(hDevice)));
  return err;
}

TY_STATUS GigE_2_0::LoadParametersFromXML(const percipio_feat& cfg)
{
  for(auto& iter : cfg) {
    for(auto& feat : iter.second) {
      try_xml_parameter(iter.first, feat.node_desc, feat.node_info);
    }
  }

  return TY_STATUS_OK;
}

TY_STATUS GigE_2_0::EnableColorStream(const bool en)
{
  TY_STATUS ret;
  if(!(ids & TY_COMPONENT_RGB_CAM))
    return TY_STATUS_INVALID_COMPONENT;
  
  if(en) {
    ret = TYEnableComponents(hDevice, TY_COMPONENT_RGB_CAM);
    if(ret) ROS_WARN("Enable rgb component failed: %d", ret);
  } else {
    ret = TYDisableComponents(hDevice, TY_COMPONENT_RGB_CAM);
    if(ret) ROS_WARN("Disable rgb component failed: %d", ret);
  }
  return ret;
}

TY_STATUS GigE_2_0::EnableDepthStream(const bool en)
{
  TY_STATUS ret;
  if(!(ids & TY_COMPONENT_DEPTH_CAM))
    return TY_STATUS_INVALID_COMPONENT;

  if(en) {
    ret = TYEnableComponents(hDevice, TY_COMPONENT_DEPTH_CAM);
    if(ret) ROS_WARN("Enable depth component failed: %d", ret);
  } else {
    ret = TYDisableComponents(hDevice, TY_COMPONENT_DEPTH_CAM);
    if(ret) ROS_WARN("Disable depth component failed: %d", ret);
  }
  return ret;
}

TY_STATUS GigE_2_0::EnableLeftIRStream(const bool en)
{
  TY_STATUS ret;
  if(!(ids & TY_COMPONENT_IR_CAM_LEFT))
    return TY_STATUS_INVALID_COMPONENT;

  if(en) {
    ret = TYEnableComponents(hDevice, TY_COMPONENT_IR_CAM_LEFT);
    if(ret) ROS_WARN("Enable Left-IR component failed: %d", ret);
  } else {
    ret = TYDisableComponents(hDevice, TY_COMPONENT_IR_CAM_LEFT);
    if(ret) ROS_WARN("Disable Left-IR component failed: %d", ret);
  }
  return ret;
}

TY_STATUS GigE_2_0::EnableRightIRStream(const bool en)
{
  TY_STATUS ret;
  if(!(ids & TY_COMPONENT_IR_CAM_RIGHT))
    return TY_STATUS_INVALID_COMPONENT;

  if(en) {
    ret = TYEnableComponents(hDevice, TY_COMPONENT_IR_CAM_RIGHT);
    if(ret) ROS_WARN("Enable Right-IR component failed: %d", ret);
  } else {
    ret = TYDisableComponents(hDevice, TY_COMPONENT_IR_CAM_RIGHT);
    if(ret) ROS_WARN("Disable Right-IR component failed: %d", ret);
  }
  return ret;
}

void GigE_2_0::Reset()
{
  TYCloseDevice(hDevice, true);
}

bool GigE_2_0::load_default_parameter()
{
  TY_STATUS status = TY_STATUS_OK;
  uint32_t block_size;
  uint8_t* blocks = new uint8_t[MAX_STORAGE_SIZE] ();

  if(!(ids & TY_COMPONENT_STORAGE)) return false;

  status = TYGetByteArraySize(hDevice, TY_COMPONENT_STORAGE, TY_BYTEARRAY_CUSTOM_BLOCK, &block_size);
  if(status != TY_STATUS_OK) {
    delete []blocks;
    return false;
  } 
    
  status = TYGetByteArray(hDevice, TY_COMPONENT_STORAGE, TY_BYTEARRAY_CUSTOM_BLOCK, blocks,  block_size);
  if(status != TY_STATUS_OK) {
    delete []blocks;
    return false;
  }
    
  uint32_t crc_data = *(uint32_t*)blocks;
  if(0 == crc_data || 0xffffffff == crc_data) {
    ROS_INFO("The CRC check code is empty.");
    delete []blocks;
    return false;
  } 
    
  uint32_t crc;
  std::string js_string;
  uint8_t* js_code = blocks + 4;
  crc = crc32_bitwise(js_code, strlen((const char*)js_code));
  if((crc != crc_data) || !isValidJsonString((const char*)js_code)) {
    EncodingType type = *(EncodingType*)(blocks + 4);
    switch(type) {
      case HUFFMAN:
      {
        uint32_t huffman_size = *(uint32_t*)(blocks + 8);
        uint8_t* huffman_ptr = (uint8_t*)(blocks + 12);
        if(huffman_size > (MAX_STORAGE_SIZE - 8)) {
          ROS_WARN("Storage data length error.");
          delete []blocks;
          return false;
        }
                
        crc = crc32_bitwise(huffman_ptr, huffman_size);
        if(crc_data != crc) {
          ROS_WARN("Storage area data check failed (check code error).");
          delete []blocks;
          return false;
        }

        std::string huffman_string(huffman_ptr, huffman_ptr + huffman_size);
        if(!TextHuffmanDecompression(huffman_string, js_string)) {
          ROS_ERROR("Huffman decompression error.");
          delete []blocks;
          return false;
        }
        break;
      }
      default:
      {
        ROS_WARN("Unsupported encoding format.");
        delete []blocks;
        return false;
      }
    }
  } else {
    js_string = std::string((const char*)js_code);
  }

  if(!isValidJsonString(js_string.c_str())) {
    ROS_WARN("Incorrect json data.");
    delete []blocks;
    return false;
  }

  bool ret = gige_2_0_load_default_parameters(hDevice, js_string.c_str());
  if(ret)  
    ROS_INFO("Loading default parameters successfully!");
  else
    ROS_WARN("Failed to load default parameters, some parameters cannot be loaded properly!");
    
  delete []blocks;
  return ret;
}

bool GigE_2_0::try_xml_parameter(const std::string& source, const std::string& feat, const std::string& str_val)
{
  TY_STATUS status = TY_STATUS_OK;
  TY_COMPONENT_ID comp = 0;
  if(source == "Depth")
    comp = TY_COMPONENT_DEPTH_CAM;
  else if(source == "Texture")
    comp = TY_COMPONENT_RGB_CAM;
  else if(source == "Left")
    comp = TY_COMPONENT_IR_CAM_LEFT;
  else if(source == "Right")
    comp = TY_COMPONENT_IR_CAM_RIGHT;
  else if(source == "Device")
    comp = TY_COMPONENT_DEVICE;
  else if(source == "Laser")
    comp = TY_COMPONENT_LASER;
  else {
    ROS_WARN("Incorrect source name: %s", source.c_str());
    return false;
  }

  if(!(ids & comp)) {
    ROS_WARN("Unsupported source name: %s", source.c_str());
    return false;
  }

  auto feature = m_gige_2_0_feature_map.find(feat);
  if (feature == m_gige_2_0_feature_map.end()) {
    ROS_WARN("Unsupported feature name: %s", feat.c_str());
    return false;
  }

  TY_FEATURE_ID feat_id = feature->second;
  TY_FEATURE_TYPE type = TYFeatureType(feat_id);
  
  switch(type) {
    case TY_FEATURE_INT:
    {
      int32_t val = atoi(str_val.c_str());
      status = TYSetInt(hDevice, comp, feat_id, val);
      break;
    }

    case TY_FEATURE_ENUM:
    {
      uint32_t val = static_cast<uint32_t>(atoi(str_val.c_str()));
      status = TYSetEnum(hDevice, comp, feat_id, val);
      break;
    }

    case TY_FEATURE_FLOAT:
    {
      float val = static_cast<float>(atof(str_val.c_str()));
      status = TYSetFloat(hDevice, comp, feat_id, val);
      break;
    }

    case TY_FEATURE_BOOL:
    {
      bool val = static_cast<bool>(atof(str_val.c_str()));
      status = TYSetBool(hDevice, comp, feat_id, val);
      break;
    }
    case TY_FEATURE_STRING:
    {
      status = TYSetString(hDevice, comp, feat_id, str_val.c_str());
      break;
    }
    case TY_FEATURE_STRUCT:
    {
      if(feat_id == TY_STRUCT_AEC_ROI) {
        if(feat == "AutoFunctionAOIOffsetX") {
          aec_roi.x = static_cast<uint32_t>(atoi(str_val.c_str()));
        } else if(feat == "AutoFunctionAOIOffsetY") {
          aec_roi.y = static_cast<uint32_t>(atoi(str_val.c_str()));
        } else if(feat == "AutoFunctionAOIWidth") {
          aec_roi.w = static_cast<uint32_t>(atoi(str_val.c_str()));
        } else if(feat == "AutoFunctionAOIHeight") {
          aec_roi.h = static_cast<uint32_t>(atoi(str_val.c_str()));
          status = TYSetStruct(hDevice, comp, feat_id, &aec_roi, sizeof(aec_roi));
        } else {
          status = TY_STATUS_INVALID_FEATURE;
          ROS_WARN("Unsupported feature type: %s(%d)", feat.c_str(), feat_id);
        }
      } else {
        status = TY_STATUS_INVALID_FEATURE;
        ROS_WARN("Unsupported feature type: %s(%d)", feat.c_str(), feat_id);
      }
      break;
    }
    default:
    {
      ROS_WARN("Unsupported feature type: %s(%d)", feat.c_str(), feat_id);
      return false;
    }
  }

  if(status) {
    ROS_WARN("Xml parameter(%s/%s(0x%08x)) init failed(%s(%d))!", source.c_str(), feat.c_str(), feat_id, 
          TYErrorString(status), status);
    return false;
  }

  return true;
}

}