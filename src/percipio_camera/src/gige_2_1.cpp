#include <boost/algorithm/string.hpp>
#include "percipio_camera/gige_2_1.h"

namespace percipio 
{

TYPixFmt Mono[] = {
  TYPixelFormatMono8,
  TYPixelFormatMono10,
  TYPixelFormatPacketMono10,
  TYPixelFormatMono12,
  TYPixelFormatPacketMono12,
  TYPixelFormatMono14,
  TYPixelFormatMono16,
  TYPixelFormatTofIRFourGroupMono16
};
  
TYPixFmt Bayer[] = {
  TYPixelFormatBayerGBRG8,
  TYPixelFormatBayerBGGR8,
  TYPixelFormatBayerGRBG8,
  TYPixelFormatBayerRGGB8,
  TYPixelFormatPacketBayerGBRG10,
  TYPixelFormatPacketBayerBGGR10,
  TYPixelFormatPacketBayerGRBG10,
  TYPixelFormatPacketBayerRGGB10,
  TYPixelFormatPacketBayerGBRG12,
  TYPixelFormatPacketBayerBGGR12,
  TYPixelFormatPacketBayerGRBG12,
  TYPixelFormatPacketBayerRGGB12,
  TYPixelFormatBayerGBRG10,
  TYPixelFormatBayerBGGR10,
  TYPixelFormatBayerGRBG10,
  TYPixelFormatBayerRGGB10,
  TYPixelFormatBayerGBRG12,
  TYPixelFormatBayerBGGR12,
  TYPixelFormatBayerGRBG12,
  TYPixelFormatBayerRGGB12,
  TYPixelFormatBayerGBRG14,
  TYPixelFormatBayerBGGR14,
  TYPixelFormatBayerGRBG14,
  TYPixelFormatBayerRGGB14,
  TYPixelFormatBayerGBRG16,
  TYPixelFormatBayerBGGR16,
  TYPixelFormatBayerGRBG16,
  TYPixelFormatBayerRGGB16
};
  
TYPixFmt Bgr[] = {
  TYPixelFormatRGB8,
  TYPixelFormatBGR8,
};

TYPixFmt Yuv[] = {
  TYPixelFormatYUV422_8,
  TYPixelFormatYUV422_8_UYVY,

  TYPixelFormatYCbCr420_8_YY_CbCr_Planar,
  TYPixelFormatYCbCr420_8_YY_CrCb_Planar,

  TYPixelFormatYCbCr420_8_YY_CbCr_Semiplanar,
  TYPixelFormatYCbCr420_8_YY_CrCb_Semiplanar
};

TYPixFmt Jpeg[] = {
  TYPixelFormatJPEG,
};

TYPixFmt D16[] = {
  TYPixelFormatCoord3D_C16,
};

TYPixFmt D48[] = {
  TYPixelFormatCoord3D_ABC16,
};

TYPixFmt P3d[] = {
  TYPixelFormatCoord3D_ABC32f,
};

extern std::map<SensorType, std::string> SensorDesc;

const static int32_t m_Source_Range = 0;
const static int32_t m_Source_Color = 1;
const static int32_t m_Source_LeftIR = 2;
const static int32_t m_Source_RightIR = 3;
static inline int SensorTypeToSourceIdx(const SensorType type)
{
    int32_t index = -1;
    switch(type) {
    case SENSOR_DEPTH:
        index = m_Source_Range;
        break;
    case SENSOR_COLOR:
        index = m_Source_Color;
        break;
    case SENSOR_IR_LEFT:
        index = m_Source_LeftIR;
        break;
    case SENSOR_IR_RIGHT:
        index = m_Source_RightIR;
        break;
    default:
        break;
    }
    return index;
}

bool MatchPixelFormat(const std::string& fmt_desc, const TYPixFmt& m_fmt)
{
  std::string str_fmt = boost::algorithm::to_lower_copy(fmt_desc);
  if(str_fmt == "mono") {
    for(size_t i = 0; i < sizeof(Mono) / sizeof(TYPixFmt); i++) {
      if(Mono[i] == m_fmt) return true;
    }
  }

  if(str_fmt == "bayer") {
    for(size_t i = 0; i < sizeof(Bayer) / sizeof(TYPixFmt); i++) {
      if(Bayer[i] == m_fmt) return true;
    }
  }

  if(str_fmt == "bgr") {
    for(size_t i = 0; i < sizeof(Bgr) / sizeof(TYPixFmt); i++) {
      if(Bgr[i] == m_fmt) return true;
    }
  }

  if(str_fmt == "yuv") {
    for(size_t i = 0; i < sizeof(Yuv) / sizeof(TYPixFmt); i++) {
      if(Yuv[i] == m_fmt) return true;
    }
  }

  if(str_fmt == "jpeg" || str_fmt == "jpg") {
    for(size_t i = 0; i < sizeof(Jpeg) / sizeof(TYPixFmt); i++) {
      if(Jpeg[i] == m_fmt) return true;
    }
  }

  if(str_fmt == "depth16") {
    for(size_t i = 0; i < sizeof(D16) / sizeof(TYPixFmt); i++) {
      if(D16[i] == m_fmt) return true;
    }
  }

  if(str_fmt == "xyz48") {
    for(size_t i = 0; i < sizeof(D48) / sizeof(TYPixFmt); i++) {
      if(D48[i] == m_fmt) return true;
    }
  }

  if(str_fmt == "p3d") {
    for(size_t i = 0; i < sizeof(P3d) / sizeof(TYPixFmt); i++) {
      if(P3d[i] == m_fmt) return true;
    }
  }

  return false;
}

GigE_2_1::GigE_2_1(const TY_DEV_HANDLE dev) : GigEBase(dev) 
{
  TY_STATUS ret = TYEnumSetString(hDevice, "SourceSelector", "Range");
  if(ret) {
    need_depth_undistortion = false;
    return;
  }

  TY_ACCESS_MODE _access;
  ret = TYParamGetAccess(hDevice, "Distortion", &_access);
  if(ret) {
    need_depth_undistortion = false;
    return;
  }

  if(_access & TY_ACCESS_READABLE)
    need_depth_undistortion = true;
  else
    need_depth_undistortion = false;
}

std::vector<VideoMode> GigE_2_1::getLeftIRVideoModeList()
{
  videos[SENSOR_IR_LEFT].clear();
  TY_STATUS ret = TYEnumSetValue(hDevice, "SourceSelector", m_Source_LeftIR);
  if(ret) return videos[SENSOR_IR_LEFT];

  auto modes = dump_source_image_mode();
  for(size_t i = 0; i < modes.size(); i++) {
    uint32_t fmt = modes[i].pixel_fmt;
    uint32_t width = modes[i].width;
    uint32_t height = modes[i].height;
    videos[SENSOR_IR_LEFT].push_back(VideoMode(fmt, width, height));
  }
  return videos[SENSOR_IR_LEFT];
}

std::vector<VideoMode> GigE_2_1::getRightIRVideoModeList()
{
  videos[SENSOR_IR_RIGHT].clear();
  TY_STATUS ret = TYEnumSetValue(hDevice, "SourceSelector", m_Source_RightIR);
  if(ret) return videos[SENSOR_IR_RIGHT];

  auto modes = dump_source_image_mode();
  for(size_t i = 0; i < modes.size(); i++) {
    uint32_t fmt = modes[i].pixel_fmt;
    uint32_t width = modes[i].width;
    uint32_t height = modes[i].height;
    videos[SENSOR_IR_RIGHT].push_back(VideoMode(fmt, width, height));
  }
  return videos[SENSOR_IR_RIGHT];
}

std::vector<VideoMode> GigE_2_1::getColorVideoModeList()
{
  videos[SENSOR_COLOR].clear();
  TY_STATUS ret = TYEnumSetValue(hDevice, "SourceSelector", m_Source_Color);
  if(ret) return videos[SENSOR_COLOR];

  auto modes = dump_source_image_mode();
  for(size_t i = 0; i < modes.size(); i++) {
    uint32_t fmt = modes[i].pixel_fmt;
    uint32_t width = modes[i].width;
    uint32_t height = modes[i].height;
    videos[SENSOR_COLOR].push_back(VideoMode(fmt, width, height));
  }
  return videos[SENSOR_COLOR];
}

std::vector<VideoMode> GigE_2_1::getDepthVideoModeList()
{
  videos[SENSOR_DEPTH].clear();
  TY_STATUS ret = TYEnumSetValue(hDevice, "SourceSelector", m_Source_Range);
  if(ret) return videos[SENSOR_DEPTH];

  auto modes = dump_source_image_mode();
  for(size_t i = 0; i < modes.size(); i++) {
    uint32_t fmt = modes[i].pixel_fmt;
    uint32_t width = modes[i].width;
    uint32_t height = modes[i].height;
    videos[SENSOR_DEPTH].push_back(VideoMode(fmt, width, height));
  }
  return videos[SENSOR_DEPTH];
}

TY_STATUS GigE_2_1::getColorCalibData(TY_CAMERA_CALIB_INFO& calib_data)
{
  TY_STATUS ret = TYEnumSetValue(hDevice, "SourceSelector", m_Source_Color);
  if(ret) return ret;

  return getCalibData(calib_data);
}

TY_STATUS GigE_2_1::getDepthCalibData(TY_CAMERA_CALIB_INFO& calib_data)
{
  TY_STATUS ret = TYEnumSetValue(hDevice, "SourceSelector", m_Source_Range);
  if(ret) return ret;

  return getCalibData(calib_data);
}

TY_STATUS GigE_2_1::getColorIntrinsic(TY_CAMERA_INTRINSIC& Intrinsic)
{
  return TYGetStruct(hDevice, TY_COMPONENT_RGB_CAM, TY_STRUCT_CAM_INTRINSIC, &Intrinsic, sizeof(Intrinsic));
}

TY_STATUS GigE_2_1::getDepthIntrinsic(TY_CAMERA_INTRINSIC& Intrinsic)
{
  return TYGetStruct(hDevice, TY_COMPONENT_DEPTH_CAM, TY_STRUCT_CAM_INTRINSIC, &Intrinsic, sizeof(Intrinsic));
}

TY_STATUS GigE_2_1::AcquisitionInit()
{
  TY_STATUS status = TYEnumSetString(hDevice, "AcquisitionMode", "Continuous");
  if(status != TY_STATUS_OK) {
    ROS_WARN("Set AcquisitionMode error : %d", status);
  } else {
    TY_ACCESS_MODE _access = 0;
    status = TYParamGetAccess(hDevice, "AcquisitionFrameRateEnable", &_access);
    if((status == TY_STATUS_OK) && (_access & TY_ACCESS_WRITABLE)) {
      status = TYBooleanSetValue(hDevice, "AcquisitionFrameRateEnable", false);
      if(status != TY_STATUS_OK)
        ROS_WARN("Disable AcquisitionFrameRateEnable error : %d", status);
    }
  }
  return status;
}

TY_STATUS GigE_2_1::SetImageMode(SensorType sensorType, int width, int height, const std::string& fmt)
{
  auto it = videos.find(sensorType);
  if (it == videos.end())
    return TY_STATUS_INVALID_COMPONENT;
  
  int sel = SensorTypeToSourceIdx(sensorType);
  if(sel < 0) {
    ROS_WARN("Invalid stream source id : %d", sel);
    return TY_STATUS_INVALID_COMPONENT;
  }

  TY_STATUS ret = TYEnumSetValue(hDevice, "SourceSelector", sel);
  if(ret) {
    ROS_WARN("Invalid source id : %d, error code : %d", sel, ret);
    return ret;
  }

  int64_t m_sensor_w, m_sensor_h;
  ret = TYIntegerGetValue(hDevice, "SensorWidth", &m_sensor_w);
  if(ret) {
    ROS_WARN("get sensor width failed, error code : %d", ret);
    return ret;
  }

  ret = TYIntegerGetValue(hDevice, "SensorHeight", &m_sensor_h);
  if(ret) {
    ROS_WARN("get sensor height failed, error code : %d", ret);
    return ret;
  }

  if(fmt.length()) {
    for(size_t i = 0; i < videos[sensorType].size(); i++) {
      int m_width = videos[sensorType][i].getResolutionX();
      int m_height = videos[sensorType][i].getResolutionY();
      int m_pixFmt = videos[sensorType][i].getPixelFormat();
      if((width == m_width) && MatchPixelFormat(fmt, m_pixFmt)) {
        ret = TYEnumSetValue(hDevice, "PixelFormat", m_pixFmt);
        if(ret != TY_STATUS_OK) continue;
        
        ret = TYEnumSetValue(hDevice, "BinningHorizontal", m_sensor_w / m_width);
        if(ret == TY_STATUS_OK) {
          current_video_mode[sensorType] = videos[sensorType][i];
          ROS_INFO("Set %s image mode : %d x %d, fmt: 0x%08x", SensorDesc[sensorType].c_str(), width, height, m_pixFmt);
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
        ret = TYEnumSetValue(hDevice, "PixelFormat", m_pixFmt);
        if(ret != TY_STATUS_OK) continue;

        ret = TYEnumSetValue(hDevice, "BinningHorizontal", m_sensor_w / m_width);
        if(ret == TY_STATUS_OK) {
          current_video_mode[sensorType] = videos[sensorType][i];
          ROS_INFO("Set %s image mode : %d x %d, fmt: 0x%08x", SensorDesc[sensorType].c_str(), width, height, m_pixFmt);
          return TY_STATUS_OK;
        }
      }
    }
  }

  ROS_WARN("Incorrect image mode : 0x%08x targte size: %d x %d - format(%s), using default value.", SensorDesc[sensorType].c_str(), width, height, fmt.c_str());

  int32_t default_fmt = 0;
  ret = TYEnumGetValue(hDevice, "PixelFormat", &default_fmt);
  if(ret) {
    ROS_WARN("Read default image mode : %s failed!", SensorDesc[sensorType].c_str());
    return ret;
  }

  int32_t binning = 0;
  ret = TYEnumGetValue(hDevice, "BinningHorizontal", &binning);
  if(ret) {
    ROS_WARN("Read default image bining : %s failed!", SensorDesc[sensorType].c_str());
    return ret;
  }

  current_video_mode[sensorType] = VideoMode(default_fmt, m_sensor_w / binning, m_sensor_h / binning);
  ROS_INFO("Set %s image mode : %d x %d, fmt: 0x%08x", SensorDesc[sensorType].c_str(), m_sensor_w / binning, m_sensor_h / binning, default_fmt);
  return TY_STATUS_OK;
}

TY_STATUS GigE_2_1::PreSetting()
{
  uint32_t cnt;
  std::vector<TYEnumEntry> entrys;
  TY_STATUS ret = TYEnumGetEntryCount(hDevice, "SourceSelector", &cnt);
  if(ret) {
    ROS_WARN("SourceSelector ERR: %d(%d)", ret, __LINE__);
    return ret;
  }

  entrys.resize(cnt);
  ret = TYEnumGetEntryInfo(hDevice, "SourceSelector", &entrys[0], cnt, &cnt);
  if(ret) {
    ROS_WARN("SourceSelector ERR: %d(%d)", ret, __LINE__);
    return ret;
  }

  for(size_t i = 0; i < cnt; i++) {
    ret = TYEnumSetValue(hDevice, "SourceSelector", entrys[i].value);
    if(ret) {
      ROS_WARN("SourceSelector set failed: %d(%d)", ret, __LINE__);
      continue;
    }

    TY_ACCESS_MODE _access = 0;
    TYParamGetAccess(hDevice, "ComponentEnable", &_access);
    if(_access & TY_ACCESS_WRITABLE) {
      TYBooleanSetValue(hDevice, "ComponentEnable", false);
    }
  }
  return TY_STATUS_OK;
}

TY_STATUS GigE_2_1::EnableColorStream(const bool en)
{
  TY_STATUS ret = TYEnumSetValue(hDevice, "SourceSelector", 1);
  if(ret) {
    ROS_WARN("SourceSelector set failed: %d(%d)", ret, __LINE__);
    return ret;
  }

  TY_ACCESS_MODE _access = 0;
  ret = TYParamGetAccess(hDevice, "ComponentEnable", &_access);
  if(_access & TY_ACCESS_WRITABLE) {
    if(en) {
      ret = TYBooleanSetValue(hDevice, "ComponentEnable", true);
      if(ret) ROS_WARN("Enable source rgb failed: %d", ret);
    } else {
      ret = TYBooleanSetValue(hDevice, "ComponentEnable", false);
      if(ret) ROS_WARN("Disable source rgb failed: %d", ret);
    }
  } else {
    ROS_WARN("Device not support source rgb :%d", ret);
  }
  return ret;
}

TY_STATUS GigE_2_1::EnableDepthStream(const bool en)
{
  TY_STATUS ret = TYEnumSetValue(hDevice, "SourceSelector", 0);
  if(ret) {
    ROS_WARN("SourceSelector set failed: %d(%d)", ret, __LINE__);
    return ret;
  }

  TY_ACCESS_MODE _access = 0;
  ret = TYParamGetAccess(hDevice, "ComponentEnable", &_access);
  if(_access & TY_ACCESS_WRITABLE) {
    if(en) {
      ret = TYBooleanSetValue(hDevice, "ComponentEnable", true);
      if(ret) ROS_WARN("Enable source range failed: %d", ret);
    } else {
      ret = TYBooleanSetValue(hDevice, "ComponentEnable", false);
      if(ret) ROS_WARN("Disable source range failed: %d", ret);
    }
  } else {
    ROS_WARN("Device not support source range :%d", ret);
  }
  return ret;
}

TY_STATUS GigE_2_1::EnableLeftIRStream(const bool en)
{
  TY_STATUS ret = TYEnumSetValue(hDevice, "SourceSelector", 2);
  if(ret) {
    ROS_WARN("SourceSelector set failed: %d(%d)", ret, __LINE__);
    return ret;
  }

  TY_ACCESS_MODE _access = 0;
  ret = TYParamGetAccess(hDevice, "ComponentEnable", &_access);
  if(_access & TY_ACCESS_WRITABLE) {
    if(en) {
      ret = TYBooleanSetValue(hDevice, "ComponentEnable", true);
      if(ret) ROS_WARN("Enable source binocular-left failed: %d", ret);
    } else {
      ret = TYBooleanSetValue(hDevice, "ComponentEnable", false);
      if(ret) ROS_WARN("Disable source binocular-left failed: %d", ret);
    }
  } else {
    ROS_WARN("Device not support source binocular-left :%d", ret);
  }
  return ret;
}

TY_STATUS GigE_2_1::EnableRightIRStream(const bool en)
{
  TY_STATUS ret = TYEnumSetValue(hDevice, "SourceSelector", 2);
  if(ret) {
    ROS_WARN("SourceSelector set failed: %d(%d)", ret, __LINE__);
    return ret;
  }

  TY_ACCESS_MODE _access = 0;
  ret = TYParamGetAccess(hDevice, "ComponentEnable", &_access);
  if(_access & TY_ACCESS_WRITABLE) {
    if(en) {
      ret = TYBooleanSetValue(hDevice, "ComponentEnable", true);
      if(ret) ROS_WARN("Enable source binocular-right failed: %d", ret);
    } else {
      ret = TYBooleanSetValue(hDevice, "ComponentEnable", false);
      if(ret) ROS_WARN("Disable source binocular-right failed: %d", ret);
    }
  } else {
    ROS_WARN("Device not support source binocular-right :%d", ret);
  }
  return ret;
}

TY_STATUS GigE_2_1::getCalibData(TY_CAMERA_CALIB_INFO& calib_data)
{
  int64_t intrinsicWidth = 0;
  int64_t intrinsicHeight = 0;
  TY_STATUS ret = TYIntegerGetValue(hDevice, "IntrinsicWidth", &intrinsicWidth);
  if(TY_STATUS_OK == ret) calib_data.intrinsicWidth = intrinsicWidth;

  ret = TYIntegerGetValue(hDevice, "IntrinsicHeight", &intrinsicHeight);
  if(TY_STATUS_OK == ret) calib_data.intrinsicHeight = intrinsicHeight;

  double intrinsic[9];
  ret = TYByteArrayGetValue(hDevice, "Intrinsic", reinterpret_cast<uint8_t*>(intrinsic), sizeof(intrinsic));
  if(TY_STATUS_OK == ret) {
    for(int32_t i = 0; i < 9; i++)
      calib_data.intrinsic.data[i] = static_cast<float>(intrinsic[i]);
  }

  double distortion[12];
  ret = TYByteArrayGetValue(hDevice, "Distortion", reinterpret_cast<uint8_t*>(distortion), sizeof(distortion));
  if(TY_STATUS_OK == ret) {
    for(int32_t i = 0; i < 12; i++) 
      calib_data.distortion.data[i] = static_cast<float>(distortion[i]);
  }

  double extrinsic[16];
  ret = TYByteArrayGetValue(hDevice, "Extrinsic", reinterpret_cast<uint8_t*>(extrinsic), sizeof(extrinsic));
  if(TY_STATUS_OK == ret) {
    for(int32_t i = 0; i < 16; i++) 
      calib_data.extrinsic.data[i] = static_cast<float>(extrinsic[i]);
  }

  return TY_STATUS_OK;
}

std::vector<GegE2ImageMode> GigE_2_1::dump_source_image_mode()
{
  std::vector<GegE2ImageMode> image_mode_list;
  int64_t m_sensor_w, m_sensor_h;
  TY_STATUS ret = TYIntegerGetValue(hDevice, "SensorWidth", &m_sensor_w);
  if(ret) {
    ROS_WARN("Read sensor width failed: %d", ret);
    return image_mode_list;
  }

  ret = TYIntegerGetValue(hDevice, "SensorHeight", &m_sensor_h);
  if(ret) {
    ROS_WARN("Read sensor height failed: %d", ret);
    return image_mode_list;
  }

  int32_t old_fmt = 0;
  ret = TYEnumGetValue(hDevice, "PixelFormat", &old_fmt);
  if(ret) ROS_WARN("read stream default fmt failed : %d",ret);
  
  int32_t old_binnng = 0;
  ret = TYEnumGetValue(hDevice, "BinningHorizontal", &old_binnng);
  if(ret) ROS_WARN("read stream default binning failed : %d",ret);

  uint32_t PixFmtCnt = 0;
  std::vector<TYEnumEntry> PixelFormatList;
  TYEnumGetEntryCount(hDevice, "PixelFormat", &PixFmtCnt);
  if(PixFmtCnt > 0) {
    PixelFormatList.resize(PixFmtCnt);
    TYEnumGetEntryInfo( hDevice, "PixelFormat", &PixelFormatList[0], PixFmtCnt, &PixFmtCnt);
  }

  for(size_t i = 0; i < PixFmtCnt; i++) {
    ret = TYEnumSetValue(hDevice, "PixelFormat", PixelFormatList[i].value);
    if(ret) {
      ROS_WARN("Set pixel fmt 0x%08x failed : %d", PixelFormatList[i].value, ret);
      continue;
    }

    uint32_t BinningCnt = 0;
    std::vector<TYEnumEntry> BinningList;
    TYEnumGetEntryCount(hDevice, "BinningHorizontal", &BinningCnt);
    if(BinningCnt > 0) {
      BinningList.resize(BinningCnt);
      TYEnumGetEntryInfo( hDevice, "BinningHorizontal", &BinningList[0], BinningCnt, &BinningCnt);
    }

    for(size_t j = 0; j < BinningCnt; j++) {
      uint32_t img_width = static_cast<uint32_t>(m_sensor_w / BinningList[j].value);
      uint32_t img_height = static_cast<uint32_t>(m_sensor_h / BinningList[j].value);
      image_mode_list.push_back({img_width, img_height, static_cast<uint32_t>(BinningList[j].value), static_cast<uint32_t>(PixelFormatList[i].value)});
    }
  }

  TYEnumSetValue(hDevice, "PixelFormat", old_fmt);
  TYEnumSetValue(hDevice, "BinningHorizontal", old_binnng);    

  return image_mode_list;
}

}