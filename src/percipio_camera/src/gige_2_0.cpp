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

static bool isValidJsonString(const char* code)
{
  std::string err;
  const auto json = Json::parse(code, err);
  if(json.is_null()) return false;
  return true;
}

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

  if(status & TY_COMPONENT_DEPTH_CAM) 
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

TY_STATUS GigE_2_0::AcquisitionInit()
{
  if(!(ids & TY_COMPONENT_DEVICE))
    return TY_STATUS_OK;

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
    LOGD("The CRC check code is empty.");
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
          LOGE("Storage data length error.");
          delete []blocks;
          return false;
        }
                
        crc = crc32_bitwise(huffman_ptr, huffman_size);
        if(crc_data != crc) {
          LOGE("Storage area data check failed (check code error).");
          delete []blocks;
          return false;
        }

        std::string huffman_string(huffman_ptr, huffman_ptr + huffman_size);
        if(!TextHuffmanDecompression(huffman_string, js_string)) {
          LOGE("Huffman decompression error.");
          delete []blocks;
          return false;
        }
        break;
      }
      default:
      {
        LOGE("Unsupported encoding format.");
        delete []blocks;
        return false;
      }
    }
  } else {
    js_string = std::string((const char*)js_code);
  }

  if(!isValidJsonString(js_string.c_str())) {
    LOGE("Incorrect json data.");
    delete []blocks;
    return false;
  }

  bool ret = json_parse(hDevice, js_string.c_str());
  if(ret)  
    LOGD("Loading default parameters successfully!");
  else
    LOGD("Failed to load default parameters, some parameters cannot be loaded properly!");
    
  delete []blocks;
  return ret;
}

}