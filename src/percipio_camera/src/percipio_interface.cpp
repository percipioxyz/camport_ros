/*
 * @Description: 
 * @Author: zxy
 * @Date: 2023-08-09 09:11:59
 * @LastEditors: zxy
 * @LastEditTime: 2024-05-30 14:59:24
 */
#include "percipio_camera/percipio_interface.h"
#include "percipio_camera/image_process.hpp"

#include "percipio_camera/gige_2_0.h"
#include "percipio_camera/gige_2_1.h"

#include "percipio_camera/DepthStreamProc.h"
#include "percipio_camera/percipio_depth_algorithm.h"

#define MAX_STORAGE_SIZE    (10*1024*1024)
namespace percipio
{
  std::map<SensorType, std::string> SensorDesc = {
    {SENSOR_IR_LEFT, "Left-IR"},
    {SENSOR_IR_RIGHT, "Right-IR"},
    {SENSOR_COLOR, "Color"},
    {SENSOR_DEPTH, "Depth"},
  };

  void NewFrameCallbackManager::register_callback(void* listener, NewFrameCallback callback)
  {
    frame_listener = listener;
    cb = callback;
  }

  void NewFrameCallbackManager::unregister_callback()
  {
    frame_listener = NULL;
    cb = NULL;
  }
  
  bool NewFrameCallbackManager::isValid()
  {
    if(frame_listener && cb && is_enable) return true;
    return false;
  }

  void NewFrameCallbackManager::enableCallback(bool en)
  {
    is_enable = en;
  }

  TY_STATUS PercipioDepthCam::initialize()
  {
    return TY_STATUS_OK;
  }

  PercipioDepthCam::PercipioDepthCam() : _M_IFACE(0), _M_DEVICE(0), mIDS(0)
  {
    frameBuffer[0] = NULL;
    frameBuffer[1] = NULL;
    isRuning = false;

    current_rgb_width = 0;
    current_rgb_height = 0;

    device_list.clear();

    DepthDomainTimeFilterMgrPtr = boost::make_shared<DepthTimeDomainMgr>(depth_time_domain_frame_cnt);

    TY_STATUS rc;
    rc = TYInitLib();

    TYImageProcesAcceEnable(false);
  }

  void PercipioDepthCam::GetDeviceList(DeviceInfo** device_info_ptr, int* cnt)
  {
    std::vector<TY_DEVICE_BASE_INFO> selected;
    TY_STATUS rc = selectDevice(TY_INTERFACE_ALL, "", "", 100, selected);
    if(rc == TY_STATUS_OK && selected.size()) {
      device_list.clear();
      for(size_t i = 0; i < selected.size(); i++) {

        if (TYIsNetworkInterface(selected[i].iface.type)) {
          device_list.push_back(DeviceInfo(selected[i].id, selected[i].vendorName, selected[i].modelName, selected[i].netInfo.ip));
        } else {
          TY_INTERFACE_HANDLE hIface;
          rc = TYOpenInterface(selected[i].iface.id, &hIface);
          if(rc == TY_STATUS_OK) {
            TY_DEV_HANDLE handle;
            int32_t ret = TYOpenDevice(hIface, selected[i].id, &handle);
            if (ret == 0) {
              TY_DEVICE_BASE_INFO dev;
              memset(&dev, 0, sizeof(TY_DEVICE_BASE_INFO));

              TYGetDeviceInfo(handle, &dev);
              TYCloseDevice(handle);

              if (strlen(dev.userDefinedName) != 0) {
                device_list.push_back(DeviceInfo(selected[i].id, dev.userDefinedName, dev.modelName, PERCIPIO_USB_PID, PERCIPIO_USB_VID));
              } else {
                //LOGD("          vendor     : %s", dev.vendorName);
                device_list.push_back(DeviceInfo(selected[i].id, dev.vendorName, dev.modelName, PERCIPIO_USB_PID, PERCIPIO_USB_VID));
              }
            }
            TYCloseInterface(hIface);

          }
        }
      }
    }
    
    *cnt = device_list.size();
    *device_info_ptr = new DeviceInfo[device_list.size()];
    for(size_t i = 0; i < device_list.size(); i++)
      *(*device_info_ptr + i) = device_list[i];
  }

  TY_STATUS PercipioDepthCam::DeviceInit()
  {
    TY_DEVICE_BASE_INFO info;
    TYGetDeviceInfo(_M_DEVICE, &info);

    bool isNetDev = TYIsNetworkInterface(info.iface.type);
    if(isNetDev) {
        std::string str_gige_version = info.netInfo.tlversion;
        if(str_gige_version == "Gige_2_1") {
            gige_version = GigeE_2_1;
        }
    }

    if(!m_gige_dev) {
      switch(gige_version) {
        case GigeE_2_1:
          m_gige_dev = boost::make_shared<GigE_2_1>(_M_DEVICE);
          break;
        default:
          m_gige_dev = boost::make_shared<GigE_2_0>(_M_DEVICE);
          break;
      }
    } else {
      m_gige_dev->hDevice = _M_DEVICE;
    }

    m_current_device_sn = info.id;

    TYGetComponentIDs(_M_DEVICE, &mIDS);
    if(mIDS & TY_COMPONENT_IR_CAM_LEFT) {
      leftIRVideoMode = m_gige_dev->getLeftIRVideoModeList();
    }

    if(mIDS & TY_COMPONENT_IR_CAM_RIGHT) {
      rightIRVideoMode = m_gige_dev->getRightIRVideoModeList();
    }

    if(mIDS & TY_COMPONENT_RGB_CAM) {
      RGBVideoMode = m_gige_dev->getColorVideoModeList();
      m_gige_dev->getColorCalibData(color_calib);
    }
    if(mIDS & TY_COMPONENT_DEPTH_CAM) {
      DepthVideoMode = m_gige_dev->getDepthVideoModeList();
      m_gige_dev->getDepthCalibData(depth_calib);
    }

    m_gige_dev->PreSetting();
    m_gige_dev->AcquisitionInit();
    
    if (TYIsNetworkInterface(info.iface.type)) {
      current_device_info = DeviceInfo(info.id, info.vendorName, info.modelName, info.netInfo.ip);
    } else {
      current_device_info = DeviceInfo(info.id, info.vendorName, info.modelName, PERCIPIO_USB_PID, PERCIPIO_USB_VID);
    }

    TYRegisterEventCallback(_M_DEVICE, eventCallback, this);

    if(!b_auto_reconnect) 
      return TY_STATUS_OK;
    
    if(!b_device_opened) {
      b_device_opened = true;
      pthread_create(&device_status_listen, NULL, device_offline_reconnect, this);
    }

    return TY_STATUS_OK;
  }

  TY_STATUS PercipioDepthCam::openWithSN(const char* sn)
  {
    TY_DEV_HANDLE deviceHandle;
    std::vector<TY_DEVICE_BASE_INFO> selected;
    std::string SN = std::string(sn);
    TY_STATUS rc = selectDevice(TY_INTERFACE_ALL, SN, "", 1, selected);
    if(!selected.size())
      return TY_STATUS_ERROR;

    TY_DEVICE_BASE_INFO& selectedDev = selected[0];
    for(size_t m = 0; m < selected.size(); m++) {
      selectedDev = selected[m];
      rc = TYOpenInterface(selectedDev.iface.id, &_M_IFACE);
      if(rc != TY_STATUS_OK) continue;
      rc = TYOpenDevice(_M_IFACE, selectedDev.id, &deviceHandle);
      if(rc == TY_STATUS_OK) break;

      ROS_INFO("TYOpenDevice err : %d", rc);
      TYCloseInterface(_M_IFACE);
    }

    if(rc != TY_STATUS_OK)
      return rc;

    
    ROS_INFO("Device %s is open!", selectedDev.id);

    _M_DEVICE = deviceHandle;

    return DeviceInit();
  }
  
  TY_STATUS PercipioDepthCam::openWithIP(const char* ip)
  {
    TY_DEV_HANDLE deviceHandle;
    std::vector<TY_DEVICE_BASE_INFO> selected;
    std::string IP = std::string(ip);
    TY_STATUS rc = selectDevice(TY_INTERFACE_ALL, "", IP, 1, selected);
    if(!selected.size())
      return TY_STATUS_ERROR;
    
    TY_DEVICE_BASE_INFO& selectedDev = selected[0];
    for(size_t m = 0; m < selected.size(); m++) {
      selectedDev = selected[m];
      rc = TYOpenInterface(selectedDev.iface.id, &_M_IFACE);
      if(rc != TY_STATUS_OK)
        continue;
      rc = TYOpenDevice(_M_IFACE, selectedDev.id, &deviceHandle);
      if(rc != TY_STATUS_OK) {
        TYCloseInterface(_M_IFACE);
        continue;
      }
      else
        break;
    }

    if(rc != TY_STATUS_OK)
      return rc;

    ROS_INFO("Device %s is open!", selectedDev.netInfo.ip);

    _M_DEVICE = deviceHandle;

    return DeviceInit();
  }

  void PercipioDepthCam::setDeviceInitCallback(boost::shared_ptr<initDeviceCallbackFunction>& callback)
  {
    ptrFuncDeviceInit = callback;
  }

  const DeviceInfo& PercipioDepthCam::get_current_device_info()
  {
    return current_device_info;
  }

  TY_STATUS PercipioDepthCam::set_image_mode(const SensorType type, const int width, const int height, const std::string& fmt)
  {
    return m_gige_dev->SetImageMode(type, width, height, fmt);
  }

  void PercipioDepthCam::eventCallback(TY_EVENT_INFO *event_info, void *userdata)
  {
    if (event_info->eventId == TY_EVENT_DEVICE_OFFLINE) 
    {
      LOGD("=== Event Callback: Device Offline!");
      for(size_t i = 0; i < cb_list.size(); i++) 
      {
        if(cb_list[i].cb == NULL) 
          continue;
        
        if(cb_list[i].cb->deviceDisconnected == NULL)
          continue;

        cb_list[i].cb->deviceDisconnected(&((PercipioDepthCam*)userdata)->current_device_info, cb_list[i].pListener);

        PercipioDepthCam::isOffline = true;

        std::unique_lock<std::mutex> lck( ((PercipioDepthCam*)userdata)->detect_mutex);
        ((PercipioDepthCam*)userdata)->detect_cond.notify_one();
      }
    }
  }

  TY_STATUS PercipioDepthCam::RegisterDeviceCallbacks(DeviceCallbacks* callback, void* listener)
  {
    pthread_mutex_lock(&m_mutex);
    for(size_t i = 0; i < cb_list.size(); i++) 
    {
      if(cb_list[i].pListener == listener) 
      {
        pthread_mutex_unlock(&m_mutex);
        return TY_STATUS_ERROR;
      }
    }
    cb_list.push_back({callback, listener});
    pthread_mutex_unlock(&m_mutex);
    return TY_STATUS_OK;
  }

  void PercipioDepthCam::UnregisterDeviceCallbacks(void* listener)
  {
    pthread_mutex_lock(&m_mutex);
    for(size_t i = 0; i < cb_list.size(); i++) {
      if(cb_list[i].pListener == listener) {
        cb_list.erase(cb_list.begin() + i);
        break;
      }
    }
    pthread_mutex_unlock(&m_mutex);
  }

  bool PercipioDepthCam::isValid() 
  {
    if (_M_DEVICE != NULL)
      return true;
    else
      return false;
  }

  TY_STATUS PercipioDepthCam::get_device_info(TY_DEVICE_BASE_INFO& info)
  {
    return TYGetDeviceInfo(_M_DEVICE, &info);
  }

  void PercipioDepthCam::close()
  {
    if (_M_DEVICE != NULL)
    {
      TYCloseDevice(_M_DEVICE);
      TYCloseInterface(_M_IFACE);
    }
    _M_DEVICE = NULL;
  }

  const uint32_t&  PercipioDepthCam::get_components() const 
  {
    return mIDS;
  }

  bool PercipioDepthCam::DeviceSetColorUndistortionEnable(bool enable)
  {
    color_undistortion = enable;
    return true;
  }

  bool PercipioDepthCam::DepthStreamSetSpeckFilterEn(bool enabled)
  {
    depth_stream_spec_enable = enabled;
    return true;
  }

  bool PercipioDepthCam::DepthStreamGetSpeckFilterEn()
  {
    return depth_stream_spec_enable;
  }

  bool PercipioDepthCam::DepthStreamSetSpeckFilterSpecSize(int spec_size)
  {
    depth_stream_spec_size = spec_size;
    return true;
  }

  int PercipioDepthCam::DepthStreamGetSpeckFilterSpecSize()
  {
    return depth_stream_spec_size;
  }

  bool PercipioDepthCam::DepthStreamSetSpeckFilterDiff(int spec_diff)
  {
    depth_stream_spec_diff = spec_diff;
    return true;
  }

  int PercipioDepthCam::DepthStreamGetSpeckFilterDiff()
  {
    return depth_stream_spec_diff;
  }

  bool PercipioDepthCam::DepthStreamSetTimeDomainFilterEn(bool enabled)
  {
    depth_time_domain_enable = enabled;
    DepthDomainTimeFilterMgrPtr->reset(depth_time_domain_frame_cnt);
    return true;
  }

  bool PercipioDepthCam::DepthStreamGetTimeDomainFilterEn()
  {
    return depth_time_domain_enable;
  }
  
  bool PercipioDepthCam::DepthStreamSetTimeDomainFilterFCnt(int frameCnt)
  {
    if(frameCnt > 1 && frameCnt <= 10) {
      depth_time_domain_frame_cnt = frameCnt;
      DepthDomainTimeFilterMgrPtr->reset(depth_time_domain_frame_cnt);
      return true;
    } else {
      ROS_WARN("Frame setting out of range (2-10) for time-domain filtering. Current value: %d is invalid, use default : %d",
           frameCnt, depth_time_domain_frame_cnt);
      return false;
    }
  }

  int  PercipioDepthCam::DepthStreamGetTimeDomainFilterFCnt()
  {
    return depth_time_domain_frame_cnt;
  }

  bool PercipioDepthCam::DeviceIsImageRegistrationModeSupported() const
  {
    if(!(mIDS & TY_COMPONENT_RGB_CAM)) {
      return false;
    }

    if(!(mIDS & TY_COMPONENT_DEPTH_CAM)) {
      return false;
    }

    return true;
  }
  
  TY_STATUS PercipioDepthCam::DeviceSetImageRegistrationMode(ImageRegistrationMode mode)
  {
    registration_mode = mode;
    return TY_STATUS_OK;
  }

  ImageRegistrationMode PercipioDepthCam::DeviceGetImageRegistrationMode()
  {
    if(DeviceIsImageRegistrationModeSupported()) {
      if(b_stream_with_color)
        return registration_mode;
      else
        return IMAGE_REGISTRATION_OFF;
    }
    else  
      return IMAGE_REGISTRATION_OFF;
  }

  TY_STATUS PercipioDepthCam::MapDepthFrameToColorCoordinate(VideoFrameData* src, VideoFrameData* dst)
  {
    return ImgProc::MapDepthImageToColorCoordinate(&depth_calib, &color_calib, current_rgb_width, current_rgb_height, src, dst, f_depth_scale_unit);
  }

  TY_STATUS PercipioDepthCam::MapXYZ48FrameToColorCoordinate(VideoFrameData* src, VideoFrameData* dst)
  {
    return ImgProc::MapXYZ48ToColorCoordinate(&depth_calib, &color_calib, current_rgb_width, current_rgb_height, src, dst, f_depth_scale_unit);
  }

  TY_STATUS PercipioDepthCam::parseColorStream(VideoFrameData* src, VideoFrameData* dst)
  {
    TY_STATUS status = TY_STATUS_OK;
    if(color_undistortion)
      status = ImgProc::doRGBUndistortion(&color_calib, src, dst);
    else
      dst->clone(*src);
    return status;
  }

  TY_STATUS PercipioDepthCam::parseIrStream(VideoFrameData* src, VideoFrameData* dst)
  {
    dst->clone(*src);
    return TY_STATUS_OK;
  }

  TY_STATUS PercipioDepthCam::parseDepthStream(VideoFrameData* src, VideoFrameData* dst)
  {
    TY_STATUS status = TY_STATUS_OK;
    if(depth_distortion)
      status = ImgProc::doDepthUndistortion(&depth_calib, src, dst);
    else
      dst->clone(*src);
    return status;
  }

  TY_STATUS PercipioDepthCam::FrameDecoder(VideoFrameData& src, VideoFrameData& dst)
  {
    return ImgProc::cvtColor(src, dst);
  }

  TY_STATUS PercipioDepthCam::DeviceGetProperty(TY_COMPONENT_ID comp, TY_FEATURE_ID feat, void* ptr, size_t size)
  {
    bool has = false;
    TY_STATUS ret = TYHasFeature(_M_DEVICE, comp, feat, &has);
    if(ret || (!has)) {
      ROS_WARN("Invalid property(0x%08x 0x%08x) ret = %d", comp, feat, ret);
      return ret;
    }

    auto type = TYFeatureType(feat);
    switch(type) {
      case TY_FEATURE_INT:{
        if(size == sizeof(int32_t))
          ret = TYGetInt(_M_DEVICE, comp, feat, static_cast<int32_t*>(ptr));
        else {
          ret = TY_STATUS_OUT_OF_MEMORY;
        }
        break;
      }
      case TY_FEATURE_FLOAT:{
        if(size == sizeof(float))
          ret = TYGetFloat(_M_DEVICE, comp, feat, static_cast<float*>(ptr));
        else {
          ret = TY_STATUS_OUT_OF_MEMORY;
        }
        break;
      }
      case TY_FEATURE_ENUM:{
        if(size == sizeof(uint32_t))
          ret = TYGetEnum(_M_DEVICE, comp, feat, static_cast<uint32_t*>(ptr));
        else {
          ret = TY_STATUS_OUT_OF_MEMORY;
        }
        break;
      }
      case TY_FEATURE_BOOL:{
        if(size == sizeof(bool))
          ret = TYGetBool(_M_DEVICE, comp, feat, static_cast<bool*>(ptr));
        else {
          ret = TY_STATUS_OUT_OF_MEMORY;
        }
        break;
      }
      case TY_FEATURE_STRING:{
        ret = TYGetString(_M_DEVICE, comp, feat, static_cast<char*>(ptr), size);
        break;
      }
      case TY_FEATURE_BYTEARRAY:{
        ret = TYGetByteArray(_M_DEVICE, comp, feat, static_cast<uint8_t*>(ptr), size);
        break;
      }
      default:{
        ROS_WARN("Invalid feature type(0x%08x)", feat);
      }
    }
    return ret;
  }

  TY_STATUS PercipioDepthCam::DeviceSetProperty(TY_COMPONENT_ID comp, TY_FEATURE_ID feat, void* ptr, size_t size)
  {
    bool has = false;
    TY_STATUS ret = TYHasFeature(_M_DEVICE, comp, feat, &has);
    if(ret || (!has)) {
      ROS_WARN("Invalid property(0x%08x 0x%08x) ret = %d", comp, feat, ret);
      return ret;
    }

    auto type = TYFeatureType(feat);
    switch(type) {
      case TY_FEATURE_INT:{
        if(size == sizeof(int32_t))
          ret = TYSetInt(_M_DEVICE, comp, feat, *(static_cast<int32_t*>(ptr)));
        else {
          ret = TY_STATUS_OUT_OF_MEMORY;
        }
        break;
      }
      case TY_FEATURE_FLOAT:{
        if(size == sizeof(float))
          ret = TYSetFloat(_M_DEVICE, comp, feat, *(static_cast<float*>(ptr)));
        else {
          ret = TY_STATUS_OUT_OF_MEMORY;
        }
        break;
      }
      case TY_FEATURE_ENUM:{
        if(size == sizeof(uint32_t))
          ret = TYSetEnum(_M_DEVICE, comp, feat, *(static_cast<uint32_t*>(ptr)));
        else {
          ret = TY_STATUS_OUT_OF_MEMORY;
        }
        break;
      }
      case TY_FEATURE_BOOL:{
        if(size == sizeof(bool))
          ret = TYSetBool(_M_DEVICE, comp, feat, *(static_cast<bool*>(ptr)));
        else {
          ret = TY_STATUS_OUT_OF_MEMORY;
        }
        break;
      }
      case TY_FEATURE_STRING:{
        ret = TYSetString(_M_DEVICE, comp, feat, static_cast<char*>(ptr));
        break;
      }
      case TY_FEATURE_BYTEARRAY:{
        ret = TYSetByteArray(_M_DEVICE, comp, feat, static_cast<uint8_t*>(ptr), size);
        break;
      }
      default:{
        ROS_WARN("Invalid feature type(0x%08x)", feat);
      }
    }
    return ret;
  }

  const std::vector<VideoMode>& PercipioDepthCam::getVideoModeList(SensorType type) const 
  {
    switch (type)
    {
    case SENSOR_IR_LEFT:
      return leftIRVideoMode;
    case SENSOR_IR_RIGHT:
      return rightIRVideoMode;
    case SENSOR_COLOR:
      return RGBVideoMode;
    case SENSOR_DEPTH:
    default:
      return DepthVideoMode;
    }
  }

  TY_STATUS PercipioDepthCam::CreateStream(SensorType sensorType, StreamHandle& streamHandle)
  {
    switch(sensorType)
    {
      case SENSOR_IR_LEFT:
        return create_leftIR_stream(streamHandle);
      case SENSOR_IR_RIGHT:
        return create_rightIR_stream(streamHandle);
      case SENSOR_COLOR:
        return create_color_stream(streamHandle);
      case SENSOR_DEPTH:
        return create_depth_stream(streamHandle);
      case SENSOR_POINT3D:
        return create_point3d_stream(streamHandle);
      default:
        return TY_STATUS_INVALID_PARAMETER;
    }
  }

  VideoMode PercipioDepthCam::get_current_video_mode(StreamHandle stream)
  {
    uint32_t value = 0;
    SensorType type = get_stream_type(stream);
    return m_gige_dev->current_video_mode[type];
  }

  TY_STATUS PercipioDepthCam::StreamRegisterNewFrameCallback(StreamHandle stream, void* listener, NewFrameCallback cb)//, void* listener)
  {
    pthread_mutex_lock(&m_mutex);
    if(stream) static_cast<NewFrameCallbackManager*>(stream)->register_callback(listener, cb);
    pthread_mutex_unlock(&m_mutex);
    return TY_STATUS_OK;
  }

  void PercipioDepthCam::StreamUnregisterNewFrameCallback(StreamHandle stream)
  {
    pthread_mutex_lock(&m_mutex);
    if(stream) static_cast<NewFrameCallbackManager*>(stream)->unregister_callback();
    pthread_mutex_unlock(&m_mutex);
  }

  static float get_fps() {
    static clock_t fps_tm = 0;
    static int fps_counter = 0;
    struct timeval start;
    
    gettimeofday(&start, NULL);
    if(0 == fps_tm) {
        fps_tm = start.tv_sec * 1000 + start.tv_usec / 1000;
        return -1.0;
    }

    fps_counter++;

    int elapse = start.tv_sec * 1000 + start.tv_usec / 1000 - fps_tm;
    if(elapse < 5000)
    {
        return -1.0;
    }

    float v = 1000.0f * fps_counter / elapse;
    fps_tm = 0;
    fps_counter = 0;

    return v;
  }

  bool PercipioDepthCam::isOffline = false;
  bool PercipioDepthCam::b_device_opened = false;

  void* PercipioDepthCam::device_offline_reconnect(void* ptr)
  {
    PercipioDepthCam* cam = (PercipioDepthCam*)ptr;

    while(cam->b_device_opened) {
      std::unique_lock<std::mutex> lck(cam->detect_mutex);
      cam->detect_cond.wait(lck);
      cam->StreamStopAll();
      cam->close();
      while(true) {
        if(!cam->openWithSN(cam->m_current_device_sn.c_str())) {
          ROS_INFO("camera: %s  opened!", cam->m_current_device_sn.c_str());
          break;
        }

        ROS_INFO("camera: %s  failed, retry!", cam->m_current_device_sn.c_str());
        MSLEEP(1000);
      }
      
      if(cam->ptrFuncDeviceInit) (*(cam->ptrFuncDeviceInit))();

      PercipioDepthCam::isOffline = false;
      if(cam->HasStream()) {
        cam->StreamStart();
      }
    }

    return nullptr;
  }

  void* PercipioDepthCam::fetch_thread(void* ptr)
  {
    TY_STATUS rc;
    PercipioDepthCam* cam = (PercipioDepthCam*)ptr;
    const TY_DEV_HANDLE handle = cam->getCurrentDeviceHandle();
    while(cam->isRuning) {
      TY_FRAME_DATA frame;
      rc = TYFetchFrame(handle, &frame, 2000);
      if(rc == TY_STATUS_OK) {
        float fps = get_fps();
        if(fps > 0) ROS_INFO("fps: %.2f", fps);
        for (int i = 0; i < frame.validCount; i++){
          if (frame.image[i].status != TY_STATUS_OK) continue;

          if (frame.image[i].componentID == TY_COMPONENT_DEPTH_CAM){
            if(frame.image[i].pixelFormat == TYPixelFormatCoord3D_C16) {
              uint16_t* ptrDepth = static_cast<uint16_t*>(frame.image[i].buffer);
              int32_t PixsCnt = frame.image[i].width * frame.image[i].height;
              for(int32_t i = 0; i < PixsCnt; i++) {
                if(ptrDepth[i] == 0xFFFF) ptrDepth[i] = 0;
              }

              if(cam->depth_stream_spec_enable) {
                DepthSpkFilterPara param = {cam->depth_stream_spec_size, cam->depth_stream_spec_diff};
                TYDepthSpeckleFilter(frame.image[i], param);
              }

              if(cam->depth_time_domain_enable) {
                cam->DepthDomainTimeFilterMgrPtr->add_frame(frame.image[i]);
                if(!cam->DepthDomainTimeFilterMgrPtr->do_time_domain_process(frame.image[i])) {
                  ROS_WARN("Do Time-domain filter, drop frame!");
                  continue;
                }
              }
            }

            if(cam->DepthStream->isValid()) {
              cam->DepthStream->cb(cam->DepthStream.get(), cam->DepthStream->frame_listener, &frame.image[i]);
            }
            
            if(cam->Point3DStream->isValid()) {
              cam->Point3DStream->cb(cam->Point3DStream.get(), cam->Point3DStream->frame_listener, &frame.image[i]);
            }
          }
          else if(frame.image[i].componentID == TY_COMPONENT_RGB_CAM){
            if(cam->ColorStream->isValid()) {
              cam->ColorStream->cb(cam->ColorStream.get(), cam->ColorStream->frame_listener, &frame.image[i]);
            }
          }
          else if(frame.image[i].componentID == TY_COMPONENT_IR_CAM_LEFT){
            if(cam->leftIRStream->isValid()) {
              cam->leftIRStream->cb(cam->leftIRStream.get(), cam->leftIRStream->frame_listener, &frame.image[i]);
            }
          }
          else if(frame.image[i].componentID == TY_COMPONENT_IR_CAM_RIGHT){
            if(cam->rightIRStream->isValid()) {
              cam->leftIRStream->cb(cam->rightIRStream.get(), cam->leftIRStream->frame_listener, &frame.image[i]);
            }
          }
        }

        TYEnqueueBuffer(handle, frame.userBuffer, frame.bufferSize);
      }
    }
    
    return NULL;
  }

  bool PercipioDepthCam::HasStream()
  {
    if(leftIRStream && leftIRStream->isValid())
      return true;

    if(rightIRStream && rightIRStream->isValid())
      return true;

    if(ColorStream && ColorStream->isValid())
      return true;

    if(DepthStream && DepthStream->isValid())
      return true;
    
    if(Point3DStream && Point3DStream->isValid())
      return true;

    return false;
  }

  void PercipioDepthCam::StreamEnable(StreamHandle stream) {
    if(stream) static_cast<NewFrameCallbackManager*>(stream)->enableCallback(true);
  }

  void PercipioDepthCam::StreamDisable(StreamHandle stream) {
    if(stream) static_cast<NewFrameCallbackManager*>(stream)->enableCallback(false);
  }
  
  TY_STATUS PercipioDepthCam::StartCapture()
  {
    pthread_mutex_lock(&m_mutex);
    TY_STATUS rc = StreamStart();
    pthread_mutex_unlock(&m_mutex);
    return rc;
  }

  void PercipioDepthCam::StopCapture(StreamHandle stream)
  {
    pthread_mutex_lock(&m_mutex);
    StreamStop(stream);
    pthread_mutex_unlock(&m_mutex);
  }

  TY_STATUS PercipioDepthCam::StreamStart()
  {
    TY_STATUS rc;
    if(isRuning) StreamStopAll();
    
    if(!HasStream()) {
      ROS_WARN("device has no stream component!");
      return TY_STATUS_ERROR;
    }

    bool b_support_depth = false;
    bool b_support_point3d = false;

    std::string component_list;
    if(mIDS & TY_COMPONENT_IR_CAM_LEFT) {
      if(leftIRStream && leftIRStream->isValid()) {
        ROS_INFO("Enable Left-IR stream!");
        rc = m_gige_dev->EnableLeftIRStream(true);
        if(!rc) component_list += "leftIR ";
      } else {
        ROS_INFO("Disable Left-IR stream!");
        rc = m_gige_dev->EnableLeftIRStream(false);
      }
    }

    if(mIDS & TY_COMPONENT_IR_CAM_RIGHT) {
      if(rightIRStream && rightIRStream->isValid()){
        ROS_INFO("Enable Right-IR stream!");
        rc = m_gige_dev->EnableRightIRStream(true);
        if(!rc) component_list += "rightIR ";
      } else {
        ROS_INFO("Disable Right-IR stream!");
        rc = m_gige_dev->EnableRightIRStream(false);
      }
    }

    if(mIDS & TY_COMPONENT_RGB_CAM) {
      if(ColorStream && ColorStream->isValid()) {
        auto it = m_gige_dev->current_video_mode.find(SENSOR_COLOR);
        if (it != m_gige_dev->current_video_mode.end()) {
          current_rgb_width = it->second.getResolutionX();
          current_rgb_height = it->second.getResolutionY();
          ROS_INFO("Current color image size : %d x %d.", current_rgb_width, current_rgb_height);
        } else {
          ROS_WARN("Got current color image size failed!");
        }

        ROS_INFO("Enable color stream!");
        rc = m_gige_dev->EnableColorStream(true);
        if(!rc) component_list += "color ";

        m_gige_dev->getColorIntrinsic(color_intr);

        b_stream_with_color = true;
      } else {
        ROS_INFO("Disable color stream!");
        rc = m_gige_dev->EnableColorStream(false);

        current_rgb_width = 0;
        current_rgb_height = 0;
        b_stream_with_color = false;
      }
    }

    if(mIDS & TY_COMPONENT_DEPTH_CAM) {
      if(DepthStream && DepthStream->isValid())
        b_support_depth = true;
      if(Point3DStream && Point3DStream->isValid())
        b_support_point3d = true;
    
      if( b_support_depth || b_support_point3d ) {
        auto it = m_gige_dev->current_video_mode.find(SENSOR_DEPTH);
        if (it != m_gige_dev->current_video_mode.end()) {
          int current_depth_width = it->second.getResolutionX();
          int current_depth_height = it->second.getResolutionY();
          ROS_INFO("Current depth image size : %d x %d.", current_depth_width, current_depth_height);
        } else {
          ROS_WARN("Got current depth image size failed, use default size!");
        }

        depth_distortion = m_gige_dev->need_depth_undistortion;

        ROS_INFO("Enable depth stream!");
        rc = m_gige_dev->EnableDepthStream(true);
        if(!rc) component_list += "depth ";

        m_gige_dev->getDepthIntrinsic(depth_intr);

        TYGetFloat(_M_DEVICE, TY_COMPONENT_DEPTH_CAM, TY_FLOAT_SCALE_UNIT, &f_depth_scale_unit);
        ROS_INFO("Depth stream scale unit : %f", f_depth_scale_unit);
      } else {
        ROS_INFO("Disable depth stream!");
        rc = m_gige_dev->EnableDepthStream(false);
      }
    }

    ROS_INFO("Device components enabled: %s.", component_list.c_str());
    ROS_INFO("Color undistortion flag: %d", color_undistortion);
    ROS_INFO("Depth undistortion flag: %d", depth_distortion);
    ROS_INFO("RGBD alignment flag:     %d", DeviceGetImageRegistrationMode());

    uint32_t frameSize;
    TYGetFrameBufferSize(_M_DEVICE, &frameSize);
    if(frameBuffer[0]) delete []frameBuffer[0];
    if(frameBuffer[1]) delete []frameBuffer[1];
    frameBuffer[0] = new char[frameSize];
    frameBuffer[1] = new char[frameSize];

    TYEnqueueBuffer(_M_DEVICE, frameBuffer[0], frameSize);
    TYEnqueueBuffer(_M_DEVICE, frameBuffer[1], frameSize);

    rc = TYStartCapture(_M_DEVICE);
    if(rc != TY_STATUS_OK) {
      ROS_WARN("TYStartCapture got error code : %d!", rc);
      return rc;
    }
    
    isRuning = true;
    pthread_create(&frame_fetch_thread, NULL, fetch_thread, this);
    
    return TY_STATUS_OK;
  }

  void PercipioDepthCam::StreamStopAll()
  {
    if(isRuning) {
      isRuning = false;
      pthread_join(frame_fetch_thread, NULL);
    }
    TYStopCapture(_M_DEVICE);
    TYClearBufferQueue(_M_DEVICE);
    delete []frameBuffer[0];
    delete []frameBuffer[1];
    frameBuffer[0] = NULL;
    frameBuffer[1] = NULL;
  }

  void PercipioDepthCam::StreamStop(StreamHandle stream)
  {
    TY_COMPONENT_ID  comp = get_stream_component_id(stream);
    if(comp == 0)
      return;

    if(isRuning) {
      isRuning = false;
      pthread_join(frame_fetch_thread, NULL);
    }
    
    TYStopCapture(_M_DEVICE);
    TYClearBufferQueue(_M_DEVICE);
    delete []frameBuffer[0];
    delete []frameBuffer[1];
    frameBuffer[0] = NULL;
    frameBuffer[1] = NULL;

    if(HasStream()) {
      StreamStart();
    }
  }

  SensorType PercipioDepthCam::StreamGetSensorInfo(StreamHandle stream)
  {
    return get_stream_type(stream);
  }

  const TY_DEV_HANDLE PercipioDepthCam::getCurrentDeviceHandle() const
  {
    return _M_DEVICE;
  }

  TY_STATUS PercipioDepthCam::create_leftIR_stream(StreamHandle& stream)
  {
    if(!leftIRStream) leftIRStream = boost::make_shared<NewFrameCallbackManager>();
    stream = static_cast<StreamHandle>(leftIRStream.get());
    return TY_STATUS_OK;
  }

  TY_STATUS PercipioDepthCam::create_rightIR_stream(StreamHandle& stream)
  {
    if(!rightIRStream) rightIRStream = boost::make_shared<NewFrameCallbackManager>();
    stream = static_cast<StreamHandle>(rightIRStream.get());
    return TY_STATUS_OK;
  }
  
  TY_STATUS PercipioDepthCam::create_color_stream(StreamHandle& stream)
  {
    if(!ColorStream) ColorStream = boost::make_shared<NewFrameCallbackManager>();
    stream = static_cast<StreamHandle>(ColorStream.get());
    return TY_STATUS_OK;
  }

  TY_STATUS PercipioDepthCam::create_depth_stream(StreamHandle& stream)
  {
    if(!DepthStream) DepthStream = boost::make_shared<NewFrameCallbackManager>();
    stream = static_cast<StreamHandle>(DepthStream.get());
    return TY_STATUS_OK;
  }

  TY_STATUS PercipioDepthCam::create_point3d_stream(StreamHandle& stream)
  {
    if(!Point3DStream) Point3DStream = boost::make_shared<NewFrameCallbackManager>();
    stream = static_cast<StreamHandle>(Point3DStream.get());
    return TY_STATUS_OK;
  }

  const TY_COMPONENT_ID PercipioDepthCam::get_stream_component_id(StreamHandle stream)
  {
    if(leftIRStream.get() == stream) {
      return TY_COMPONENT_IR_CAM_LEFT;
    }else if(rightIRStream.get() == stream) {
      return TY_COMPONENT_IR_CAM_RIGHT;
    } else if(ColorStream.get() == stream) {
      return TY_COMPONENT_RGB_CAM;
    } else if(DepthStream.get() == stream) {
      return TY_COMPONENT_DEPTH_CAM;
    } else if(Point3DStream.get() == stream) {
      return TY_COMPONENT_DEPTH_CAM;
    }
    else {
      return 0;
    }
  }

  const SensorType PercipioDepthCam::get_stream_type(StreamHandle stream)
  {
    if(leftIRStream.get() == stream) {
      return SENSOR_IR_LEFT;
    }else if(rightIRStream.get() == stream) {
      return SENSOR_IR_RIGHT;
    } else if(ColorStream.get() == stream) {
      return SENSOR_COLOR;
    } else if(DepthStream.get() == stream) {
      return SENSOR_DEPTH;
    } else if(Point3DStream.get() == stream) {
      return SENSOR_POINT3D;
    } else {
      return SENSOR_NONE;
    }
  }

  static boost::shared_ptr<PercipioDepthCam> g_Context = boost::shared_ptr<PercipioDepthCam>();

  TY_STATUS VideoStream::addNewFrameListener(NewFrameListener* pListener)
  {
    if (!isValid()) 
      return TY_STATUS_ERROR;

    pListener->VideoStreamInit(this);
    return g_Context->StreamRegisterNewFrameCallback(m_stream, static_cast<void*>(pListener), pListener->callback);
  }

  void VideoStream::removeNewFrameListener(NewFrameListener* pListener)
  {
    if (!isValid())
    {
      return;
    }
    g_Context->StreamUnregisterNewFrameCallback(m_stream);
    pListener->VideoStreamDeinit();
  }

  TY_STATUS VideoStream::start()
  {
    if (!isValid()) {
      return TY_STATUS_ERROR;
    }

    g_Context->StreamEnable(m_stream);
    return g_Context->StartCapture();
  }

  void VideoStream::stop()
  {
    if (!isValid())
      return;
    g_Context->StreamDisable(m_stream);
    g_Context->StopCapture(m_stream);
  }

  TY_STATUS VideoStream::readFrame(VideoFrameRef* pFrame)
  {
    if (!isValid())
    {
      return TY_STATUS_ERROR;
    }
    
    VideoFrameData& frame = getFrame();
    pFrame->updateFrameData(&frame);
    if(pFrame->getData() != NULL)
      return TY_STATUS_OK;
    else
      return TY_STATUS_ERROR;
  }
  
  VideoMode VideoStream::getVideoMode() const
  {
    VideoMode videoMode = g_Context->get_current_video_mode(m_stream);
    return videoMode;
  }

  float VideoStream::getVerticalFieldOfView() const
  {
    float vertical = 0;
    return vertical;
  }

  TY_STATUS VideoStream::create(const Device& device, SensorType sensorType)
  {
    StreamHandle streamHandle;
    TY_STATUS rc = g_Context->CreateStream(sensorType, streamHandle);
    if (rc != TY_STATUS_OK)
    {
      return rc;
    }
    
    _setHandle(streamHandle);
    return TY_STATUS_OK;
  }

  void VideoStream::destroy()
  {
    if (!isValid())
    {
      return;
    }
    if (m_stream != NULL)
    {
      m_stream = NULL;
    }
  }

  const SensorInfo& VideoStream::getSensorInfo() const
  {
    return m_sensorInfo;
  }

  const StreamHandle& VideoStream::_getHandle() const
  {
    return m_stream;
  }

  void VideoStream::_setHandle(StreamHandle stream)
  {
    //need bind stream handle & sensorInfo
    m_sensorInfo._setInternal(SENSOR_NONE);
    m_stream = stream;
    if (stream != NULL)
    {
      m_sensorInfo._setInternal(g_Context->StreamGetSensorInfo(m_stream));
    }
  }

  void  VideoStream::parseImageData(TY_IMAGE_DATA* img)
  {
    TY_STATUS ret;
    VideoFrameData vframe, tframe;
    SensorType type;
    vframe.setData(img);
    switch(img->componentID)
    {
      case TY_COMPONENT_DEPTH_CAM:
        type = getSensorInfo().getSensorType();
        if(img->pixelFormat == TYPixelFormatCoord3D_C16) {
          g_Context->parseDepthStream(&vframe, &tframe);
          ImageRegistrationMode mode = g_Context->DeviceGetImageRegistrationMode();
          if(mode == IMAGE_REGISTRATION_DEPTH_TO_COLOR) {
            VideoFrameData  tempFrame;
            ret = g_Context->MapDepthFrameToColorCoordinate(&tframe, &tempFrame);//&frame);
            if(ret == TY_STATUS_OK) {
              frame.clone(tempFrame);
            }
          } else {
            frame.clone(tframe);
          }
        } else if(img->pixelFormat == TYPixelFormatCoord3D_ABC16) {
          ImageRegistrationMode mode = g_Context->DeviceGetImageRegistrationMode();
          if(mode == IMAGE_REGISTRATION_DEPTH_TO_COLOR) {
            VideoFrameData  tempFrame;
            ret = g_Context->MapXYZ48FrameToColorCoordinate(&vframe, &tempFrame);
            if(ret == TY_STATUS_OK) {
              frame.clone(tempFrame);
            } else {
              ROS_WARN("XYZ48 MapXYZ48FrameToColorCoordinate ERR : %d", ret);
            }
          } else {
            frame.clone(vframe);
          }
        }
        break;
      case TY_COMPONENT_RGB_CAM:
        if(img->pixelFormat == TYPixelFormatMono8)
          tframe.clone(vframe);
        else
          g_Context->FrameDecoder(vframe, tframe);
        g_Context->parseColorStream(&tframe, &frame);
        break;
      case TY_COMPONENT_IR_CAM_LEFT:
      case TY_COMPONENT_IR_CAM_RIGHT:
        if(img->pixelFormat == TYPixelFormatMono8) {
          frame.clone(vframe);
        } else {
          g_Context->FrameDecoder(vframe, tframe);
          g_Context->parseIrStream(&tframe, &frame);
        }
        break;
      default:
        break;
    }
    return;
  }

  bool VideoStream::isValid() const
  {
    return m_stream != NULL;
  }

  VideoFrameData& VideoStream::getFrame()
  {
    return frame;
  }

  SensorType SensorInfo::getSensorType() const 
  { 
    return m_type;
  }
  
  const Array<VideoMode>& SensorInfo::getSupportedVideoModes() const 
  {
    return m_videoModes;
  }

  SensorInfo::SensorInfo() : /*m_pInfo(NULL), */m_type(SENSOR_NONE), m_videoModes(NULL, 0) 
  {
  }

  SensorInfo::SensorInfo(const SensorType type) : m_videoModes(NULL, 0)
  {
    _setInternal(type);
  }

  void SensorInfo::_setInternal(const SensorType type)
  {
    m_type = type;
    if(m_type == SENSOR_NONE)
    {
      m_videoModes._setData(NULL, 0);
    }
    else
    {
      const std::vector<VideoMode>& video_mode = g_Context->getVideoModeList(type);
      if(video_mode.size()) {
        m_videoModes._setData(&video_mode[0], video_mode.size());
      } else {
        m_videoModes._setData(NULL, 0);
      }
    }
  }

  VideoFrameData::VideoFrameData()
  {
    pthread_mutex_lock(&_mutex);
    m_isOwner = true;

    timestamp = 0;
    imageIndex = 0;
    status = 0;
    componentID = 0;
    size = 0;
    
    width = 0;
    height = 0;
    pixelFormat = 0;

    buffer = NULL;
    pthread_mutex_unlock(&_mutex);
  }
  
  VideoFrameData::~VideoFrameData()
  {
    pthread_mutex_lock(&_mutex);
    if(m_isOwner) {
      if(buffer) {
        free(buffer);
        buffer = NULL;
      }
    }
    pthread_mutex_unlock(&_mutex);
  }

  void  VideoFrameData::setTimestamp(uint64_t time) 
  {
    timestamp = time;
  }
  
  void  VideoFrameData::setWidth(int32_t w)
  {
    width = w;
  }

  void  VideoFrameData::setHeight(int32_t h)
  {
    height = h;
  }

  void  VideoFrameData::Resize(int32_t sz)
  {
    if(size == sz) return;
    
    size = sz;

    if(size != 0) {
      if(m_isOwner) {
        if(buffer != NULL)
          free(buffer);
      }

      buffer = malloc(size);
      memset(buffer, 0, size);
      m_isOwner = true;
    }
  }

  void VideoFrameData::setData(TY_IMAGE_DATA* data)
  {
    pthread_mutex_lock(&_mutex);
    timestamp = data->timestamp;
    imageIndex = data->imageIndex;
    status = data->status;
    componentID = data->componentID;
    size = data->size;
    
    buffer = data->buffer;
    width = data->width;
    height = data->height;
    pixelFormat = data->pixelFormat;

    m_isOwner = false;
    pthread_mutex_unlock(&_mutex);
  }

  void  VideoFrameData::clone(const VideoFrameData& frame)
  {
    pthread_mutex_lock(&_mutex);
    timestamp = frame.timestamp;
    imageIndex = frame.imageIndex;
    status = frame.status;
    componentID = frame.componentID;
    size = frame.size;
    
    width = frame.width;
    height = frame.height;
    pixelFormat = frame.pixelFormat;

    if(m_isOwner && buffer)
      free(buffer);
    
    buffer = malloc(size);
    memcpy(buffer, frame.getData(), size);

    m_isOwner = true;
    pthread_mutex_unlock(&_mutex);
  }

  void  VideoFrameData::setPixelFormat(uint32_t fmt) 
  {
    pixelFormat = fmt;
  }

  void  VideoFrameData::setFrameIndex(int32_t idx) 
  {
    imageIndex = idx;
  }

  void  VideoFrameData::setComponentID(int32_t compID)
  {
    componentID = compID;
  }

  TY_STATUS Device::open(const char* uri, const bool auto_reconnect)
  {
    TY_STATUS rc;
    if(!m_isOwner)
    {
      if(isValid()){
        return TY_STATUS_OK;
      }else{
        return TY_STATUS_ERROR;
      }
    }

    g_Context->b_auto_reconnect = auto_reconnect;

    bool isIP = IPv4_verify(uri);
    if(isIP)
      rc = g_Context->openWithIP(uri);
    else
      rc = g_Context->openWithSN(uri);
    if(rc != TY_STATUS_OK)
      return rc;

    DeviceGetInfo();

    return TY_STATUS_OK;
  }

  void Device::close()
  {
    if(!m_isOwner)
    {
      g_Context->close();
    }
  }

  boost::shared_ptr<PercipioDepthCam> Device::DevicePtr()
  {
    return g_Context;
  }

  bool Device::isValid() const 
  { 
    return g_Context->isValid(); 
  }

  void Device::setColorUndistortion(bool enabled)
  {
    g_Context->DeviceSetColorUndistortionEnable(enabled);
  }

  bool Device::setDepthSpecFilterEn(bool enabled)
  {
    return g_Context->DepthStreamSetSpeckFilterEn(enabled);
  }

  bool Device::getDepthSpecFilterEn()
  {
    return g_Context->DepthStreamGetSpeckFilterEn();
  }

  bool Device::setDepthSpecFilterSpecSize(int spec_size)
  {
    return g_Context->DepthStreamSetSpeckFilterSpecSize(spec_size);
  }

  int Device::getDepthSpecFilterSpecSize()
  {
    return g_Context->DepthStreamGetSpeckFilterSpecSize();
  }

  bool Device::setDepthSpeckFilterDiff(int spec_diff)
  {
    return g_Context->DepthStreamSetSpeckFilterDiff(spec_diff);
  }

  int Device::getDepthSpeckFilterDiff()
  {
    return g_Context->DepthStreamGetSpeckFilterDiff();
  }

  bool Device::setDepthTimeDomainFilterEn(bool enabled)
  {
    return g_Context->DepthStreamSetTimeDomainFilterEn(enabled);
  }

  bool Device::getDepthTimeDomainFilterEn()
  {
    return g_Context->DepthStreamGetTimeDomainFilterEn();
  }

  bool Device::setDepthTimeDomainFilterNum(int frames)
  {
    return g_Context->DepthStreamSetTimeDomainFilterFCnt(frames);
  }

  int  Device::getDepthTimeDomainFilterNum()
  {
    return g_Context->DepthStreamGetTimeDomainFilterFCnt();
  }

  bool Device::isImageRegistrationModeSupported(ImageRegistrationMode mode) const
  {
    return (g_Context->DeviceIsImageRegistrationModeSupported() == true);
  }

  TY_STATUS Device::setImageRegistrationMode(ImageRegistrationMode mode)
  {
    return g_Context->DeviceSetImageRegistrationMode(mode);
  }

  ImageRegistrationMode Device::getImageRegistrationMode() const
  {
    return g_Context->DeviceGetImageRegistrationMode();
  }

  TY_STATUS Device::PropertyGet(TY_COMPONENT_ID comp, TY_FEATURE_ID feat, void* ptr, size_t size)
  {
    return g_Context->DeviceGetProperty(comp, feat, ptr, size);
  }

  TY_STATUS Device::PropertySet(TY_COMPONENT_ID comp, TY_FEATURE_ID feat, void* ptr, size_t size)
  {
    return g_Context->DeviceSetProperty(comp, feat, ptr, size);
  }

  bool Device::hasSensor(SensorType sensorType)
  {
    bool has = false;
    switch (sensorType)
    {
    case SENSOR_IR_LEFT:
    {
      if(g_Context->get_components() & TY_COMPONENT_IR_CAM_LEFT)
        has = true;
      break;
    }
    case SENSOR_IR_RIGHT:
    {
      if(g_Context->get_components() & TY_COMPONENT_IR_CAM_RIGHT)
        has = true;
      break;
    }
	  case SENSOR_COLOR:
    {
      if(g_Context->get_components() & TY_COMPONENT_RGB_CAM)
        has = true;
      break;
    }
	  case SENSOR_DEPTH:
    case SENSOR_POINT3D:
    {
      if(g_Context->get_components() & TY_COMPONENT_DEPTH_CAM)
        has = true;
      break;
    }
    default:
      break;
    }
    return has;
  }

  bool Device::ResolutionSetting(SensorType sensorType, int width, int height, const std::string& fmt)
  {
    switch (sensorType)
    {
      case SENSOR_COLOR:
        return g_Context->set_image_mode(SENSOR_COLOR, width, height, fmt);
	    case SENSOR_DEPTH:
        return g_Context->set_image_mode(SENSOR_DEPTH, width, height, fmt);
      default:
        return false;
    }
  }

  TY_STATUS Device::_setHandle(TY_DEV_HANDLE deviceHandle)
  {
      return DeviceGetInfo();
  }

  TY_STATUS Device::DeviceGetInfo()
  {
    TY_STATUS rc;
    TY_DEVICE_BASE_INFO  info;
    rc = g_Context->get_device_info(info);
    if(rc != TY_STATUS_OK)
      return rc;

    if (TYIsNetworkInterface(info.iface.type)) {
      m_deviceInfo = DeviceInfo(info.id, info.vendorName, info.modelName, info.netInfo.ip);
    } else {
      m_deviceInfo = DeviceInfo(info.id, info.vendorName, info.modelName, PERCIPIO_USB_PID, PERCIPIO_USB_VID);
    }
#if 0    
    if(info.iface.type == TY_INTERFACE_USB) {
      m_deviceInfo.setUsbVendorId(PERCIPIO_USB_VID);
      m_deviceInfo.setUsbProductId(PERCIPIO_USB_PID);
    } else {
      m_deviceInfo.setUsbVendorId(0);
      m_deviceInfo.setUsbProductId(0);
    }
#endif
  }

  bool Device::IPv4_verify(const char *ip) 
  {
    int a,b,c,d;
    char t;

	  if (4 == sscanf(ip,"%d.%d.%d.%d%c",&a,&b,&c,&d,&t)) {
      if (0<=a && a<=255 && 
          0<=b && b<=255 && 
          0<=c && c<=255 && 
          0<=d && d<=255) {
        return true;
      }
    }
    return false;
  }

  ////
  TY_STATUS Percipio::addDeviceConnectedListener(DeviceConnectedListener* pListener)
  {
    return g_Context->RegisterDeviceCallbacks(&pListener->m_deviceConnectedCallbacks, pListener);
  }

  void Percipio::removeDeviceConnectedListener(DeviceConnectedListener* pListener)
  {
    g_Context->UnregisterDeviceCallbacks(pListener);
  }

  TY_STATUS Percipio::addDeviceDisconnectedListener(DeviceDisconnectedListener* pListener)
  {
    return g_Context->RegisterDeviceCallbacks(&pListener->m_deviceDisconnectedCallbacks, pListener);
  } 

  void Percipio::removeDeviceDisconnectedListener(DeviceDisconnectedListener* pListener)
  {
    g_Context->UnregisterDeviceCallbacks(pListener);
  } 

  TY_STATUS Percipio::initialize()
  {
    if(g_Context) g_Context.reset();

    g_Context = boost::make_shared<PercipioDepthCam>();
    return g_Context->initialize();
  } 

  void Percipio::enumerateDevices(Array<DeviceInfo>* deviceInfoList)
  {
    DeviceInfo* m_pDeviceInfos;
    int m_deviceInfoCount;
    g_Context->GetDeviceList(&m_pDeviceInfos, &m_deviceInfoCount);
    deviceInfoList->_setData((DeviceInfo*)m_pDeviceInfos, m_deviceInfoCount, true);
    delete []m_pDeviceInfos;
  } 

  const char* Percipio::getExtendedError(TY_STATUS status) {
    return TYErrorString(status);
  }
}