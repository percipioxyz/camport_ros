/*
 * @Description: 
 * @Author: zxy
 * @Date: 2023-08-09 09:11:59
 * @LastEditors: zxy
 * @LastEditTime: 2023-08-15 11:54:36
 */
#include "percipio_camera/percipio_interface.h"
#include "percipio_camera/image_process.hpp"

namespace percipio
{
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
  
  bool NewFrameCallbackManager::isInvalid()
  {
    if(frame_listener && cb && is_enable) return true;
    return false;
  }

  void NewFrameCallbackManager::enableCallback(bool en)
  {
    is_enable = en;
  }

  //// 
  TY_STATUS percipio_depth_cam::initialize()
  {
    TY_STATUS rc;
    rc = TYInitLib();
    if(rc != TY_STATUS_OK)
      return rc;

    std::vector<TY_DEVICE_BASE_INFO> selected;
    selectDevice(TY_INTERFACE_ALL, "", "", 100, selected);
    for(size_t i = 0; i < selected.size(); i++) {
      device_list.push_back(DeviceInfo(selected[i].id, selected[i].vendorName, selected[i].modelName));
    }
    return TY_STATUS_OK;
  }

  void percipio_depth_cam::GetDeviceList(DeviceInfo** device_info_ptr, int* cnt)
  {
    *cnt = device_list.size();
    *device_info_ptr = new DeviceInfo[device_list.size()];
    for(size_t i = 0; i < device_list.size(); i++)
      *(*device_info_ptr + i) = device_list[i];
  }

  TY_STATUS percipio_depth_cam::open(const char* sn)
  {
    TY_DEV_HANDLE deviceHandle;
    std::vector<TY_DEVICE_BASE_INFO> selected;
    TY_STATUS rc = selectDevice(TY_INTERFACE_ALL, sn, "", 1, selected);
    if(!selected.size())
      return TY_STATUS_ERROR;
    TY_DEVICE_BASE_INFO& selectedDev = selected[0];
    rc = TYOpenInterface(selectedDev.iface.id, &_M_IFACE);
    if(rc != TY_STATUS_OK)
      return rc;
    rc = TYOpenDevice(_M_IFACE, selectedDev.id, &deviceHandle);
    if(rc != TY_STATUS_OK) {
      TYCloseInterface(_M_IFACE);
      return rc;
    }
    _M_DEVICE = deviceHandle;
    TYGetComponentIDs(_M_DEVICE, &m_ids);
    std::vector<TY_ENUM_ENTRY> feature_info;
    if(m_ids & TY_COMPONENT_IR_CAM_LEFT) {
      feature_info.clear();
      get_feature_enum_list(_M_DEVICE,TY_COMPONENT_IR_CAM_LEFT,TY_ENUM_IMAGE_MODE, feature_info);
      generate_video_mode(feature_info, leftIRVideoMode);

      TYGetStruct(_M_DEVICE, TY_COMPONENT_RGB_CAM, TY_STRUCT_CAM_CALIB_DATA, &color_calib, sizeof(color_calib));
    }
    if(m_ids & TY_COMPONENT_IR_CAM_RIGHT) {
      feature_info.clear();
      get_feature_enum_list(_M_DEVICE,TY_COMPONENT_IR_CAM_RIGHT,TY_ENUM_IMAGE_MODE, feature_info);
      generate_video_mode(feature_info, rightIRVideoMode);
    }
    if(m_ids & TY_COMPONENT_RGB_CAM) {
      feature_info.clear();
      get_feature_enum_list(_M_DEVICE,TY_COMPONENT_RGB_CAM,TY_ENUM_IMAGE_MODE, feature_info);
      generate_video_mode(feature_info, RGBVideoMode);
    }
    if(m_ids & TY_COMPONENT_DEPTH_CAM) {
      feature_info.clear();
      get_feature_enum_list(_M_DEVICE,TY_COMPONENT_DEPTH_CAM,TY_ENUM_IMAGE_MODE, feature_info);
      generate_video_mode(feature_info, DepthVideoMode);

      TYGetStruct(_M_DEVICE, TY_COMPONENT_DEPTH_CAM, TY_STRUCT_CAM_CALIB_DATA, &depth_calib, sizeof(depth_calib));

      TYGetFloat(_M_DEVICE, TY_COMPONENT_DEPTH_CAM, TY_FLOAT_SCALE_UNIT, &f_depth_scale_unit);
    }

    current_device_info = DeviceInfo(selectedDev.id, selectedDev.vendorName, selectedDev.modelName);
    TYRegisterEventCallback(_M_DEVICE, eventCallback, &current_device_info);
    return TY_STATUS_OK;
  }

  const DeviceInfo& percipio_depth_cam::get_current_device_info()
  {
    return current_device_info;
  }

  bool percipio_depth_cam::set_image_mode(TY_COMPONENT_ID  comp, int width, int height)
  {
    std::vector<TY_ENUM_ENTRY> image_mode_list;
    TY_STATUS rc = get_feature_enum_list(_M_DEVICE, comp, TY_ENUM_IMAGE_MODE, image_mode_list);
    if(rc != TY_STATUS_OK)
      return false;
    
    for(size_t i = 0; i < image_mode_list.size(); i++) {
      TY_IMAGE_MODE mode = image_mode_list[i].value;
      int w = TYImageWidth(mode);
      int h = TYImageHeight(mode);
      if((w == width) && (h == height)) {
        if(TYSetEnum(_M_DEVICE, comp, TY_ENUM_IMAGE_MODE, mode) == TY_STATUS_OK)
          return true;
      }

      if((w == width) || (h == height)) {
        if(TYSetEnum(_M_DEVICE, comp, TY_ENUM_IMAGE_MODE, mode) == TY_STATUS_OK) {
          ROS_WARN("Resolution mismatch %dx%x  != %dx%d\n", width, height, w, h);
          return true;
        }
      }
    }

    ROS_WARN("Unsuitable resolution %dx%x\n", width, height);
    return false;
  }

  void percipio_depth_cam::eventCallback(TY_EVENT_INFO *event_info, void *userdata)
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
        
        cb_list[i].cb->deviceDisconnected(static_cast<DeviceInfo*>(userdata), cb_list[i].pListener);
      }
    }
  }

  TY_STATUS percipio_depth_cam::RegisterDeviceCallbacks(DeviceCallbacks* callback, void* listener)
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

  void percipio_depth_cam::UnregisterDeviceCallbacks(void* listener)
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

  bool percipio_depth_cam::isValid() 
  {
    if (_M_DEVICE != NULL)
      return true;
    else
      return false;
  }

  TY_STATUS percipio_depth_cam::get_device_info(TY_DEVICE_BASE_INFO& info)
  {
    return TYGetDeviceInfo(_M_DEVICE, &info);
  }

  void percipio_depth_cam::close()
  {
    if (_M_DEVICE != NULL)
    {
      TYCloseDevice(_M_DEVICE);
      TYCloseInterface(_M_IFACE);
    }
    _M_DEVICE = NULL;
  }

  const uint32_t&  percipio_depth_cam::get_components() const 
  {
    return m_ids;
  }

  bool percipio_depth_cam::DeviceSetColorUndistortionEnable(bool enable)
  {
    color_undistortion = enable;
  }

  bool percipio_depth_cam::DeviceIsImageRegistrationModeSupported() const
  {
    if((m_ids & TY_COMPONENT_DEPTH_CAM) &&(m_ids &  TY_COMPONENT_RGB_CAM))
      return true;
    else
      return false;
  }
  
  TY_STATUS percipio_depth_cam::DeviceSetImageRegistrationMode(ImageRegistrationMode mode)
  {
    registration_mode = mode;
    return TY_STATUS_OK;
  }

  ImageRegistrationMode percipio_depth_cam::DeviceGetImageRegistrationMode()
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

  TY_STATUS percipio_depth_cam::MapDepthFrameToColorCoordinate(VideoFrameData* src, VideoFrameData* dst)
  {
    return ImgProc::MapDepthImageToColorCoordinate(&depth_calib, &color_calib, current_rgb_width, current_rgb_height, src, dst, f_depth_scale_unit);
  }

  TY_STATUS percipio_depth_cam::parseColorStream(VideoFrameData* src, VideoFrameData* dst)
  {
    if(color_undistortion) {
      return ImgProc::doRGBUndistortion(&color_calib, src, dst);
    }else {
      dst->clone(*src);
      return TY_STATUS_OK;
    }
  }

  TY_STATUS percipio_depth_cam::parseDepthStream(VideoFrameData* src, VideoFrameData* dst)
  {
    if(depth_distortion)
      return ImgProc::doDepthUndistortion(&depth_calib, src, dst);
    else {
      dst->clone(*src);
      return TY_STATUS_OK;
    }
  }

  TY_STATUS percipio_depth_cam::FrameDecoder(VideoFrameData& src, VideoFrameData& dst)
  {
    return ImgProc::cvtColor(src, dst);
  }

  bool percipio_depth_cam::isDepthColorSyncSupport()
  {
    bool has = false;
    TYHasFeature(_M_DEVICE, TY_COMPONENT_DEVICE, TY_BOOL_CMOS_SYNC, &has);
    return has;
  }

  TY_STATUS percipio_depth_cam::DeviceEnableDepthColorSync()
  {
    return TYSetBool(_M_DEVICE, TY_COMPONENT_DEVICE, TY_BOOL_CMOS_SYNC, true);
  }

  TY_STATUS percipio_depth_cam::DeviceDisableDepthColorSync()
  {
    return TYSetBool(_M_DEVICE, TY_COMPONENT_DEVICE, TY_BOOL_CMOS_SYNC, false);
  }

  bool percipio_depth_cam::DeviceGetDepthColorSyncEnabled()
  {
    bool sync = false;
    TYGetBool(_M_DEVICE, TY_COMPONENT_DEVICE, TY_BOOL_CMOS_SYNC, &sync);
    return sync;
  }

  template <class T>
  TY_STATUS percipio_depth_cam::getProperty(StreamHandle stream, uint32_t propertyId, void* value)
  {
    TY_COMPONENT_ID comp = get_stream_component_id(stream);
    if(0 == comp)
      return TY_STATUS_INVALID_PARAMETER;
    TY_FEATURE_TYPE type = TYFeatureType(propertyId);
    switch(type) {
      case TY_FEATURE_INT:
        return TYGetInt(_M_DEVICE, comp, propertyId, static_cast<int*>(value));
      case TY_FEATURE_FLOAT:
        return TYGetFloat(_M_DEVICE, comp, propertyId, static_cast<float*>(value));
      case TY_FEATURE_ENUM:
        return TYGetEnum(_M_DEVICE, comp, propertyId, static_cast<uint32_t*>(value));
      case TY_FEATURE_BOOL:
        return TYGetBool(_M_DEVICE, comp, propertyId, static_cast<bool*>(value));
    }
    return TY_STATUS_INVALID_PARAMETER;
  }

  template <class T>
  TY_STATUS percipio_depth_cam::setProperty(StreamHandle stream, uint32_t propertyId, const T& value)
  {
    TY_COMPONENT_ID comp = get_stream_component_id(stream);
    if(0 == comp)
      return TY_STATUS_INVALID_PARAMETER;
    
    TY_FEATURE_TYPE type = TYFeatureType(propertyId);
    switch(type) {
      case TY_FEATURE_INT:
        return TYSetInt(_M_DEVICE, comp, propertyId, value);
      case TY_FEATURE_FLOAT:
        return TYSetFloat(_M_DEVICE, comp, propertyId, value);
      case TY_FEATURE_ENUM:
        return TYSetEnum(_M_DEVICE, comp, propertyId, value);
      case TY_FEATURE_BOOL:
        return TYSetBool(_M_DEVICE, comp, propertyId, value);
    }
    return TY_STATUS_INVALID_PARAMETER;
  }

  TY_STATUS percipio_depth_cam::DeviceGetProperty(int propertyId, void* data, int* dataSize)
  {
    TY_STATUS rc;
    TY_DEVICE_BASE_INFO inf;
    switch(propertyId)
    {
      case TY_DEVICE_PROPERTY_DEPTH_CALIB_INTRISTIC:
        if((*dataSize) < (9 * sizeof(float)))
          return TY_STATUS_NO_BUFFER;
        memcpy(data, depth_intr.data, sizeof(depth_intr.data));
        break;
      case TY_DEVICE_PROPERTY_COLOR_CALIB_INTRISTIC:
        if((*dataSize) < (9 * sizeof(float)))
          return TY_STATUS_NO_BUFFER;
        memcpy(data, color_intr.data, sizeof(color_intr.data));
        break;
      case TY_DEVICE_PROPERTY_SERIAL_NUMBER:
        rc = TYGetDeviceInfo(_M_DEVICE, &inf);
        if(rc != TY_STATUS_OK)
          return rc;
        if((*dataSize) < (strlen(inf.id) + 1))
          return TY_STATUS_NO_BUFFER;
        strcpy(static_cast<char*>(data), inf.id);
        break;
      default:
        return TY_STATUS_NOT_PERMITTED;
    }
    return TY_STATUS_OK;
  }

  TY_STATUS percipio_depth_cam::DeviceSetProperty(int propertyId, const void* data, int* dataSize)
  {
    //TODO
    return TY_STATUS_NOT_PERMITTED;
  }

  const std::vector<VideoMode>& percipio_depth_cam::getVideoModeList(SensorType type) const 
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

  TY_STATUS percipio_depth_cam::CreateStream(SensorType sensorType, StreamHandle& streamHandle)
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
      default:
        return TY_STATUS_INVALID_PARAMETER;
    }
  }

  VideoMode percipio_depth_cam::get_current_video_mode(StreamHandle stream)
  {
    uint32_t value = 0;
    pthread_mutex_lock(&m_mutex);
    TY_COMPONENT_ID comp = get_stream_component_id(stream);
    if(0 == comp)
      return VideoMode();
    
    TYGetEnum(_M_DEVICE, comp, TY_ENUM_IMAGE_MODE, &value);
    pthread_mutex_unlock(&m_mutex);
    return VideoMode(value);
  }

  TY_STATUS percipio_depth_cam::set_current_video_mode(StreamHandle stream, const VideoMode& videoMode)
  {
    TY_COMPONENT_ID comp = get_stream_component_id(stream);
    if(0 == comp)
      return TY_STATUS_INVALID_PARAMETER;
    
    uint32_t image_mode = get_stream_image_mode(comp, videoMode);
    if(image_mode == 0) {
      return TY_STATUS_INVALID_PARAMETER;
    }
    return TYSetEnum(_M_DEVICE, comp, TY_ENUM_IMAGE_MODE, image_mode);
  }

  TY_STATUS percipio_depth_cam::StreamRegisterNewFrameCallback(StreamHandle stream, void* listener, NewFrameCallback cb)//, void* listener)
  {
    pthread_mutex_lock(&m_mutex);
    if(leftIRStream.get() == stream) {
      leftIRStream.get()->register_callback(listener, cb);
    } else if(rightIRStream.get() == stream) {
      rightIRStream.get()->register_callback(listener, cb);
    } else if(ColorStream.get() == stream) {
      ColorStream.get()->register_callback(listener, cb);
    } else if(DepthStream.get() == stream) {
      DepthStream.get()->register_callback(listener, cb);
    } else {
      pthread_mutex_unlock(&m_mutex);
      return TY_STATUS_INVALID_PARAMETER;
    }
    pthread_mutex_unlock(&m_mutex);
    return TY_STATUS_OK;
  }

  void percipio_depth_cam::StreamUnregisterNewFrameCallback(StreamHandle stream)
  {
    pthread_mutex_lock(&m_mutex);
    if(leftIRStream.get() == stream) {
      leftIRStream.get()->unregister_callback();
    } else if(rightIRStream.get() == stream) {
      rightIRStream.get()->unregister_callback();
    } else if(ColorStream.get() == stream) {
      ColorStream.get()->unregister_callback();
    } else if(DepthStream.get() == stream) {
      DepthStream.get()->unregister_callback();
    }
    pthread_mutex_unlock(&m_mutex);
  }

  void* percipio_depth_cam::fetch_thread(void* ptr)
  {
    TY_STATUS rc;
    percipio_depth_cam* cam = (percipio_depth_cam*)ptr;
    const TY_DEV_HANDLE handle = cam->getCurrentDeviceHandle();
    while(cam->isRuning) {
      TY_FRAME_DATA frame;
      rc = TYFetchFrame(handle, &frame, 2000);
      if(rc == TY_STATUS_OK) {
        for (int i = 0; i < frame.validCount; i++){
          if (frame.image[i].status != TY_STATUS_OK) continue;
          if (frame.image[i].componentID == TY_COMPONENT_DEPTH_CAM){
            if(cam->DepthStream.get()->cb) {
              cam->DepthStream.get()->cb(cam->DepthStream.get(), cam->DepthStream.get()->frame_listener, &frame.image[i]);
            }
          }
          else if(frame.image[i].componentID == TY_COMPONENT_RGB_CAM){ 
            if(cam->ColorStream.get()->cb) {
              cam->ColorStream.get()->cb(cam->ColorStream.get(), cam->ColorStream.get()->frame_listener, &frame.image[i]);
            }
          }
          else if(frame.image[i].componentID == TY_COMPONENT_IR_CAM_LEFT){ 
            if(cam->leftIRStream.get()->cb) {
              cam->leftIRStream.get()->cb(cam->leftIRStream.get(), cam->leftIRStream.get()->frame_listener, &frame.image[i]);
            }
          }
          else if(frame.image[i].componentID == TY_COMPONENT_IR_CAM_RIGHT){ 
            if(cam->rightIRStream.get()->cb) {
              cam->leftIRStream.get()->cb(cam->rightIRStream.get(), cam->leftIRStream.get()->frame_listener, &frame.image[i]);
            }
          }
        }

        TYEnqueueBuffer(handle, frame.userBuffer, frame.bufferSize);
      }
    }
    
    return NULL;
  }

  bool percipio_depth_cam::HasStream()
  {
    if(leftIRStream.get() && leftIRStream.get()->isInvalid())
      return true;

    if(rightIRStream.get() && rightIRStream.get()->isInvalid())
      return true;

    if(ColorStream.get() && ColorStream.get()->isInvalid())
      return true;

    if(DepthStream.get() && DepthStream.get()->isInvalid())
      return true;
    
    return false;
  }

  void percipio_depth_cam::enable_stream(StreamHandle stream)
  {
    if(leftIRStream.get() == stream) {
      leftIRStream.get()->enableCallback(true);
    } else if(rightIRStream.get() == stream) {
      rightIRStream.get()->enableCallback(true);
    } else if(ColorStream.get() == stream) {
      ColorStream.get()->enableCallback(true);
    } else if(DepthStream.get() == stream) {
      DepthStream.get()->enableCallback(true);
    } else {
      printf("unknown stream : %p %p %p %p %p\n", stream, leftIRStream.get(), rightIRStream.get(), ColorStream.get(), DepthStream.get());
    }
  }

  void percipio_depth_cam::disable_stream(StreamHandle stream)
  {
    if(leftIRStream.get() == stream) {
      leftIRStream.get()->enableCallback(false);
    } else if(rightIRStream.get() == stream) {
      rightIRStream.get()->enableCallback(false);
    } else if(ColorStream.get() == stream) {
      ColorStream.get()->enableCallback(false);
    } else if(DepthStream.get() == stream) {
      DepthStream.get()->enableCallback(false);
    } 
  }

  
  TY_STATUS percipio_depth_cam::StartCapture()
  {
    pthread_mutex_lock(&m_mutex);
    TY_STATUS rc = StreamStart();
    pthread_mutex_unlock(&m_mutex);
    return rc;
  }

  void percipio_depth_cam::StopCapture(StreamHandle stream)
  {
    pthread_mutex_lock(&m_mutex);
    StreamStop(stream);
    pthread_mutex_unlock(&m_mutex);
  }

  TY_STATUS percipio_depth_cam::StreamStart()
  {
    TY_STATUS rc;
    if(isRuning) {
      StreamStopAll();
    }
    
    b_stream_with_color = false;

    depth_distortion = false;
    
    if(!HasStream()) {
      return TY_STATUS_ERROR;
    }

    //VideoStream

    if(leftIRStream.get() && leftIRStream.get()->isInvalid()) {
      TYEnableComponents(_M_DEVICE, TY_COMPONENT_IR_CAM_LEFT);
    }else
      TYDisableComponents(_M_DEVICE, TY_COMPONENT_IR_CAM_LEFT);
    
    if(rightIRStream.get() && rightIRStream.get()->isInvalid()){
      TYEnableComponents(_M_DEVICE, TY_COMPONENT_IR_CAM_RIGHT);
    }else
      TYDisableComponents(_M_DEVICE, TY_COMPONENT_IR_CAM_RIGHT);

    if(ColorStream.get() && ColorStream.get()->isInvalid()) {
      TYEnableComponents(_M_DEVICE, TY_COMPONENT_RGB_CAM);
      TYGetInt(_M_DEVICE, TY_COMPONENT_RGB_CAM, TY_INT_WIDTH, &current_rgb_width);
      TYGetInt(_M_DEVICE, TY_COMPONENT_RGB_CAM, TY_INT_HEIGHT, &current_rgb_height);
      b_stream_with_color = true;
    }else {
      TYDisableComponents(_M_DEVICE, TY_COMPONENT_RGB_CAM);
      current_rgb_width = 0;
      current_rgb_height = 0;
      b_stream_with_color = false;
    }

    if(DepthStream.get() && DepthStream.get()->isInvalid()) {
      TYEnableComponents(_M_DEVICE, TY_COMPONENT_DEPTH_CAM);
      TYGetInt(_M_DEVICE, TY_COMPONENT_DEPTH_CAM, TY_INT_WIDTH, &current_depth_width);
      TYGetInt(_M_DEVICE, TY_COMPONENT_DEPTH_CAM, TY_INT_HEIGHT, &current_depth_height);
      TYHasFeature(_M_DEVICE, TY_COMPONENT_DEPTH_CAM, TY_STRUCT_CAM_INTRINSIC, &depth_distortion);
    }else {
      TYDisableComponents(_M_DEVICE, TY_COMPONENT_DEPTH_CAM);
      current_depth_width = 0;
      current_depth_height = 0;
    }

    TYGetStruct(_M_DEVICE, TY_COMPONENT_DEPTH_CAM, TY_STRUCT_CAM_INTRINSIC, &depth_intr, sizeof(depth_intr));
    TYGetStruct(_M_DEVICE, TY_COMPONENT_RGB_CAM, TY_STRUCT_CAM_INTRINSIC, &color_intr, sizeof(color_intr));
    
    uint32_t frameSize;
    TYGetFrameBufferSize(_M_DEVICE, &frameSize);
    if(frameBuffer[0]) delete []frameBuffer[0];
    if(frameBuffer[1]) delete []frameBuffer[1];
    frameBuffer[0] = new char[frameSize];
    frameBuffer[1] = new char[frameSize];

    TYEnqueueBuffer(_M_DEVICE, frameBuffer[0], frameSize);
    TYEnqueueBuffer(_M_DEVICE, frameBuffer[1], frameSize);

    rc = TYStartCapture(_M_DEVICE);
    if(rc != TY_STATUS_OK)
      return rc;
    
    isRuning = true;
    pthread_create(&frame_fetch_thread, NULL, fetch_thread, this);
    return TY_STATUS_OK;
  }

  void percipio_depth_cam::StreamStopAll()
  {
    isRuning = false;
    pthread_join(frame_fetch_thread, NULL);
    TYStopCapture(_M_DEVICE);
    TYClearBufferQueue(_M_DEVICE);
    delete []frameBuffer[0];
    delete []frameBuffer[1];
    frameBuffer[0] = NULL;
    frameBuffer[1] = NULL;
  }

  void percipio_depth_cam::StreamStop(StreamHandle stream)
  {
    TY_COMPONENT_ID  comp = get_stream_component_id(stream);
    if(comp == 0)
      return;

    isRuning = false;
    pthread_join(frame_fetch_thread, NULL);
    TYStopCapture(_M_DEVICE);
    TYClearBufferQueue(_M_DEVICE);
    delete []frameBuffer[0];
    delete []frameBuffer[1];
    frameBuffer[0] = NULL;
    frameBuffer[1] = NULL;
#if 0
    if(leftIRStream.get() == stream)
      leftIRStream.reset();
    else if(rightIRStream.get() == stream)
      rightIRStream.reset();
    else if(ColorStream.get() == stream)
      ColorStream.reset();
    else if(DepthStream.get() == stream)
      DepthStream.reset();
#endif
    if(HasStream()) {
      StreamStart();
    }
  }

  SensorType percipio_depth_cam::StreamGetSensorInfo(StreamHandle stream)
  {
    return get_stream_type(stream);
  }

  const TY_DEV_HANDLE percipio_depth_cam::getCurrentDeviceHandle() const
  {
    return _M_DEVICE;
  }

  void percipio_depth_cam::generate_video_mode(const std::vector<TY_ENUM_ENTRY>& feature_info, std::vector<VideoMode>& videomode)
  {
    for(size_t i = 0; i < feature_info.size(); i++) {
      uint32_t IMAGE_MODE = feature_info[i].value;
      videomode.push_back(VideoMode(IMAGE_MODE));
    }
  }

  TY_STATUS percipio_depth_cam::create_leftIR_stream(StreamHandle& stream)
  {
    if(leftIRStream.get() == NULL) {
      leftIRStream = boost::make_shared<NewFrameCallbackManager>();
      stream = leftIRStream.get();
      return TY_STATUS_OK;
    }
  }

  TY_STATUS percipio_depth_cam::create_rightIR_stream(StreamHandle& stream)
  {
    if(rightIRStream.get() == NULL) {
      rightIRStream = boost::make_shared<NewFrameCallbackManager>();
      stream = rightIRStream.get();
      return TY_STATUS_OK;
    }
  }
  
  TY_STATUS percipio_depth_cam::create_color_stream(StreamHandle& stream)
  {
    if(ColorStream.get() == NULL) {
      ColorStream = boost::make_shared<NewFrameCallbackManager>();
      stream = ColorStream.get();
      return TY_STATUS_OK;
    }
  }

  TY_STATUS percipio_depth_cam::create_depth_stream(StreamHandle& stream)
  {
    if(DepthStream.get() == NULL) {
      DepthStream = boost::make_shared<NewFrameCallbackManager>();
      stream = DepthStream.get();
      return TY_STATUS_OK;
    }
  }
  const TY_COMPONENT_ID percipio_depth_cam::get_stream_component_id(StreamHandle stream)
  {
    if(leftIRStream.get() == stream) {
      return TY_COMPONENT_IR_CAM_LEFT;
    }else if(rightIRStream.get() == stream) {
      return TY_COMPONENT_IR_CAM_RIGHT;
    } else if(ColorStream.get() == stream) {
      return TY_COMPONENT_RGB_CAM;
    } else if(DepthStream.get() == stream) {
      return TY_COMPONENT_DEPTH_CAM;
    } else {
      return 0;
    }
  }

  const SensorType percipio_depth_cam::get_stream_type(StreamHandle stream)
  {
    if(leftIRStream.get() == stream) {
      return SENSOR_IR_LEFT;
    }else if(rightIRStream.get() == stream) {
      return SENSOR_IR_RIGHT;
    } else if(ColorStream.get() == stream) {
      return SENSOR_COLOR;
    } else if(DepthStream.get() == stream) {
      return SENSOR_DEPTH;
    } else {
      return SENSOR_NONE;
    }
  }
  uint32_t percipio_depth_cam::get_stream_image_mode(TY_COMPONENT_ID comp, const VideoMode& videoMode)
  {
    switch(comp) 
    {
      case TY_COMPONENT_IR_CAM_LEFT:
        return get_leftir_stream_image_mode(videoMode);
      case TY_COMPONENT_IR_CAM_RIGHT:
        return get_rightir_stream_image_mode(videoMode);
      case TY_COMPONENT_RGB_CAM:
        return get_rgb_stream_image_mode(videoMode);
      case TY_COMPONENT_DEPTH_CAM:
        return get_depth_stream_image_mode(videoMode);
    }
  }
  uint32_t percipio_depth_cam::get_leftir_stream_image_mode(const VideoMode& videoMode)
  {
    for(size_t i = 0; i < leftIRVideoMode.size(); i++)
    {
      VideoMode v = leftIRVideoMode[i];
      if((v.getResolutionX() == videoMode.getResolutionX()) && 
         (v.getResolutionY() == videoMode.getResolutionY())) {
          return v.getImageMode();
      }
    }
    return 0;
  }

  uint32_t percipio_depth_cam::get_rightir_stream_image_mode(const VideoMode& videoMode)
  {
    for(size_t i = 0; i < rightIRVideoMode.size(); i++)
    {
      VideoMode v = rightIRVideoMode[i];
      if((v.getResolutionX() == videoMode.getResolutionX()) && 
         (v.getResolutionY() == videoMode.getResolutionY())) {
          return v.getImageMode();
      }
    }
    return 0;
  }
  
  uint32_t percipio_depth_cam::get_rgb_stream_image_mode(const VideoMode& videoMode)
  {
    for(size_t i = 0; i < RGBVideoMode.size(); i++)
    {
      VideoMode v = RGBVideoMode[i];
      if((v.getResolutionX() == videoMode.getResolutionX()) && 
         (v.getResolutionY() == videoMode.getResolutionY())) {
          return v.getImageMode();
      }
    }
    return 0;
  }

  uint32_t percipio_depth_cam::get_depth_stream_image_mode(const VideoMode& videoMode)
  {
    for(size_t i = 0; i < DepthVideoMode.size(); i++)
    {
      VideoMode v = DepthVideoMode[i];
      if((v.getResolutionX() == videoMode.getResolutionX()) && 
         (v.getResolutionY() == videoMode.getResolutionY())) {
          return v.getImageMode();
      }
    }
    return 0;
  }

  ////

  static boost::shared_ptr<percipio_depth_cam> g_Context = NULL;

  CameraSettings* VideoStream::getCameraSettings() 
  {
    return m_pCameraSettings;
  }

  TY_STATUS VideoStream::addNewFrameListener(NewFrameListener* pListener)
  {
    if (!isValid())
    {
      return TY_STATUS_ERROR;
    }
    return g_Context.get()->StreamRegisterNewFrameCallback(m_stream, static_cast<void*>(pListener), pListener->callback);
  }

  void VideoStream::removeNewFrameListener(NewFrameListener* pListener)
  {
    if (!isValid())
    {
      return;
    }
    g_Context.get()->StreamUnregisterNewFrameCallback(m_stream);
  }

  TY_STATUS VideoStream::start()
  {
    if (!isValid()) {
      return TY_STATUS_ERROR;
    }

    g_Context.get()->enable_stream(m_stream);
    return g_Context.get()->StartCapture();
  }

  void VideoStream::stop()
  {
    if (!isValid())
      return;
    g_Context.get()->disable_stream(m_stream);
    g_Context.get()->StopCapture(m_stream);
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
  
  TY_STATUS VideoStream::getProperty(int propertyId, void* data, int* dataSize) const
  {
    if (!isValid())
    {
      return TY_STATUS_ERROR;
    }
    //return (Status)oniStreamGetProperty(m_stream, propertyId, data, dataSize);
  }

  TY_STATUS VideoStream::setProperty(int propertyId, const void* data, int dataSize)
  {
    if (!isValid())
    {
      return TY_STATUS_ERROR; 
    }
    //return (Status)oniStreamSetProperty(m_stream, propertyId, data, dataSize);
  }
  VideoMode VideoStream::getVideoMode() const
  {
    VideoMode videoMode = g_Context.get()->get_current_video_mode(m_stream);
    return videoMode;
  }

  TY_STATUS VideoStream::setVideoMode(const VideoMode& videoMode)
  {
    return g_Context.get()->set_current_video_mode(m_stream, videoMode);
  }

  float VideoStream::getVerticalFieldOfView() const
  {
    float vertical = 0;
    ////TODO
    return vertical;
  }

  TY_STATUS VideoStream::create(const Device& device, SensorType sensorType)
  {
    StreamHandle streamHandle;
    TY_STATUS rc = g_Context.get()->CreateStream(sensorType, streamHandle);
    if (rc != TY_STATUS_OK)
    {
      return rc;
    }
    
    _setHandle(streamHandle);
    m_pCameraSettings = new CameraSettings(this);
    return TY_STATUS_OK;
  }

  void VideoStream::destroy()
  {
    if (!isValid())
    {
      return;
    }
    if (m_pCameraSettings != NULL)
    {
      delete m_pCameraSettings;
      m_pCameraSettings = NULL;
    }
    if (m_stream != NULL)
    {
      ////TODO
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
    ////TODO
    //need bind stream handle & sensorInfo
    m_sensorInfo._setInternal(SENSOR_NONE);
    m_stream = stream;
    if (stream != NULL)
    {
      m_sensorInfo._setInternal(g_Context.get()->StreamGetSensorInfo(m_stream));
    }
  }

  void  VideoStream::parseImageData(TY_IMAGE_DATA* img)
  { 
    //TODO
    VideoFrameData vframe;
    vframe.setData(img);

    VideoFrameData tframe;
    if(img->pixelFormat == TY_PIXEL_FORMAT_DEPTH16) {
      g_Context->parseDepthStream(&vframe, &tframe);
      ImageRegistrationMode mode = g_Context->DeviceGetImageRegistrationMode();
      if(mode == IMAGE_REGISTRATION_DEPTH_TO_COLOR) {
        g_Context->MapDepthFrameToColorCoordinate(&tframe, &frame);
      } else {
        frame.clone(tframe);
      }
    } else if(img->pixelFormat == TY_PIXEL_FORMAT_MONO) {
      frame.clone(vframe);
    } else {
      g_Context->FrameDecoder(vframe, tframe);
      g_Context->parseColorStream(&tframe, &frame);
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
  ////
  SensorType SensorInfo::getSensorType() const 
  { 
    return m_type;
  }
  
  const Array<VideoMode>& SensorInfo::getSupportedVideoModes() const 
  {
    return m_videoModes;
  }

  //SensorInfo(const SensorInfo&);
  //SensorInfo& operator=(const SensorInfo&);

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
      const std::vector<VideoMode>& video_mode = g_Context.get()->getVideoModeList(type);
      if(video_mode.size()) {
        m_videoModes._setData(&video_mode[0], video_mode.size());
      } else {
        m_videoModes._setData(NULL, 0);
      }
    }
  }

  ////
  VideoFrameData::VideoFrameData()
  {
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
  }
  
  VideoFrameData::~VideoFrameData() 
  {
    if(m_isOwner) {
      if(buffer) {
        free(buffer);
        buffer = NULL;
      }
    }
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
    if(size != sz) {
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
  }

  void  VideoFrameData::setData(TY_IMAGE_DATA* data)
  {
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
  }

  void  VideoFrameData::clone(const VideoFrameData& frame)
  {
    timestamp = frame.timestamp;
    imageIndex = frame.imageIndex;
    status = frame.status;
    componentID = frame.componentID;
    size = frame.size;
    
    width = frame.width;
    height = frame.height;
    pixelFormat = frame.pixelFormat;

    buffer = malloc(size);

    memcpy(buffer, frame.getData(), size);

    m_isOwner = true;
  }

  void  VideoFrameData::setPixelFormat(int32_t fmt) 
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

  ////
  TY_STATUS CameraSettings::setLaserPower(int power)
  {
    return setProperty(TY_INT_LASER_POWER, power);
  }
  TY_STATUS CameraSettings::setAutoExposureEnabled(bool enabled)
  {
    return CameraSettings::setProperty(TY_BOOL_AUTO_EXPOSURE, (bool)(enabled ? true : false));
  }
  TY_STATUS CameraSettings::setAutoWhiteBalanceEnabled(bool enabled)
  {
    return setProperty(TY_BOOL_AUTO_AWB, (bool)(enabled ? true : false));
  }
  TY_STATUS CameraSettings::setPixelsAnalogGain(int gain)
  {
    return setProperty(TY_INT_ANALOG_GAIN, gain);
  }
  TY_STATUS CameraSettings::setPixelsRedGain(int gain)
  {
    return CameraSettings::setProperty(TY_INT_R_GAIN, gain);
  }
  TY_STATUS CameraSettings::setPixelsGreenGain(int gain)
  {
    return CameraSettings::setProperty(TY_INT_G_GAIN, gain);
  }
  TY_STATUS CameraSettings::setPixelsBlueGain(int gain)
  {
    return setProperty(TY_INT_B_GAIN, gain);
  }
  TY_STATUS CameraSettings::setGain(int gain)
  {
    return setProperty(TY_INT_GAIN, gain);
  }
  TY_STATUS CameraSettings::setExposure(int exposure)
  {
    return setProperty(TY_INT_EXPOSURE_TIME, exposure);
  }
  TY_STATUS CameraSettings::setTOFCamDepthChannel(int channel)
  {
    return setProperty(TY_INT_TOF_CHANNEL, channel);
  }
  TY_STATUS CameraSettings::setTOFCamDepthQuality(int quality)
  {
    return setProperty(TY_ENUM_DEPTH_QUALITY, quality);
  }

  bool CameraSettings::getLaserPower(int* power) const
  {
    TY_STATUS rc = getProperty(TY_INT_LASER_POWER, power);
    return rc == TY_STATUS_OK;
  }
  bool CameraSettings::getAutoExposureEnabled(bool* enable) const
  {
    TY_STATUS rc = getProperty(TY_BOOL_AUTO_EXPOSURE, enable);
    return rc == TY_STATUS_OK;
  }
  bool CameraSettings::getAutoWhiteBalanceEnabled(bool* enable) const
  {
    TY_STATUS rc = getProperty(TY_BOOL_AUTO_AWB, enable);
    return rc == TY_STATUS_OK;
  }
  bool CameraSettings::getAnalogGain(int* gain) const
  {
    TY_STATUS rc = getProperty(TY_INT_ANALOG_GAIN, gain);
    return rc == TY_STATUS_OK;
  }
  bool CameraSettings::getRedGain(int* gain) const
  {
    TY_STATUS rc = getProperty(TY_INT_R_GAIN, gain);
    return rc == TY_STATUS_OK;
  }
  bool CameraSettings::getGreenGain(int* gain) const
  {
    TY_STATUS rc = getProperty(TY_INT_G_GAIN, gain);
    return rc == TY_STATUS_OK;
  }
  bool CameraSettings::getBlueGain(int* gain) const
  {
    TY_STATUS rc = getProperty(TY_INT_B_GAIN, gain);
    return rc == TY_STATUS_OK;
  }
  int CameraSettings::getExposure(int* exposure)
  {
    TY_STATUS rc = getProperty(TY_INT_EXPOSURE_TIME, exposure);
    return rc == TY_STATUS_OK;
  }
  int CameraSettings::getGain(int* gain) //for percipio ir cam
  {
    TY_STATUS rc = getProperty(TY_INT_GAIN, gain);
    return rc == TY_STATUS_OK;
  }
  int CameraSettings::getTOFCamDepthChannel(int* channel)
  {
    TY_STATUS rc = getProperty(TY_INT_TOF_CHANNEL, channel);
    return rc == TY_STATUS_OK;
  }
  int CameraSettings::getTOFCamDepthQuality(int* quality)
  {
    TY_STATUS rc = getProperty(TY_ENUM_DEPTH_QUALITY, quality);
    return rc == TY_STATUS_OK;
  }

  TY_STATUS CameraSettings::setDepthSgbmImageChanNumber(int value)
  {
    return setProperty(TY_INT_SGBM_IMAGE_NUM, value);
  }
  TY_STATUS CameraSettings::setDepthSgbmDispNumber(int value)
  {
    return setProperty(TY_INT_SGBM_DISPARITY_NUM, value);
  }
  TY_STATUS CameraSettings::setDepthSgbmDispOffset(int value)
  {
    return setProperty(TY_INT_SGBM_DISPARITY_OFFSET, value);
  }
  TY_STATUS CameraSettings::setDepthSgbmMatchWinHeight(int value)
  {
    return setProperty(TY_INT_SGBM_MATCH_WIN_HEIGHT, value);
  }
  TY_STATUS CameraSettings::setDepthSgbmSemiP1(int value)
  {
    return setProperty(TY_INT_SGBM_SEMI_PARAM_P1, value);
  }
  TY_STATUS CameraSettings::setDepthSgbmSemiP2(int value)
  {
    return setProperty(TY_INT_SGBM_SEMI_PARAM_P2, value);
  }
  TY_STATUS CameraSettings::setDepthSgbmUniqueFactor(int value)
  {
    return setProperty(TY_INT_SGBM_UNIQUE_FACTOR, value);
  }
  TY_STATUS CameraSettings::setDepthSgbmUniqueAbsDiff(int value)
  {
    return setProperty(TY_INT_SGBM_UNIQUE_ABSDIFF, value);
  }
  TY_STATUS CameraSettings::setDepthSgbmCostParam(int value)
  {
    return setProperty(TY_INT_SGBM_COST_PARAM, value);
  }
  TY_STATUS CameraSettings::setDepthSgbmHalfWinSizeEn(int value)
  {
    return setProperty(TY_BOOL_SGBM_HFILTER_HALF_WIN, static_cast<bool>(value));
  }
  TY_STATUS CameraSettings::setDepthSgbmMatchWinWidth(int value)
  {
    return setProperty(TY_INT_SGBM_MATCH_WIN_WIDTH, value);
  }
  TY_STATUS CameraSettings::setDepthSgbmMedianFilterEn(int value)
  {
    return setProperty(TY_BOOL_SGBM_MEDFILTER, static_cast<bool>(value));
  }
  TY_STATUS CameraSettings::setDepthSgbmLRCCheckEn(int value)
  {
    return setProperty(TY_BOOL_SGBM_LRC, static_cast<bool>(value));
  }
  TY_STATUS CameraSettings::setDepthSgbmLRCMaxDiff(int value)
  {
    return setProperty(TY_INT_SGBM_LRC_DIFF, value);
  }
  TY_STATUS CameraSettings::setDepthSgbmMedianFilterThresh(int value)
  {
    return setProperty(TY_INT_SGBM_MEDFILTER_THRESH, value);
  }
  TY_STATUS CameraSettings::setDepthSgbmSemiP1Scale(int value)
  {
    return setProperty(TY_INT_SGBM_SEMI_PARAM_P1_SCALE, value);
  }
  
  bool CameraSettings::isValid() const 
  {
    return m_pStream != NULL;
  }

  template <class T>
  TY_STATUS CameraSettings::getProperty(uint32_t propertyId, T* value) const
  {
    if (!isValid()) { printf("id[%d] is not valid!\n", propertyId); return TY_STATUS_ERROR; }
    return g_Context.get()->getProperty<T>(m_pStream->_getHandle(), propertyId, value);
  }
  template <class T>
  TY_STATUS CameraSettings::setProperty(uint32_t propertyId, const T& value)
  {
    if (!isValid()) {printf("id[%d] is not valid!\n", propertyId); return TY_STATUS_ERROR; }
    return g_Context.get()->setProperty<T>(m_pStream->_getHandle(), propertyId, value);
  }

  CameraSettings::CameraSettings(VideoStream* pStream)
  {
    m_pStream = pStream;
  }

  TY_STATUS Device::open(const char* uri)
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

    rc = g_Context.get()->open(uri);
    if(rc != TY_STATUS_OK)
      return rc;

    DeviceGetInfo();

    return TY_STATUS_OK;
  }

  void Device::close()
  {
    if(!m_isOwner)
    {
      g_Context.get()->close();
    }
  }

  bool Device::isValid() const 
  { 
    return g_Context.get()->isValid(); 
  }

  void Device::setColorUndistortion(bool enabled)
  {
    g_Context.get()->DeviceSetColorUndistortionEnable(enabled);
  }

  bool Device::isImageRegistrationModeSupported(ImageRegistrationMode mode) const
  {
    return (g_Context.get()->DeviceIsImageRegistrationModeSupported() == true);
  }

  TY_STATUS Device::setImageRegistrationMode(ImageRegistrationMode mode)
  {
    return g_Context.get()->DeviceSetImageRegistrationMode(mode);
  }

  ImageRegistrationMode Device::getImageRegistrationMode() const
  {
    return g_Context.get()->DeviceGetImageRegistrationMode();
  }

  bool Device::isDepthColorSyncSupport()
  {
    return g_Context.get()->isDepthColorSyncSupport();
  }

  TY_STATUS Device::setDepthColorSyncEnabled(bool isEnabled)
  {
    TY_STATUS rc = TY_STATUS_OK;
    if (isEnabled)
    {
      rc = g_Context.get()->DeviceEnableDepthColorSync();
    }
    else
    {
      rc = g_Context.get()->DeviceDisableDepthColorSync();
    }
    return rc;
  }

  bool Device::getDepthColorSyncEnabled()
  {
    return g_Context.get()->DeviceGetDepthColorSyncEnabled();
  }

  TY_STATUS Device::getProperty(int propertyId, void* data, int* dataSize) const
  {
    return g_Context.get()->DeviceGetProperty(propertyId, data, dataSize);
  }

  TY_STATUS Device::setProperty(int propertyId, const void* data, int* dataSize)
  {
    return g_Context.get()->DeviceSetProperty(propertyId, data, dataSize);
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

  bool Device::ReslotionSetting(SensorType sensorType, int width, int height)
  {
    //TODO
    switch (sensorType)
    {
      case SENSOR_COLOR:
        return g_Context->set_image_mode(TY_COMPONENT_RGB_CAM, width, height);
	    case SENSOR_DEPTH:
        return g_Context->set_image_mode(TY_COMPONENT_DEPTH_CAM, width, height);
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
    m_deviceInfo = DeviceInfo(info.id, info.vendorName, info.modelName);
    if(info.iface.type == TY_INTERFACE_USB) {
      m_deviceInfo.setUsbVendorId(PERCIPIO_USB_VID);
      m_deviceInfo.setUsbProductId(PERCIPIO_USB_PID);
    } else {
      m_deviceInfo.setUsbVendorId(0);
      m_deviceInfo.setUsbProductId(0);
    }
  }

  ////
  TY_STATUS Percipio::addDeviceConnectedListener(DeviceConnectedListener* pListener)
  {
    return g_Context.get()->RegisterDeviceCallbacks(&pListener->m_deviceConnectedCallbacks, pListener);
  }

  void Percipio::removeDeviceConnectedListener(DeviceConnectedListener* pListener)
  {
    g_Context.get()->UnregisterDeviceCallbacks(pListener);
  }

  TY_STATUS Percipio::addDeviceDisconnectedListener(DeviceDisconnectedListener* pListener)
  {
    return g_Context.get()->RegisterDeviceCallbacks(&pListener->m_deviceDisconnectedCallbacks, pListener);
  } 

  void Percipio::removeDeviceDisconnectedListener(DeviceDisconnectedListener* pListener)
  {
    g_Context.get()->UnregisterDeviceCallbacks(pListener);
  } 

  TY_STATUS Percipio::initialize()
  {
    if(g_Context.get() == NULL) {
      g_Context = boost::make_shared<percipio_depth_cam>();
      return g_Context.get()->initialize();
    } else {
      return TY_STATUS_OK;
    }
  } 

  void Percipio::enumerateDevices(Array<DeviceInfo>* deviceInfoList)
  {
    DeviceInfo* m_pDeviceInfos;
    int m_deviceInfoCount;
    g_Context.get()->GetDeviceList(&m_pDeviceInfos, &m_deviceInfoCount);
    deviceInfoList->_setData((DeviceInfo*)m_pDeviceInfos, m_deviceInfoCount, true);
    delete []m_pDeviceInfos;
  } 

  const char* Percipio::getExtendedError(TY_STATUS status) {
    return TYErrorString(status);
  }
}