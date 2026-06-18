#include "TYApi.h"

#include <boost/lexical_cast.hpp>
#include <boost/algorithm/string/replace.hpp>

#include "percipio_camera/percipio_device.h"
#include "percipio_camera/percipio_exception.h"
#include "percipio_camera/percipio_convert.h"
#include "percipio_camera/percipio_frame_listener.h"

#include <ros/ros.h>

#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>

#include <string>
#include "percipio_camera/percipio_interface.h"

namespace percipio
{

using percipio_wrapper::throwPercipioException;

PercipioDevice::PercipioDevice() :
    m_isOwner(true),
    ir_video_started_(false),
    color_video_started_(false),
    depth_video_started_(false),
    point3d_video_started_(false),
    image_registration_activated_(false),
    percipio_device_time_(false)
{
  ir_frame_listener = boost::make_shared<percipio_wrapper::PercipioFrameListener>("IR-Stream");
  color_frame_listener = boost::make_shared<percipio_wrapper::PercipioFrameListener>("Color-Stream");
  depth_frame_listener = boost::make_shared<percipio_wrapper::PercipioFrameListener>("Depth-Stream");
  point3d_frame_listener = boost::make_shared<percipio_wrapper::PercipioFrameListener>("Point3D-Stream");
}

PercipioDevice::PercipioDevice(const std::string& device_URI, const bool auto_reconnect) :
    m_isOwner(true),
    ir_video_started_(false),
    color_video_started_(false),
    depth_video_started_(false),
    point3d_video_started_(false),
    image_registration_activated_(false),
    percipio_device_time_(false)
{
  TY_STATUS rc = Percipio::initialize();
  if (rc != TY_STATUS_OK)
    THROW_PERCIPIO_EXCEPTION("Initialize failed\n%d\n", rc);

  if (device_URI.length() > 0)
  {
    rc = open(device_URI.c_str(), auto_reconnect);
  }
  else
  {
    rc = open(ANY_DEVICE, auto_reconnect);
  }

  if (rc != TY_STATUS_OK)
    THROW_PERCIPIO_EXCEPTION("Device open failed\n%d\n", rc);

  DeviceGetInfo();

  ir_frame_listener = boost::make_shared<percipio_wrapper::PercipioFrameListener>("IR-Stream");
  color_frame_listener = boost::make_shared<percipio_wrapper::PercipioFrameListener>("Color-Stream");
  depth_frame_listener = boost::make_shared<percipio_wrapper::PercipioFrameListener>("Depth-Stream");
  point3d_frame_listener = boost::make_shared<percipio_wrapper::PercipioFrameListener>("Point3D-Stream");

  boost::shared_ptr<VideoStream> ir_stream = getIRVideoStream();
  if (ir_stream)
    ir_stream->addNewFrameListener(ir_frame_listener.get());

  boost::shared_ptr<VideoStream> color_stream = getColorVideoStream();
  if (color_stream)
    color_stream->addNewFrameListener(color_frame_listener.get());

  boost::shared_ptr<VideoStream> depth_stream = getDepthVideoStream();
  if (depth_stream)
    depth_stream->addNewFrameListener(depth_frame_listener.get());

  boost::shared_ptr<VideoStream> point3d_stream = getPoint3DVideoStream();
  if(point3d_stream)
    point3d_stream->addNewFrameListener(point3d_frame_listener.get());
}

PercipioDevice::~PercipioDevice()
{
  stopAllStreams();

  if (ir_video_stream_)
    ir_video_stream_->removeNewFrameListener(ir_frame_listener.get());
  if(color_video_stream_)
    color_video_stream_->removeNewFrameListener(color_frame_listener.get());
  if(depth_video_stream_)
    depth_video_stream_->removeNewFrameListener(depth_frame_listener.get());
  if(point3d_video_stream_)
    point3d_video_stream_->removeNewFrameListener(point3d_frame_listener.get());
  shutdown();
  close();
}

TY_STATUS PercipioDevice::open(const char* uri, const bool auto_reconnect)
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

  Percipio::getContext()->b_auto_reconnect = auto_reconnect;

  bool isIP = IPv4_verify(uri);
  if(isIP)
    rc = Percipio::getContext()->openWithIP(uri);
  else
    rc = Percipio::getContext()->openWithSN(uri);

  return rc;
}

void PercipioDevice::close()
{
  if(!m_isOwner)
  {
    Percipio::getContext()->close();
  }
}

const std::string PercipioDevice::getUri() const
{
  return std::string(m_deviceInfo.getUri());
}

const std::string PercipioDevice::getVendor() const
{
  return std::string(m_deviceInfo.getVendor());
}

const std::string PercipioDevice::getName() const
{
  return std::string(m_deviceInfo.getName());
}

uint16_t PercipioDevice::getUsbVendorId() const
{
  return m_deviceInfo.getUsbVendorId();
}

uint16_t PercipioDevice::getUsbProductId() const
{
  return m_deviceInfo.getUsbProductId();
}

const std::string PercipioDevice::getStringID() const
{
  std::string ID_str = getName() + "_" + getVendor();

  boost::replace_all(ID_str, "/", "");
  boost::replace_all(ID_str, ".", "");
  boost::replace_all(ID_str, "@", "");

  return ID_str;
}

const DeviceInfo& PercipioDevice::getDeviceInfo() const
{
  return m_deviceInfo;
}

boost::shared_ptr<PercipioDepthCam> PercipioDevice::DevicePtr()
{
  return Percipio::getContext();
}

bool PercipioDevice::isValid() const
{
  return Percipio::getContext() && Percipio::getContext()->isValid();
}

bool PercipioDevice::hasIRSensor() const
{
  return hasSensor(SENSOR_IR_LEFT);
}

bool PercipioDevice::hasColorSensor() const
{
  return hasSensor(SENSOR_COLOR);
}

bool PercipioDevice::hasDepthSensor() const
{
  return hasSensor(SENSOR_DEPTH);
}

bool PercipioDevice::hasPoint3DSensor() const
{
  return hasSensor(SENSOR_POINT3D);
}

bool PercipioDevice::hasSensor(SensorType sensorType) const
{
  bool has = false;
  switch (sensorType)
  {
  case SENSOR_IR_LEFT:
  {
    if(Percipio::getContext()->get_components() & TY_COMPONENT_IR_CAM_LEFT)
      has = true;
    break;
  }
  case SENSOR_IR_RIGHT:
  {
    if(Percipio::getContext()->get_components() & TY_COMPONENT_IR_CAM_RIGHT)
      has = true;
    break;
  }
  case SENSOR_COLOR:
  {
    if(Percipio::getContext()->get_components() & TY_COMPONENT_RGB_CAM)
      has = true;
    break;
  }
  case SENSOR_DEPTH:
  case SENSOR_POINT3D:
  {
    if(Percipio::getContext()->get_components() & TY_COMPONENT_DEPTH_CAM)
      has = true;
    break;
  }
  default:
    break;
  }
  return has;
}

bool PercipioDevice::ResolutionSetting(SensorType sensorType, int width, int height, const std::string& fmt)
{
  switch (sensorType)
  {
    case SENSOR_COLOR:
      return Percipio::getContext()->set_image_mode(SENSOR_COLOR, width, height, fmt);
    case SENSOR_DEPTH:
      return Percipio::getContext()->set_image_mode(SENSOR_DEPTH, width, height, fmt);
    default:
      return false;
  }
}

bool PercipioDevice::setColorResolution(int w, int h, const std::string& format)
{
  if(hasSensor(SENSOR_COLOR))
  {
    return ResolutionSetting(SENSOR_COLOR, w, h, format);
  }
  else
    return false;
}

bool PercipioDevice::setDepthResolutuon(int w, int h, const std::string& format)
{
  if(hasSensor(SENSOR_DEPTH))
  {
    return ResolutionSetting(SENSOR_DEPTH, w, h, format);
  }
  else
    return false;
}

void PercipioDevice::setStreamResendEnable(bool enabled)
{
  (void)enabled;
}

void PercipioDevice::startIRStream()
{
  boost::shared_ptr<VideoStream> stream = getIRVideoStream();
  if (stream)
  {
    ir_stream_topic_enable(true);
    stream->start();
    ir_video_started_ = true;
  }
}

void PercipioDevice::startColorStream()
{
  boost::shared_ptr<VideoStream> stream = getColorVideoStream();
  if (stream)
  {
    color_stream_topic_enable(true);
    stream->start();
    color_video_started_ = true;
  }
}

void PercipioDevice::startDepthStream()
{
  boost::shared_ptr<VideoStream> stream = getDepthVideoStream();
  if (stream)
  {
    depth_stream_topic_enable(true);
    stream->start();
    depth_video_started_ = true;
  }
  else
  {
    printf("invalid stream!\n");
  }
}

void PercipioDevice::startPoint3DStream()
{
  boost::shared_ptr<VideoStream> stream = getPoint3DVideoStream();
  if(stream)
  {
    pointcloud_stream_topic_enable(true);
    stream->start();
    point3d_video_started_ = true;
  }
  else
  {
    printf("invalid stream!\n");
  }
}

void PercipioDevice::stopAllStreams()
{
  stopIRStream();
  stopColorStream();
  stopDepthStream();
  stopPoint3DStream();
}

void PercipioDevice::stopIRStream()
{
  if (ir_video_stream_)
  {
    ir_video_started_ = false;
    ir_stream_topic_enable(false);
    ir_video_stream_->stop();
  }
}

void PercipioDevice::stopColorStream()
{
  if (color_video_stream_)
  {
    color_video_started_ = false;
    color_stream_topic_enable(false);
    color_video_stream_->stop();
  }
}

void PercipioDevice::stopDepthStream()
{
  if (depth_video_stream_)
  {
    depth_video_started_ = false;
    depth_stream_topic_enable(false);
    depth_video_stream_->stop();
  }
}

void PercipioDevice::stopPoint3DStream()
{
  if(point3d_video_stream_)
  {
    point3d_video_started_ = false;
    pointcloud_stream_topic_enable(false);
    point3d_video_stream_->stop();
  }
}

void PercipioDevice::shutdown()
{
  if (ir_video_stream_)
    ir_video_stream_->destroy();

  if (color_video_stream_)
    color_video_stream_->destroy();

  if (depth_video_stream_)
    depth_video_stream_->destroy();
}

bool PercipioDevice::isIRStreamStarted()
{
  return ir_video_started_;
}

bool PercipioDevice::isColorStreamStarted()
{
  return color_video_started_;
}

bool PercipioDevice::isDepthStreamStarted()
{
  return depth_video_started_;
}

bool PercipioDevice::isPoint3DStreamStarted()
{
  return point3d_video_started_;
}

void PercipioDevice::setColorUndistortion(bool enabled)
{
  Percipio::getContext()->DeviceSetColorUndistortionEnable(enabled);
}

void PercipioDevice::setIRUndistortion(bool enabled)
{
  Percipio::getContext()->DeviceSetIRUndistortionEnable(enabled);
}

bool PercipioDevice::setDepthSpecFilterEn(bool en)
{
  return Percipio::getContext()->DepthStreamSetSpeckFilterEn(en);
}

bool PercipioDevice::getDepthSpecFilterEn()
{
  return Percipio::getContext()->DepthStreamGetSpeckFilterEn();
}

bool PercipioDevice::setDepthSpecFilterSpecSize(int spec_size)
{
  return Percipio::getContext()->DepthStreamSetSpeckFilterSpecSize(spec_size);
}

int PercipioDevice::getDepthSpecFilterSpecSize()
{
  return Percipio::getContext()->DepthStreamGetSpeckFilterSpecSize();
}

bool PercipioDevice::setDepthSpeckFilterDiff(int spec_diff)
{
  return Percipio::getContext()->DepthStreamSetSpeckFilterDiff(spec_diff);
}

int PercipioDevice::getDepthSpeckFilterDiff()
{
  return Percipio::getContext()->DepthStreamGetSpeckFilterDiff();
}

bool PercipioDevice::setDepthSpeckFilterPhySize(float phy_size)
{
  return Percipio::getContext()->DepthStreamSetSpeckFilterPhySize(phy_size);
}

float PercipioDevice::getDepthSpeckFilterPhySize()
{
  return Percipio::getContext()->DepthStreamGetSpeckFilterPhySize();
}

bool PercipioDevice::setDepthTimeDomainFilterEn(bool en)
{
  return Percipio::getContext()->DepthStreamSetTimeDomainFilterEn(en);
}

bool PercipioDevice::getDepthTimeDomainFilterEn()
{
  return Percipio::getContext()->DepthStreamGetTimeDomainFilterEn();
}

bool PercipioDevice::setDepthTimeDomainFilterNum(int frames)
{
  return Percipio::getContext()->DepthStreamSetTimeDomainFilterFCnt(frames);
}

int PercipioDevice::getDepthTimeDomainFilterNum()
{
  return Percipio::getContext()->DepthStreamGetTimeDomainFilterFCnt();
}

void PercipioDevice::setIREnhancement(IREnhanceModel model, int coeff)
{
  Percipio::getContext()->IRStreamEnhancementConfig(model, coeff);
}

bool PercipioDevice::isImageRegistrationModeSupported(ImageRegistrationMode mode) const
{
  return (Percipio::getContext()->DeviceIsImageRegistrationModeSupported() == true);
}

TY_STATUS PercipioDevice::setImageRegistrationMode(ImageRegistrationMode mode)
{
  return Percipio::getContext()->DeviceSetImageRegistrationMode(mode);
}

ImageRegistrationMode PercipioDevice::getImageRegistrationMode() const
{
  return Percipio::getContext()->DeviceGetImageRegistrationMode();
}

bool PercipioDevice::isImageRegistrationModeSupported() const
{
  return isImageRegistrationModeSupported(IMAGE_REGISTRATION_DEPTH_TO_COLOR);
}

void PercipioDevice::setImageRegistrationMode(bool enabled)
{
  if (isImageRegistrationModeSupported())
  {
    image_registration_activated_ = enabled;
    if (enabled)
    {
      TY_STATUS rc = Percipio::getContext()->DeviceSetImageRegistrationMode(IMAGE_REGISTRATION_DEPTH_TO_COLOR);
      if (rc != TY_STATUS_OK)
        THROW_PERCIPIO_EXCEPTION("Enabling image registration mode failed: \n%s\n", Percipio::getExtendedError(rc));
    }
    else
    {
      TY_STATUS rc = Percipio::getContext()->DeviceSetImageRegistrationMode(IMAGE_REGISTRATION_OFF);
      if (rc != TY_STATUS_OK)
        THROW_PERCIPIO_EXCEPTION("Enabling image registration mode failed: \n%s\n", Percipio::getExtendedError(rc));
    }
  } else {
    printf("Not support registration!\n");
  }
}

void PercipioDevice::ir_stream_topic_enable(bool enabled)
{
  Percipio::getContext()->EnableIRTopic(enabled);
}

void PercipioDevice::color_stream_topic_enable(bool enabled)
{
  Percipio::getContext()->EnableColorTopic(enabled);
}

void PercipioDevice::depth_stream_topic_enable(bool enabled)
{
  Percipio::getContext()->EnableDepthTopic(enabled);
}

void PercipioDevice::pointcloud_stream_topic_enable(bool enabled)
{
  Percipio::getContext()->EnableP3DTopic(enabled);
}

TY_STATUS PercipioDevice::PropertyGet(TY_COMPONENT_ID comp, TY_FEATURE_ID feat, void* ptr, size_t size)
{
  return Percipio::getContext()->DeviceGetProperty(comp, feat, ptr, size);
}

TY_STATUS PercipioDevice::PropertySet(TY_COMPONENT_ID comp, TY_FEATURE_ID feat, void* ptr, size_t size)
{
  return Percipio::getContext()->DeviceSetProperty(comp, feat, ptr, size);
}

const std::vector<percipio_wrapper::PercipioVideoMode>& PercipioDevice::getSupportedIRVideoModes() const
{
  boost::shared_ptr<VideoStream> stream = getIRVideoStream();
  ir_video_modes_.clear();
  if (stream)
  {
    const SensorInfo& sensor_info = stream->getSensorInfo();
    ir_video_modes_ = percipio_wrapper::percipio_convert(sensor_info.getSupportedVideoModes());
  }
  return ir_video_modes_;
}

const std::vector<percipio_wrapper::PercipioVideoMode>& PercipioDevice::getSupportedColorVideoModes() const
{
  boost::shared_ptr<VideoStream> stream = getColorVideoStream();
  color_video_modes_.clear();
  if (stream)
  {
    const SensorInfo& sensor_info = stream->getSensorInfo();
    color_video_modes_ = percipio_wrapper::percipio_convert(sensor_info.getSupportedVideoModes());
  }
  return color_video_modes_;
}

const std::vector<percipio_wrapper::PercipioVideoMode>& PercipioDevice::getSupportedDepthVideoModes() const
{
  boost::shared_ptr<VideoStream> stream = getDepthVideoStream();
  depth_video_modes_.clear();
  if (stream)
  {
    const SensorInfo& sensor_info = stream->getSensorInfo();
    depth_video_modes_ = percipio_wrapper::percipio_convert(sensor_info.getSupportedVideoModes());
  }
  return depth_video_modes_;
}

void PercipioDevice::setIRFrameCallback(boost::shared_ptr<percipio_wrapper::FrameCallbackFunction>& callback)
{
  ir_frame_listener->setCallback(callback);
}

void PercipioDevice::setColorFrameCallback(boost::shared_ptr<percipio_wrapper::FrameCallbackFunction>& callback)
{
  color_frame_listener->setCallback(callback);
}

void PercipioDevice::setDepthFrameCallback(boost::shared_ptr<percipio_wrapper::FrameCallbackFunction>& callback)
{
  depth_frame_listener->setCallback(callback);
}

void PercipioDevice::setPoint3DFrameCallback(boost::shared_ptr<percipio_wrapper::FrameCallbackFunction>& callback)
{
  point3d_frame_listener->setCallback(callback);
}

void PercipioDevice::setDeviceInitCallback(boost::shared_ptr<DeviceInitCallbackFunction>& callback)
{
  Percipio::getContext()->setDeviceInitCallback(callback);
}

void PercipioDevice::setDeviceEventCallback(boost::shared_ptr<DeviceEventCallbackFunction>& callback)
{
  Percipio::getContext()->setDeviceEventCallback(callback);
}

void PercipioDevice::setUseDeviceTimer(bool enable)
{
  percipio_device_time_ = enable;
}

void PercipioDevice::setDeviceWorkMode(bool enable_rate, float frame_rate, bool trigger_mode_en)
{
  return Percipio::getContext()->set_work_mode(enable_rate, frame_rate, trigger_mode_en);
}

int PercipioDevice::sendSoftTrigger()
{
  return Percipio::getContext()->send_soft_trigger();
}

void PercipioDevice::reset()
{
  return Percipio::getContext()->reset();
}

void PercipioDevice::dynamic_configure(const std::string& str)
{
  return Percipio::getContext()->dynamic_configure(str);
}

float PercipioDevice::getDepthScale()
{
  return Percipio::getContext()->getDepthScaleUnit();
}

int PercipioDevice::init_parameters_from_xml(const std::string& xml)
{
  return Percipio::getContext()->parse_xml_parameters(xml);
}

bool PercipioDevice::getDepthCalibIntristic(void* data, const size_t size)
{
  auto intr = Percipio::getContext()->getDepthIntr();
  if(size != sizeof(intr))
    return false;

  memcpy(data, &intr, sizeof(intr));
  return true;
}

bool PercipioDevice::getColorCalibIntristic(void* data, const size_t size)
{
  auto intr = Percipio::getContext()->getColorIntr();
  if(size != sizeof(intr))
    return false;

  memcpy(data, &intr, sizeof(intr));
  return true;
}

bool PercipioDevice::getColorCalibDistortion(void* data, const size_t size)
{
  auto dist = Percipio::getContext()->getColorDist();
  if(size != sizeof(dist))
    return false;

  memcpy(data, &dist, sizeof(dist));
  return true;
}

boost::shared_ptr<VideoStream> PercipioDevice::getIRVideoStream() const
{
  if(!ir_video_stream_)
  {
    if (hasIRSensor())
    {
      ir_video_stream_ = boost::make_shared<VideoStream>("IR");
      const TY_STATUS rc = ir_video_stream_->create(*const_cast<PercipioDevice*>(this), SENSOR_IR_LEFT);
      if (rc != TY_STATUS_OK)
        THROW_PERCIPIO_EXCEPTION("Couldn't create IR video stream: \n%s\n", Percipio::getExtendedError(rc));
    }
  }

  return ir_video_stream_;
}

boost::shared_ptr<VideoStream> PercipioDevice::getColorVideoStream() const
{
  if(!color_video_stream_)
  {
    if (hasColorSensor())
    {
      color_video_stream_ = boost::make_shared<VideoStream>("Color");
      const TY_STATUS rc = color_video_stream_->create(*const_cast<PercipioDevice*>(this), SENSOR_COLOR);
      if (rc != TY_STATUS_OK)
        THROW_PERCIPIO_EXCEPTION("Couldn't create color video stream: \n%s\n", Percipio::getExtendedError(rc));
    }
  }
  return color_video_stream_;
}

boost::shared_ptr<VideoStream> PercipioDevice::getDepthVideoStream() const
{
  if(!depth_video_stream_)
  {
    if (hasDepthSensor())
    {
      depth_video_stream_ = boost::make_shared<VideoStream>("Depth");
      const TY_STATUS rc = depth_video_stream_->create(*const_cast<PercipioDevice*>(this), SENSOR_DEPTH);
      if (rc != TY_STATUS_OK)
        THROW_PERCIPIO_EXCEPTION("Couldn't create depth video stream: \n%s\n", Percipio::getExtendedError(rc));
    }
  }
  return depth_video_stream_;
}

boost::shared_ptr<VideoStream> PercipioDevice::getPoint3DVideoStream() const
{
  if(!point3d_video_stream_)
  {
    if (hasPoint3DSensor())
    {
      point3d_video_stream_ = boost::make_shared<VideoStream>("Point3D");
      const TY_STATUS rc = point3d_video_stream_->create(*const_cast<PercipioDevice*>(this), SENSOR_POINT3D);
      if (rc != TY_STATUS_OK)
        THROW_PERCIPIO_EXCEPTION("Couldn't create point3d video stream: \n%s\n", Percipio::getExtendedError(rc));
    }
  }
  return point3d_video_stream_;
}

TY_STATUS PercipioDevice::DeviceGetInfo()
{
  m_deviceInfo = Percipio::getContext()->get_current_device_info();
  return TY_STATUS_OK;
}

bool PercipioDevice::IPv4_verify(const char *ip)
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

std::ostream& operator <<(std::ostream& stream, const PercipioDevice& device)
{
  stream << "Device info (" << device.getUri() << ")" << std::endl;
  stream << "   Vendor: " << device.getVendor() << std::endl;
  stream << "   Name: " << device.getName() << std::endl;
  stream << "   USB Vendor ID: " << device.getUsbVendorId() << std::endl;
  stream << "   USB Product ID: " << device.getUsbVendorId() << std::endl << std::endl;

  if (device.hasIRSensor())
  {
    stream << "IR sensor video modes:" << std::endl;
    const std::vector<percipio_wrapper::PercipioVideoMode>& video_modes = device.getSupportedIRVideoModes();
    std::vector<percipio_wrapper::PercipioVideoMode>::const_iterator it = video_modes.begin();
    std::vector<percipio_wrapper::PercipioVideoMode>::const_iterator it_end = video_modes.end();
    for (; it != it_end; ++it)
      stream << "   - " << *it << std::endl;
  }
  else
  {
    stream << "No IR sensor available" << std::endl;
  }

  if (device.hasColorSensor())
  {
    stream << "Color sensor video modes:" << std::endl;
    const std::vector<percipio_wrapper::PercipioVideoMode>& video_modes = device.getSupportedColorVideoModes();

    std::vector<percipio_wrapper::PercipioVideoMode>::const_iterator it = video_modes.begin();
    std::vector<percipio_wrapper::PercipioVideoMode>::const_iterator it_end = video_modes.end();
    for (; it != it_end; ++it)
      stream << "   - " << *it << std::endl;
  }
  else
  {
    stream << "No Color sensor available" << std::endl;
  }

  if (device.hasDepthSensor())
  {
    stream << "Depth sensor video modes:" << std::endl;
    const std::vector<percipio_wrapper::PercipioVideoMode>& video_modes = device.getSupportedDepthVideoModes();

    std::vector<percipio_wrapper::PercipioVideoMode>::const_iterator it = video_modes.begin();
    std::vector<percipio_wrapper::PercipioVideoMode>::const_iterator it_end = video_modes.end();
    for (; it != it_end; ++it)
      stream << "   - " << *it << std::endl;
  }
  else
  {
    stream << "No Depth sensor available" << std::endl;
  }

  return stream;
}

}
