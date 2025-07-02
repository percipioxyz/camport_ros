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


namespace percipio_wrapper
{

PercipioDevice::PercipioDevice(const std::string& device_URI, const bool auto_reconnect) :
    percipio_device_(),
    ir_video_started_(false),
    color_video_started_(false),
    depth_video_started_(false),
    image_registration_activated_(false),
    percipio_device_time_(false)
{
  TY_STATUS rc = percipio::Percipio::initialize();
  if (rc != TY_STATUS_OK)
    THROW_PERCIPIO_EXCEPTION("Initialize failed\n%d\n", rc);

  percipio_device_ = boost::make_shared<percipio::Device>();
  if (device_URI.length() > 0)
  {
    rc = percipio_device_->open(device_URI.c_str(), auto_reconnect);
  }
  else
  {
    rc = percipio_device_->open(percipio::ANY_DEVICE, auto_reconnect);
  }

  if (rc != TY_STATUS_OK)
    THROW_PERCIPIO_EXCEPTION("Device open failed\n%d\n", rc);

  device_info_ = boost::make_shared<percipio::DeviceInfo>();
  *device_info_ = percipio_device_->getDeviceInfo();

  ir_frame_listener = boost::make_shared<PercipioFrameListener>();
  color_frame_listener = boost::make_shared<PercipioFrameListener>();
  depth_frame_listener = boost::make_shared<PercipioFrameListener>();
  point3d_frame_listener = boost::make_shared<PercipioFrameListener>();

  //m_sensorInfo._setInternal(g_Context.get()->StreamGetSensorInfo(m_stream));
  boost::shared_ptr<percipio::VideoStream> ir_stream = getIRVideoStream();
  if (ir_stream)
    ir_stream->addNewFrameListener(ir_frame_listener.get());

  boost::shared_ptr<percipio::VideoStream> color_stream = getColorVideoStream();
  if (color_stream)
    color_stream->addNewFrameListener(color_frame_listener.get());

  boost::shared_ptr<percipio::VideoStream> depth_stream = getDepthVideoStream();
  if (depth_stream)
    depth_stream->addNewFrameListener(depth_frame_listener.get());

  boost::shared_ptr<percipio::VideoStream> point3d_stream = getPoint3DVideoStream();
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

  percipio_device_->close();
}

const std::string PercipioDevice::getUri() const
{
  return std::string(device_info_->getUri());
}

const std::string PercipioDevice::getVendor() const
{
  return std::string(device_info_->getVendor());
}

const std::string PercipioDevice::getName() const
{
  return std::string(device_info_->getName());
}

uint16_t PercipioDevice::getUsbVendorId() const
{
  return device_info_->getUsbVendorId();
}

uint16_t PercipioDevice::getUsbProductId() const
{
  return device_info_->getUsbProductId();
}

const std::string PercipioDevice::getStringID() const
{
  std::string ID_str = getName() + "_" + getVendor();

  boost::replace_all(ID_str, "/", "");
  boost::replace_all(ID_str, ".", "");
  boost::replace_all(ID_str, "@", "");

  return ID_str;
}

bool PercipioDevice::isValid() const
{
  return percipio_device_ && percipio_device_->isValid();
}

bool PercipioDevice::hasIRSensor() const
{
  return percipio_device_->hasSensor(percipio::SENSOR_IR_LEFT);
}

bool PercipioDevice::hasColorSensor() const
{
  return percipio_device_->hasSensor(percipio::SENSOR_COLOR);
}

bool PercipioDevice::hasDepthSensor() const
{
  return percipio_device_->hasSensor(percipio::SENSOR_DEPTH);
}

bool PercipioDevice::hasPoint3DSensor() const
{
  return percipio_device_->hasSensor(percipio::SENSOR_POINT3D);
}

bool PercipioDevice::setColorResolution(int w, int h)
{
  if(percipio_device_->hasSensor(percipio::SENSOR_COLOR))
  {
    std::string fmt = "";
    return percipio_device_->ResolutionSetting(percipio::SENSOR_COLOR, w, h, fmt);
  }
  else
    return false;
}

bool PercipioDevice::setDepthResolutuon(int w, int h)
{
  if(percipio_device_->hasSensor(percipio::SENSOR_DEPTH))
  {
    std::string fmt = "";
    return percipio_device_->ResolutionSetting(percipio::SENSOR_DEPTH, w, h, fmt);
  }
  else
    return false;
}

void PercipioDevice::startIRStream()
{
  boost::shared_ptr<percipio::VideoStream> stream = getIRVideoStream();
  if (stream)
  {
    stream->addNewFrameListener(ir_frame_listener.get());
    stream->start();
    ir_video_started_ = true;
  }

}

void PercipioDevice::startColorStream()
{
  boost::shared_ptr<percipio::VideoStream> stream = getColorVideoStream();
  if (stream)
  {
    stream->addNewFrameListener(color_frame_listener.get());
    stream->start();
    color_video_started_ = true;
  }
}
void PercipioDevice::startDepthStream()
{
  boost::shared_ptr<percipio::VideoStream> stream = getDepthVideoStream();
  if (stream)
  {
    stream->addNewFrameListener(depth_frame_listener.get());
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
  boost::shared_ptr<percipio::VideoStream> stream = getPoint3DVideoStream();
  if(stream)
  {
    stream->addNewFrameListener(point3d_frame_listener.get());
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
    ir_video_stream_->stop();
    //ir_video_stream_->destroy();
  }
}

void PercipioDevice::stopColorStream()
{
  if (color_video_stream_)
  {
    color_video_started_ = false;
    color_video_stream_->stop();
    //color_video_stream_->destroy();
  }
}

void PercipioDevice::stopDepthStream()
{
  if (depth_video_stream_)
  {
    depth_video_started_ = false;
    depth_video_stream_->stop();
    //depth_video_stream_->destroy();
  }
}

void PercipioDevice::stopPoint3DStream()
{
  if(point3d_video_stream_)
  {
    point3d_video_started_ = false;
    point3d_video_stream_->stop();
    //point3d_video_stream_->destroy();
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

const std::vector<PercipioVideoMode>& PercipioDevice::getSupportedIRVideoModes() const
{
  boost::shared_ptr<percipio::VideoStream> stream = getIRVideoStream();
  ir_video_modes_.clear();

  if (stream)
  {
    const percipio::SensorInfo& sensor_info = stream->getSensorInfo();
    ir_video_modes_ = percipio_convert(sensor_info.getSupportedVideoModes());
  }
  return ir_video_modes_;
}

const std::vector<PercipioVideoMode>& PercipioDevice::getSupportedColorVideoModes() const
{
  boost::shared_ptr<percipio::VideoStream> stream = getColorVideoStream();
  color_video_modes_.clear();
  if (stream)
  {
    const percipio::SensorInfo& sensor_info = stream->getSensorInfo();
    color_video_modes_ = percipio_convert(sensor_info.getSupportedVideoModes());
  }
  return color_video_modes_;
}

const std::vector<PercipioVideoMode>& PercipioDevice::getSupportedDepthVideoModes() const
{
  boost::shared_ptr<percipio::VideoStream> stream = getDepthVideoStream();
  depth_video_modes_.clear();

  if (stream)
  {
    const percipio::SensorInfo& sensor_info = stream->getSensorInfo();
    depth_video_modes_ = percipio_convert(sensor_info.getSupportedVideoModes());
  }
  return depth_video_modes_;
}

bool PercipioDevice::isImageRegistrationModeSupported() const
{
  return percipio_device_->isImageRegistrationModeSupported(percipio::IMAGE_REGISTRATION_DEPTH_TO_COLOR);
}

void PercipioDevice::setColorUndistortion(bool enabled)
{
  percipio_device_->setColorUndistortion(enabled);
}

bool PercipioDevice::setDepthSpecFilterEn(bool en)
{
  return percipio_device_->setDepthSpecFilterEn(en);
}

bool PercipioDevice::getDepthSpecFilterEn()
{
  return percipio_device_->getDepthSpecFilterEn();
}

bool PercipioDevice::setDepthSpecFilterSpecSize(int spec_size)
{
  return percipio_device_->setDepthSpecFilterSpecSize(spec_size);
}

int PercipioDevice::getDepthSpecFilterSpecSize()
{
  return percipio_device_->getDepthSpecFilterSpecSize();
}

bool PercipioDevice::setDepthSpeckFilterDiff(int spec_diff)
{
  return percipio_device_->setDepthSpeckFilterDiff(spec_diff);
}

int PercipioDevice::getDepthSpeckFilterDiff()
{
  return percipio_device_->getDepthSpeckFilterDiff();
}

//
bool PercipioDevice::setDepthTimeDomainFilterEn(bool en)
{
  return percipio_device_->setDepthTimeDomainFilterEn(en);
}

bool PercipioDevice::getDepthTimeDomainFilterEn()
{
  return percipio_device_->getDepthTimeDomainFilterEn();
}

bool PercipioDevice::setDepthTimeDomainFilterNum(int frames)
{
  return percipio_device_->setDepthTimeDomainFilterNum(frames);
}

int  PercipioDevice::getDepthTimeDomainFilterNum()
{
  return percipio_device_->getDepthTimeDomainFilterNum();
}

void PercipioDevice::setImageRegistrationMode(bool enabled)
{
  if (isImageRegistrationModeSupported())
  {
    image_registration_activated_ = enabled;
    if (enabled)
    {
      TY_STATUS rc = percipio_device_->setImageRegistrationMode(percipio::IMAGE_REGISTRATION_DEPTH_TO_COLOR);
      if (rc != TY_STATUS_OK)
        THROW_PERCIPIO_EXCEPTION("Enabling image registration mode failed: \n%s\n", percipio::Percipio::getExtendedError(rc));
    }
    else
    {
      TY_STATUS rc = percipio_device_->setImageRegistrationMode(percipio::IMAGE_REGISTRATION_OFF);
      if (rc != TY_STATUS_OK)
        THROW_PERCIPIO_EXCEPTION("Enabling image registration mode failed: \n%s\n", percipio::Percipio::getExtendedError(rc));
    }
  } else {
    printf("Not support registration!\n");
  }
}

percipio::ImageRegistrationMode PercipioDevice::getImageRegistrationMode() const
{
  return percipio_device_->getImageRegistrationMode();
}

float PercipioDevice::getDepthScale()
{
  return percipio_device_->DevicePtr()->getDepthScaleUnit();
}

void PercipioDevice::setUseDeviceTimer(bool enable)
{
  if (ir_frame_listener)
    ir_frame_listener->setUseDeviceTimer(enable);

  if (color_frame_listener)
    color_frame_listener->setUseDeviceTimer(enable);

  if (depth_frame_listener)
    depth_frame_listener->setUseDeviceTimer(enable);
  
  if (point3d_frame_listener)
    point3d_frame_listener->setUseDeviceTimer(enable);
}

void PercipioDevice::setIRFrameCallback(boost::shared_ptr<FrameCallbackFunction>& callback)
{
  ir_frame_listener->setCallback(callback);
}

void PercipioDevice::setColorFrameCallback(boost::shared_ptr<FrameCallbackFunction>& callback)
{
  color_frame_listener->setCallback(callback);
}

void PercipioDevice::setDepthFrameCallback(boost::shared_ptr<FrameCallbackFunction>& callback)
{
  depth_frame_listener->setCallback(callback);
}

void PercipioDevice::setPoint3DFrameCallback(boost::shared_ptr<FrameCallbackFunction>& callback)
{
  point3d_frame_listener->setCallback(callback);
}

boost::shared_ptr<percipio::VideoStream> PercipioDevice::getIRVideoStream() const
{
  if(!ir_video_stream_)
  {
    if (hasIRSensor())
    {
      ir_video_stream_ = boost::make_shared<percipio::VideoStream>("IR");
      const TY_STATUS rc = ir_video_stream_->create(*percipio_device_, percipio::SENSOR_IR_LEFT);
      if (rc != TY_STATUS_OK)
        THROW_PERCIPIO_EXCEPTION("Couldn't create IR video stream: \n%s\n", percipio::Percipio::getExtendedError(rc));
    }
  }

  return ir_video_stream_;
}

boost::shared_ptr<percipio::VideoStream> PercipioDevice::getColorVideoStream() const
{
  if(!color_video_stream_)
  {
    if (hasColorSensor())
    {
      color_video_stream_ = boost::make_shared<percipio::VideoStream>("Color");

      const TY_STATUS rc = color_video_stream_->create(*percipio_device_, percipio::SENSOR_COLOR);
      if (rc != TY_STATUS_OK)
        THROW_PERCIPIO_EXCEPTION("Couldn't create color video stream: \n%s\n", percipio::Percipio::getExtendedError(rc));
    }
  }
  return color_video_stream_;
}

boost::shared_ptr<percipio::VideoStream> PercipioDevice::getDepthVideoStream() const
{
  if(!depth_video_stream_)
  {
    if (hasDepthSensor())
    {
      depth_video_stream_ = boost::make_shared<percipio::VideoStream>("Depth");
      const TY_STATUS rc = depth_video_stream_->create(*percipio_device_, percipio::SENSOR_DEPTH);
      if (rc != TY_STATUS_OK)
        THROW_PERCIPIO_EXCEPTION("Couldn't create depth video stream: \n%s\n", percipio::Percipio::getExtendedError(rc));
    }
  }
  return depth_video_stream_;
}

boost::shared_ptr<percipio::VideoStream> PercipioDevice::getPoint3DVideoStream() const
{
  if(!point3d_video_stream_)
  {
    if (hasPoint3DSensor())
    {
      point3d_video_stream_ = boost::make_shared<percipio::VideoStream>("Point3D");
      const TY_STATUS rc = point3d_video_stream_->create(*percipio_device_, percipio::SENSOR_POINT3D);
      if (rc != TY_STATUS_OK)
        THROW_PERCIPIO_EXCEPTION("Couldn't create point3d video stream: \n%s\n", percipio::Percipio::getExtendedError(rc));
    }
  }
  return point3d_video_stream_;
}

bool PercipioDevice::getDepthCalibIntristic(void* data, const size_t size)
{
  auto intr = percipio_device_->DevicePtr()->getDepthIntr();
  if(size != sizeof(intr))
    return false;

  memcpy(data, &intr, sizeof(intr));
	return true;
}

bool PercipioDevice::getColorCalibIntristic(void* data, const size_t size)
{
  auto intr = percipio_device_->DevicePtr()->getColorIntr();
  if(size != sizeof(intr))
    return false;
  
  memcpy(data, &intr, sizeof(intr));
	return true;
}

bool PercipioDevice::getColorCalibDistortion(void* data, const size_t size)
{
  auto dist = percipio_device_->DevicePtr()->getColorDist();
  if(size != sizeof(dist))
    return false;
  
  memcpy(data, &dist, sizeof(dist));
	return true;
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
    const std::vector<PercipioVideoMode>& video_modes = device.getSupportedIRVideoModes();
    std::vector<PercipioVideoMode>::const_iterator it = video_modes.begin();
    std::vector<PercipioVideoMode>::const_iterator it_end = video_modes.end();
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
    const std::vector<PercipioVideoMode>& video_modes = device.getSupportedColorVideoModes();

    std::vector<PercipioVideoMode>::const_iterator it = video_modes.begin();
    std::vector<PercipioVideoMode>::const_iterator it_end = video_modes.end();
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
    const std::vector<PercipioVideoMode>& video_modes = device.getSupportedDepthVideoModes();

    std::vector<PercipioVideoMode>::const_iterator it = video_modes.begin();
    std::vector<PercipioVideoMode>::const_iterator it_end = video_modes.end();
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
