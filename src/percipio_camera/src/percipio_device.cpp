/*
 * Copyright (c) 2013, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 *      Author: Julius Kammerl (jkammerl@willowgarage.com)
 */

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

PercipioDevice::PercipioDevice(const std::string& device_URI) :
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
    rc = percipio_device_->open(device_URI.c_str());
  }
  else
  {
    rc = percipio_device_->open(percipio::ANY_DEVICE);
  }

  if (rc != TY_STATUS_OK)
    THROW_PERCIPIO_EXCEPTION("Device open failed\n%d\n", rc);

  device_info_ = boost::make_shared<percipio::DeviceInfo>();
  *device_info_ = percipio_device_->getDeviceInfo();

  ir_frame_listener = boost::make_shared<PercipioFrameListener>();
  color_frame_listener = boost::make_shared<PercipioFrameListener>();
  depth_frame_listener = boost::make_shared<PercipioFrameListener>();

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
}

PercipioDevice::~PercipioDevice()
{
  stopAllStreams();

  if (ir_video_stream_.get() != 0)
    ir_video_stream_->removeNewFrameListener(ir_frame_listener.get());
  if(color_video_stream_.get() != 0)
    color_video_stream_->removeNewFrameListener(color_frame_listener.get());
  if(depth_video_stream_.get() != 0)
    depth_video_stream_->removeNewFrameListener(depth_frame_listener.get());
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
  return (percipio_device_.get() != 0) && percipio_device_->isValid();
}

float PercipioDevice::getIRFocalLength(int output_y_resolution) const
{
  float focal_length = 0.0f;
  boost::shared_ptr<percipio::VideoStream> stream = getIRVideoStream();

  if (stream)
  {
    focal_length = (float)output_y_resolution / (2 * tan(stream->getVerticalFieldOfView() / 2));
  }

  return focal_length;
}

float PercipioDevice::getColorFocalLength(int output_y_resolution) const
{
  float focal_length = 0.0f;
  boost::shared_ptr<percipio::VideoStream> stream = getColorVideoStream();

  if (stream)
  {
    focal_length = (float)output_y_resolution / (2 * tan(stream->getVerticalFieldOfView() / 2));
  }

  return focal_length;
}

float PercipioDevice::getDepthFocalLength(int output_y_resolution) const
{
  float focal_length = 0.0f;
  boost::shared_ptr<percipio::VideoStream> stream = getDepthVideoStream();

  if (stream)
  {
    focal_length = (float)output_y_resolution / (2 * tan(stream->getVerticalFieldOfView() / 2));
  }

  return focal_length;
}

bool PercipioDevice::isIRVideoModeSupported(const PercipioVideoMode& video_mode) const
{
  getSupportedIRVideoModes();

  bool supported = false;

  std::vector<PercipioVideoMode>::const_iterator it = ir_video_modes_.begin();
  std::vector<PercipioVideoMode>::const_iterator it_end = ir_video_modes_.end();

  while (it != it_end && !supported)
  {
    supported = (*it == video_mode);
    ++it;
  }

  return supported;
}

bool PercipioDevice::isColorVideoModeSupported(const PercipioVideoMode& video_mode) const
{
  getSupportedColorVideoModes();

  bool supported = false;

  std::vector<PercipioVideoMode>::const_iterator it = color_video_modes_.begin();
  std::vector<PercipioVideoMode>::const_iterator it_end = color_video_modes_.end();

  while (it != it_end && !supported)
  {
    supported = (*it == video_mode);
    ++it;
  }

  return supported;
}

bool PercipioDevice::isDepthVideoModeSupported(const PercipioVideoMode& video_mode) const
{
  getSupportedDepthVideoModes();

  bool supported = false;

  std::vector<PercipioVideoMode>::const_iterator it = depth_video_modes_.begin();
  std::vector<PercipioVideoMode>::const_iterator it_end = depth_video_modes_.end();

  while (it != it_end && !supported)
  {
    supported = (*it == video_mode);
    ++it;
  }

  return supported;

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


bool PercipioDevice::setColorResolution(int w, int h)
{
  if(percipio_device_->hasSensor(percipio::SENSOR_COLOR))
  {
    return percipio_device_->ReslotionSetting(percipio::SENSOR_COLOR, w, h);
  }
  else
    return false;
}

bool PercipioDevice::setDepthResolutuon(int w, int h)
{
  if(percipio_device_->hasSensor(percipio::SENSOR_DEPTH))
  {
    return percipio_device_->ReslotionSetting(percipio::SENSOR_DEPTH, w, h);
  }
  else
    return false;
}

void PercipioDevice::startIRStream()
{
  boost::shared_ptr<percipio::VideoStream> stream = getIRVideoStream();

  if (stream)
  {
    stream->start();
    ir_video_started_ = true;
  }

}

void PercipioDevice::startColorStream()
{
  boost::shared_ptr<percipio::VideoStream> stream = getColorVideoStream();

  if (stream)
  {
    stream->start();
    color_video_started_ = true;
  }
}
void PercipioDevice::startDepthStream()
{
  boost::shared_ptr<percipio::VideoStream> stream = getDepthVideoStream();

  if (stream)
  {
    stream->start();
    depth_video_started_ = true;
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
}

void PercipioDevice::stopIRStream()
{
  if (ir_video_stream_.get() != 0)
  {
    ir_video_started_ = false;
    ir_video_stream_->stop();
  }
}

void PercipioDevice::stopColorStream()
{
  if (color_video_stream_.get() != 0)
  {
    color_video_started_ = false;
    color_video_stream_->stop();
  }
}

void PercipioDevice::stopDepthStream()
{
  if (depth_video_stream_.get() != 0)
  {
    depth_video_started_ = false;
    depth_video_stream_->stop();
  }
}

void PercipioDevice::shutdown()
{
  if (ir_video_stream_.get() != 0)
    ir_video_stream_->destroy();

  if (color_video_stream_.get() != 0)
    color_video_stream_->destroy();

  if (depth_video_stream_.get() != 0)
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

void PercipioDevice::setGvspResendEnable(bool enabled)
{
  percipio_device_->setStreamResendEnable(enabled);
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

bool PercipioDevice::isDeviceRGBDSyncSupported() const
{
  return percipio_device_->isDepthColorSyncSupport();
}

void PercipioDevice::setDeviceRGBDSynchronization(bool enabled)
{
  TY_STATUS rc = percipio_device_->setDepthColorSyncEnabled(enabled);
  if (rc != TY_STATUS_OK)
    THROW_PERCIPIO_EXCEPTION("Enabling depth color synchronization failed: \n%s\n", percipio::Percipio::getExtendedError(rc));
}


const PercipioVideoMode PercipioDevice::getIRVideoMode()
{
  PercipioVideoMode ret;

  boost::shared_ptr<percipio::VideoStream> stream = getIRVideoStream();

  if (stream)
  {
    percipio::VideoMode video_mode = stream->getVideoMode();

    ret = percipio_convert(video_mode);
  }
  else
    THROW_PERCIPIO_EXCEPTION("Could not create video stream.");

  return ret;
}

const PercipioVideoMode PercipioDevice::getColorVideoMode()
{
  PercipioVideoMode ret;

  boost::shared_ptr<percipio::VideoStream> stream = getColorVideoStream();

  if (stream)
  {
    percipio::VideoMode video_mode = stream->getVideoMode();

    ret = percipio_convert(video_mode);
  }
  else
  {
    PercipioVideoMode err_mode;
    err_mode.x_resolution_ = 0;
    err_mode.y_resolution_ = 0;
    ret = err_mode;
    //THROW_PERCIPIO_EXCEPTION("Could not create video stream.");
  }

  return ret;
}

const PercipioVideoMode PercipioDevice::getDepthVideoMode()
{
  PercipioVideoMode ret;

  boost::shared_ptr<percipio::VideoStream> stream = getDepthVideoStream();

  if (stream)
  {
    percipio::VideoMode video_mode = stream->getVideoMode();

    ret = percipio_convert(video_mode);
  }
  else
    THROW_PERCIPIO_EXCEPTION("Could not create video stream.");

  return ret;
}

void PercipioDevice::setIRVideoMode(const PercipioVideoMode& video_mode)
{
  boost::shared_ptr<percipio::VideoStream> stream = getIRVideoStream();

  if (stream)
  {
    const percipio::VideoMode videoMode = percipio_convert(video_mode);
    const TY_STATUS rc = stream->setVideoMode(videoMode);
    if (rc != TY_STATUS_OK)
      THROW_PERCIPIO_EXCEPTION("Couldn't set IR video mode: \n%s\n", percipio::Percipio::getExtendedError(rc));
  }
}

void PercipioDevice::setColorVideoMode(const PercipioVideoMode& video_mode)
{
  boost::shared_ptr<percipio::VideoStream> stream = getColorVideoStream();

  if (stream)
  {
    const percipio::VideoMode videoMode = percipio_convert(video_mode);
    const TY_STATUS rc = stream->setVideoMode(videoMode);
    if (rc != TY_STATUS_OK)
      THROW_PERCIPIO_EXCEPTION("Couldn't set color video mode: \n%s\n", percipio::Percipio::getExtendedError(rc));
  }
}

void PercipioDevice::setDepthVideoMode(const PercipioVideoMode& video_mode)
{
  boost::shared_ptr<percipio::VideoStream> stream = getDepthVideoStream();

  if (stream)
  {
    const percipio::VideoMode videoMode = percipio_convert(video_mode);
    const TY_STATUS rc = stream->setVideoMode(videoMode);
    if (rc != TY_STATUS_OK)
      THROW_PERCIPIO_EXCEPTION("Couldn't set depth video mode: \n%s\n", percipio::Percipio::getExtendedError(rc));
  }
}

percipio::CameraSettings* PercipioDevice::getDepthCameraSettingsPtr() const
{
  boost::shared_ptr<percipio::VideoStream> stream = getDepthVideoStream();
  if(stream) 
    return stream->getCameraSettings();
  else
    return NULL;
}

percipio::CameraSettings* PercipioDevice::getColorCameraSettingsPtr() const
{
  boost::shared_ptr<percipio::VideoStream> stream = getColorVideoStream();
  if(stream) 
    return stream->getCameraSettings();
  else
    return NULL;
}

percipio::CameraSettings* PercipioDevice::getIRCameraSettingsPtr() const
{
  boost::shared_ptr<percipio::VideoStream> stream = getIRVideoStream();
  if(stream) 
    return stream->getCameraSettings();
  else
    return NULL;
}

////////////////////////////////////////////////////////////////////////////////////////
bool PercipioDevice::hasLaserPower() const
{
  percipio::CameraSettings* camera_seeting = getDepthCameraSettingsPtr();
  if (camera_seeting) {
    int power = 0;
    return camera_seeting->getLaserPower(&power);
  }
  return false;
}

bool PercipioDevice::hasAutoExposure() const
{
  percipio::CameraSettings* camera_seeting = getColorCameraSettingsPtr();
  if (camera_seeting) {
    bool enable = false;
    return camera_seeting->getAutoExposureEnabled(&enable);
  }
  return false;
}

bool PercipioDevice::hasAutoWhiteBalance() const
{
  percipio::CameraSettings* camera_seeting = getColorCameraSettingsPtr();
  if (camera_seeting) {
    bool enable = false;
    return camera_seeting->getAutoWhiteBalanceEnabled(&enable);
  }
  return false;
}

bool PercipioDevice::hasColorAnalogGain() const
{
  percipio::CameraSettings* camera_seeting = getColorCameraSettingsPtr();
  if (camera_seeting) {
    int analog_gain = 0;
    return camera_seeting->getAnalogGain(&analog_gain);
  }
  return false;
}

bool PercipioDevice::hasColorRedGain() const
{
  percipio::CameraSettings* camera_seeting = getColorCameraSettingsPtr();
  if (camera_seeting) {
    int rgb_r_gain = 0;
    return camera_seeting->getRedGain(&rgb_r_gain);
  }
  return false;
}

bool PercipioDevice::hasColorGreenGain() const
{
  percipio::CameraSettings* camera_seeting = getColorCameraSettingsPtr();
  if (camera_seeting) {
    int rgb_g_gain = 0;
    return camera_seeting->getGreenGain(&rgb_g_gain);
  }
  return false;
}

bool PercipioDevice::hasColorBlueGain() const
{
  percipio::CameraSettings* camera_seeting = getColorCameraSettingsPtr();
  if (camera_seeting) {
    int rgb_b_gain = 0;
    return camera_seeting->getBlueGain(&rgb_b_gain);
  }
  return false;
}

bool PercipioDevice::hasColorAecROI() const
{
  percipio::CameraSettings* camera_seeting = getColorCameraSettingsPtr();
  if(camera_seeting) {
    double roi[4];
    return camera_seeting->getColorAecROI(roi);
  }
  return false;
}

bool PercipioDevice::hasColorExposureTime() const
{
  percipio::CameraSettings* camera_seeting = getColorCameraSettingsPtr();
  if (camera_seeting) {
    int rgb_exp = 0;
    return camera_seeting->getExposure(&rgb_exp);
  }
  return false;
}
bool PercipioDevice::hasIrExposureTime() const
{
  percipio::CameraSettings* camera_seeting = getIRCameraSettingsPtr();
  if (camera_seeting) {
    int ir_exposure = 0;
    return camera_seeting->getExposure(&ir_exposure);
  }
  return false;
}

bool PercipioDevice::hasIrAnalogGain() const
{
  percipio::CameraSettings* camera_seeting = getIRCameraSettingsPtr();
  if (camera_seeting) {
    int ir_analog_gain = 0;
    return camera_seeting->getAnalogGain(&ir_analog_gain);
  }
  return false;
}

bool PercipioDevice::hasIrGain() const
{
  percipio::CameraSettings* camera_seeting = getIRCameraSettingsPtr();
  if (camera_seeting) {
    int ir_gain = 0;
    return camera_seeting->getGain(&ir_gain);
  }
  return false;
}

bool PercipioDevice::hasTofDepthChannel() const
{
  percipio::CameraSettings* camera_seeting = getDepthCameraSettingsPtr();
  if (camera_seeting) {
    int depth_chan = 0;
    return camera_seeting->getTOFCamDepthChannel(&depth_chan);
  }
  return false;
}

bool PercipioDevice::hasTofDepthQuality() const
{
  percipio::CameraSettings* camera_seeting = getDepthCameraSettingsPtr();
  if (camera_seeting) {
    int depth_qua = 0;
    return camera_seeting->getTOFCamDepthQuality(&depth_qua);
  }
  return false;
}

bool PercipioDevice::hasTofFilterThreshold() const
{
  percipio::CameraSettings* camera_seeting = getDepthCameraSettingsPtr();
  if (camera_seeting) {
    int threshold = 0;
    return camera_seeting->getFilterThreshold(&threshold);
  }
  return false;
}

bool PercipioDevice::hasTofModulationThreshold() const
{
  percipio::CameraSettings* camera_seeting = getDepthCameraSettingsPtr();
  if (camera_seeting) {
    int modulation_threshold = 0;
    return camera_seeting->getModulationThreshold(&modulation_threshold);
  }
  return false;
}

bool PercipioDevice::hasTofHdrRatio() const
{
  percipio::CameraSettings* camera_seeting = getDepthCameraSettingsPtr();
  if (camera_seeting) {
    int hdr_ratio = 0;
    return camera_seeting->getTofHdrRatio(&hdr_ratio);
  }
  return false;
}

bool PercipioDevice::hasTofJitterThreshold() const
{
  percipio::CameraSettings* camera_seeting = getDepthCameraSettingsPtr();
  if (camera_seeting) {
    int jitter_threshold = 0;
    return camera_seeting->getTofJitterThreshold(&jitter_threshold);
  }
  return false;
}

bool PercipioDevice::hasTofDepthAntiSunlightIndex() const
{
  percipio::CameraSettings* camera_seeting = getDepthCameraSettingsPtr();
  if (camera_seeting) {
    int index = 0;
    return camera_seeting->getTofAntiSunlightIndex(&index);
  }
  return false;
}

bool PercipioDevice::hasDepthFilterSpeckMaxSize() const
{
  percipio::CameraSettings* camera_seeting = getDepthCameraSettingsPtr();
  if (camera_seeting) {
    int speck_size = 0;
    return camera_seeting->getFilterSpeckMaxSize(&speck_size);
  }
  return false;
}

bool PercipioDevice::hasDepthFilterSpecMaxDiff() const
{
  percipio::CameraSettings* camera_seeting = getDepthCameraSettingsPtr();
  if (camera_seeting) {
    int diff = 0;
    return camera_seeting->getFilterSpecMaxDiff(&diff);
  }
  return false;
}

void PercipioDevice::setLaserPower(int power)
{
  percipio::CameraSettings* depth_camera_seeting = getDepthCameraSettingsPtr();
  percipio::CameraSettings* ir_camera_seeting = getIRCameraSettingsPtr();
  if (depth_camera_seeting) {
    const TY_STATUS rc =  depth_camera_seeting->setLaserPower(power);
    if (rc != TY_STATUS_OK)
      ROS_WARN("Couldn't set laser power: \n%s\n", percipio::Percipio::getExtendedError(rc));
  } else if(ir_camera_seeting) {
    const TY_STATUS rc =  ir_camera_seeting->setLaserPower(power);
    if (rc != TY_STATUS_OK)
      ROS_WARN("Couldn't set laser power: \n%s\n", percipio::Percipio::getExtendedError(rc));
  } else {
    ;
  }
}

void PercipioDevice::setAutoExposure(bool enable)
{
  percipio::CameraSettings* camera_seeting = getColorCameraSettingsPtr();
  if (camera_seeting)
  {
    const TY_STATUS rc = camera_seeting->setAutoExposureEnabled(enable);
    if (rc != TY_STATUS_OK)
      ROS_WARN("Couldn't set auto exposure: \n%s\n", percipio::Percipio::getExtendedError(rc));
  }
}
void PercipioDevice::setAutoWhiteBalance(bool enable)
{
  percipio::CameraSettings* camera_seeting = getColorCameraSettingsPtr();
  if (camera_seeting)
  {
    const TY_STATUS rc = camera_seeting->setAutoWhiteBalanceEnabled(enable);
    if (rc != TY_STATUS_OK)
      ROS_WARN("Couldn't set auto white balance: \n%s\n", percipio::Percipio::getExtendedError(rc));
  }
}

void PercipioDevice::setColorAnalogGain(int value)
{
  percipio::CameraSettings* camera_seeting = getColorCameraSettingsPtr();
  if (camera_seeting)
  {
    const TY_STATUS rc = camera_seeting->setPixelsAnalogGain(value);
    if (rc != TY_STATUS_OK)
      ROS_WARN("Couldn't set pixels analog gain: \n%s\n", percipio::Percipio::getExtendedError(rc));
  }
}

void PercipioDevice::setColorRedGain(int value)
{
  percipio::CameraSettings* camera_seeting = getColorCameraSettingsPtr();
  if (camera_seeting)
  {
    const TY_STATUS rc = camera_seeting->setPixelsRedGain(value);
    if (rc != TY_STATUS_OK)
      ROS_WARN("Couldn't set pixels red gain: \n%s\n", percipio::Percipio::getExtendedError(rc));
  }
}

void PercipioDevice::setColorGreenGain(int value)
{
  percipio::CameraSettings* camera_seeting = getColorCameraSettingsPtr();
  if (camera_seeting)
  {
    const TY_STATUS rc = camera_seeting->setPixelsGreenGain(value);
    if (rc != TY_STATUS_OK)
      ROS_WARN("Couldn't set pixels green gain: \n%s\n", percipio::Percipio::getExtendedError(rc));
  }
}

void PercipioDevice::setColorBlueGain(int value)
{
  percipio::CameraSettings* camera_seeting = getColorCameraSettingsPtr();
  if (camera_seeting)
  {
    const TY_STATUS rc = camera_seeting->setPixelsBlueGain(value);
    if (rc != TY_STATUS_OK)
      ROS_WARN("Couldn't set pixels blue gain: \n%s\n", percipio::Percipio::getExtendedError(rc));
  }
}

void PercipioDevice::setColorAecROI(double p1_x, double p1_y, double p2_x, double p2_y)
{
  percipio::CameraSettings* camera_seeting = getColorCameraSettingsPtr();
  if (camera_seeting)
  {
    const TY_STATUS rc = camera_seeting->setColorAecROI(p1_x, p1_y, p2_x, p2_y);
    if (rc != TY_STATUS_OK)
      ROS_WARN("Couldn't set aec roi: \n%s\n", percipio::Percipio::getExtendedError(rc));
  }
}

void PercipioDevice::setColorExposureTime(int value)
{
  percipio::CameraSettings* camera_seeting = getColorCameraSettingsPtr();
  if (camera_seeting)
  {
    const TY_STATUS rc = camera_seeting->setExposure(value);
    if (rc != TY_STATUS_OK)
      ROS_WARN("Couldn't set exposure time: \n%s\n", percipio::Percipio::getExtendedError(rc));
  }
}

void PercipioDevice::setIrExposureTime(int value)
{
  percipio::CameraSettings* camera_seeting = getIRCameraSettingsPtr();
  if (camera_seeting)
  {
    const TY_STATUS rc = camera_seeting->setExposure(value);
    if (rc != TY_STATUS_OK)
      ROS_WARN("Couldn't set ir exposure time: \n%s\n", percipio::Percipio::getExtendedError(rc));
  }
}

void PercipioDevice::setIrAnalogGain(int value)
{
  percipio::CameraSettings* camera_seeting = getIRCameraSettingsPtr();
  if (camera_seeting)
  {
    const TY_STATUS rc = camera_seeting->setPixelsAnalogGain(value);
    if (rc != TY_STATUS_OK)
      ROS_WARN("Couldn't set ir gain: \n%s\n", percipio::Percipio::getExtendedError(rc));
  }
}

void PercipioDevice::setIrGain(int value)
{
  percipio::CameraSettings* camera_seeting = getIRCameraSettingsPtr();
  if (camera_seeting)
  {
    const TY_STATUS rc = camera_seeting->setGain(value);
    if (rc != TY_STATUS_OK)
      ROS_WARN("Couldn't set ir gain: \n%s\n", percipio::Percipio::getExtendedError(rc));
  }
}

void PercipioDevice::setTofDepthChannel(int value)
{
  percipio::CameraSettings* camera_seeting = getDepthCameraSettingsPtr();
  if (camera_seeting)
  {
    const TY_STATUS rc = camera_seeting->setTOFCamDepthChannel(value);
    if (rc != TY_STATUS_OK)
      ROS_WARN("Couldn't set tof depth channel: \n%s\n", percipio::Percipio::getExtendedError(rc));
  }
}

void PercipioDevice::setTofDepthQuality(int value)
{
  percipio::CameraSettings* camera_seeting = getDepthCameraSettingsPtr();
  if (camera_seeting)
  {
    const TY_STATUS rc = camera_seeting->setTOFCamDepthQuality(value);
    if (rc != TY_STATUS_OK)
      ROS_WARN("Couldn't set depth quality: \n%s\n", percipio::Percipio::getExtendedError(rc));
  }
}

void PercipioDevice::setTofFilterThreshold(int value)
{
  percipio::CameraSettings* camera_seeting = getDepthCameraSettingsPtr();
  if (camera_seeting)
  {
    const TY_STATUS rc = camera_seeting->setFilterThreshold(value);
    if (rc != TY_STATUS_OK)
      ROS_WARN("Couldn't set depth filter threshold: \n%s\n", percipio::Percipio::getExtendedError(rc));
  }
}

void PercipioDevice::setTofModulationThreshold(int value)
{
  percipio::CameraSettings* camera_seeting = getDepthCameraSettingsPtr();
  if (camera_seeting)
  {
    const TY_STATUS rc = camera_seeting->setModulationThreshold(value);
    if (rc != TY_STATUS_OK)
      ROS_WARN("Couldn't set depth modulation threshold: \n%s\n", percipio::Percipio::getExtendedError(rc));
  }
}

void PercipioDevice::setTofHdrRatio(int value)
{
  percipio::CameraSettings* camera_seeting = getDepthCameraSettingsPtr();
  if (camera_seeting)
  {
    const TY_STATUS rc = camera_seeting->setTofHdrRatio(value);
    if (rc != TY_STATUS_OK)
      ROS_WARN("Couldn't set depth hdr ratio: \n%s\n", percipio::Percipio::getExtendedError(rc));
  }
}

void PercipioDevice::setTofJitterThreshold(int value)
{
  percipio::CameraSettings* camera_seeting = getDepthCameraSettingsPtr();
  if (camera_seeting)
  {
    const TY_STATUS rc = camera_seeting->setTofJitterThreshold(value);
    if (rc != TY_STATUS_OK)
      ROS_WARN("Couldn't set depth jitter threshold: \n%s\n", percipio::Percipio::getExtendedError(rc));
  }
}

void PercipioDevice::setTofDepthAntiSunlightIndex(int value)
{
  percipio::CameraSettings* camera_seeting = getDepthCameraSettingsPtr();
  if (camera_seeting)
  {
    const TY_STATUS rc = camera_seeting->setTofAntiSunlightIndex(value);
    if (rc != TY_STATUS_OK)
      ROS_WARN("Couldn't set anti-sunlight index: \n%s\n", percipio::Percipio::getExtendedError(rc));
  }
}

void PercipioDevice::setDepthFilterSpeckMaxSize(int value)
{
  percipio::CameraSettings* camera_seeting = getDepthCameraSettingsPtr();
  if (camera_seeting)
  {
    const TY_STATUS rc = camera_seeting->setFilterSpeckMaxSize(value);
    if (rc != TY_STATUS_OK)
      ROS_WARN("Couldn't set max size of speckle: \n%s\n", percipio::Percipio::getExtendedError(rc));
  }
}

void PercipioDevice::setDepthFilterSpecMaxDiff(int value)
{
  percipio::CameraSettings* camera_seeting = getDepthCameraSettingsPtr();
  if (camera_seeting)
  {
    const TY_STATUS rc = camera_seeting->setFilterSpecMaxDiff(value);
    if (rc != TY_STATUS_OK)
      ROS_WARN("Couldn't set max diff of speckle: \n%s\n", percipio::Percipio::getExtendedError(rc));
  }
}

void PercipioDevice::setDepthScale(float scale)
{
  percipio::CameraSettings* camera_seeting = getDepthCameraSettingsPtr();
  if (camera_seeting)
  {
    camera_seeting->setDepthScaleValue(scale);
  }
}

/*sgbm paramters setting*/
void PercipioDevice::setSgbmImageChanNumber(int value)
{
  percipio::CameraSettings* camera_seeting = getDepthCameraSettingsPtr();
  if (camera_seeting)
  {
    const TY_STATUS rc = camera_seeting->setDepthSgbmImageChanNumber(value);
    if (rc != TY_STATUS_OK)
      ROS_WARN("Couldn't set sgbm image channel number: \n%s\n", percipio::Percipio::getExtendedError(rc));
  }
}
void PercipioDevice::setSgbmDispNumber(int value)
{
  percipio::CameraSettings* camera_seeting = getDepthCameraSettingsPtr();
  if (camera_seeting)
  {
    const TY_STATUS rc = camera_seeting->setDepthSgbmDispNumber(value);
    if (rc != TY_STATUS_OK)
      ROS_WARN("Couldn't set sgbm disp number: \n%s\n", percipio::Percipio::getExtendedError(rc));
  }
}
void PercipioDevice::setSgbmDispOffset(int value)
{
  percipio::CameraSettings* camera_seeting = getDepthCameraSettingsPtr();
  if (camera_seeting)
  {
    const TY_STATUS rc = camera_seeting->setDepthSgbmDispOffset(value);
    if (rc != TY_STATUS_OK)
      ROS_WARN("Couldn't set sgbm disp offset: \n%s\n", percipio::Percipio::getExtendedError(rc));
  }
}
void PercipioDevice::setSgbmMatchWinHeight(int value)
{
  percipio::CameraSettings* camera_seeting = getDepthCameraSettingsPtr();
  if (camera_seeting)
  {
    const TY_STATUS rc = camera_seeting->setDepthSgbmMatchWinHeight(value);
    if (rc != TY_STATUS_OK)
      ROS_WARN("Couldn't set sgbm match window height: \n%s\n", percipio::Percipio::getExtendedError(rc));
  }
}
void PercipioDevice::setSgbmSemiP1(int value)
{
  percipio::CameraSettings* camera_seeting = getDepthCameraSettingsPtr();
  if (camera_seeting)
  {
    const TY_STATUS rc = camera_seeting->setDepthSgbmSemiP1(value);
    if (rc != TY_STATUS_OK)
      ROS_WARN("Couldn't set semi global param p1: \n%s\n", percipio::Percipio::getExtendedError(rc));
  }
}
void PercipioDevice::setSgbmSemiP2(int value)
{
  percipio::CameraSettings* camera_seeting = getDepthCameraSettingsPtr();
  if (camera_seeting)
  {
    const TY_STATUS rc = camera_seeting->setDepthSgbmSemiP2(value);
    if (rc != TY_STATUS_OK)
      ROS_WARN("Couldn't set sgbm semi global param p2: \n%s\n", percipio::Percipio::getExtendedError(rc));
  }
}
void PercipioDevice::setSgbmUniqueFactor(int value)
{
  percipio::CameraSettings* camera_seeting = getDepthCameraSettingsPtr();
  if (camera_seeting)
  {
    const TY_STATUS rc = camera_seeting->setDepthSgbmUniqueFactor(value);
    if (rc != TY_STATUS_OK)
      ROS_WARN("Couldn't set sgbm uniqueness factor param: \n%s\n", percipio::Percipio::getExtendedError(rc));
  }
}
void PercipioDevice::setSgbmUniqueAbsDiff(int value)
{
  percipio::CameraSettings* camera_seeting = getDepthCameraSettingsPtr();
  if (camera_seeting)
  {
    const TY_STATUS rc = camera_seeting->setDepthSgbmUniqueAbsDiff(value);
    if (rc != TY_STATUS_OK)
      ROS_WARN("Couldn't set sgbm uniqueness min absolute diff: \n%s\n", percipio::Percipio::getExtendedError(rc));
  }
}
void PercipioDevice::setSgbmCostParam(int value)
{
  percipio::CameraSettings* camera_seeting = getDepthCameraSettingsPtr();
  if (camera_seeting)
  {
    const TY_STATUS rc = camera_seeting->setDepthSgbmCostParam(value);
    if (rc != TY_STATUS_OK)
      ROS_WARN("Couldn't set sgbm cost param: \n%s\n", percipio::Percipio::getExtendedError(rc));
  }
}
void PercipioDevice::setSgbmHalfWinSizeEn(int value)
{
  percipio::CameraSettings* camera_seeting = getDepthCameraSettingsPtr();
  if (camera_seeting)
  {
    const TY_STATUS rc = camera_seeting->setDepthSgbmHalfWinSizeEn(value);
    if (rc != TY_STATUS_OK)
      ROS_WARN("Couldn't enable sgbm half window size: \n%s\n", percipio::Percipio::getExtendedError(rc));
  }
}
void PercipioDevice::setSgbmMatchWinWidth(int value)
{
  percipio::CameraSettings* camera_seeting = getDepthCameraSettingsPtr();
  if (camera_seeting)
  {
    const TY_STATUS rc = camera_seeting->setDepthSgbmMatchWinWidth(value);
    if (rc != TY_STATUS_OK)
      ROS_WARN("Couldn't set sgbm match window width: \n%s\n", percipio::Percipio::getExtendedError(rc));
  }
}
void PercipioDevice::setSgbmMedianFilterEn(int value)
{
  percipio::CameraSettings* camera_seeting = getDepthCameraSettingsPtr();
  if (camera_seeting)
  {
    const TY_STATUS rc = camera_seeting->setDepthSgbmMedianFilterEn(value);
    if (rc != TY_STATUS_OK)
      ROS_WARN("Couldn't enable sgbm median filter: \n%s\n", percipio::Percipio::getExtendedError(rc));
  }
}
void PercipioDevice::setSgbmLRCCheckEn(int value)
{
  percipio::CameraSettings* camera_seeting = getDepthCameraSettingsPtr();
  if (camera_seeting)
  {
    const TY_STATUS rc = camera_seeting->setDepthSgbmLRCCheckEn(value);
    if (rc != TY_STATUS_OK)
      ROS_WARN("Couldn't enable sgbm left right consist check: \n%s\n", percipio::Percipio::getExtendedError(rc));
  }
}
void PercipioDevice::setSgbmLRCMaxDiff(int value)
{
  percipio::CameraSettings* camera_seeting = getDepthCameraSettingsPtr();
  if (camera_seeting)
  {
    const TY_STATUS rc = camera_seeting->setDepthSgbmLRCMaxDiff(value);
    if (rc != TY_STATUS_OK)
      ROS_WARN("Couldn't set sgbm max diff: \n%s\n", percipio::Percipio::getExtendedError(rc));
  }
}
void PercipioDevice::setSgbmMedianFilterThresh(int value)
{
  percipio::CameraSettings* camera_seeting = getDepthCameraSettingsPtr();
  if (camera_seeting)
  {
    const TY_STATUS rc = camera_seeting->setDepthSgbmMedianFilterThresh(value);
    if (rc != TY_STATUS_OK)
      ROS_WARN("Couldn't set sgbm median filter thresh: \n%s\n", percipio::Percipio::getExtendedError(rc));
  }
}
void PercipioDevice::setSgbmSemiP1Scale(int value)
{
  percipio::CameraSettings* camera_seeting = getDepthCameraSettingsPtr();
  if (camera_seeting)
  {
    const TY_STATUS rc = camera_seeting->setDepthSgbmSemiP1Scale(value);
    if (rc != TY_STATUS_OK)
      ROS_WARN("Couldn't set sgbm semi global param p1 scale: \n%s\n", percipio::Percipio::getExtendedError(rc));
  }
}

bool PercipioDevice::getLaserPower(int* value) const
{
  percipio::CameraSettings* depth_camera_seeting = getDepthCameraSettingsPtr();
  percipio::CameraSettings* ir_camera_seeting = getIRCameraSettingsPtr();

  if (depth_camera_seeting)
    return  depth_camera_seeting->getLaserPower(value);
  if(ir_camera_seeting)
    return ir_camera_seeting->getLaserPower(value);

  return false;
}

bool PercipioDevice::getAutoExposure() const
{
  bool enable = false;
  percipio::CameraSettings* camera_seeting = getColorCameraSettingsPtr();
  if (camera_seeting)
    camera_seeting->getAutoExposureEnabled(&enable);

  return enable;
}

bool PercipioDevice::getAutoWhiteBalance() const
{
  bool enable = false;
  percipio::CameraSettings* camera_seeting = getColorCameraSettingsPtr();
  if (camera_seeting)
    camera_seeting->getAutoWhiteBalanceEnabled(&enable);

  return enable;
}

bool PercipioDevice::getColorAnalogGain(int* value) const
{
  percipio::CameraSettings* camera_seeting = getColorCameraSettingsPtr();
  if (camera_seeting)
    return camera_seeting->getAnalogGain(value);
  else 
    return false;
}

bool PercipioDevice::getColorRedGain(int* value) const
{
  percipio::CameraSettings* camera_seeting = getColorCameraSettingsPtr();
  if (camera_seeting)
    return  camera_seeting->getRedGain(value);
  else
    return false;
}

bool PercipioDevice::getColorGreenGain(int* value) const
{
  percipio::CameraSettings* camera_seeting = getColorCameraSettingsPtr();
  if (camera_seeting)
    return camera_seeting->getGreenGain(value);
  else
    return false;
}

bool PercipioDevice::getColorBlueGain(int* value) const
{
  percipio::CameraSettings* camera_seeting = getColorCameraSettingsPtr();
  if (camera_seeting)
    return camera_seeting->getBlueGain(value);
  else
    return false;
}

bool PercipioDevice::getColorAecROI(double* p1_x, double* p1_y, double* p2_x, double* p2_y)
{
  percipio::CameraSettings* camera_seeting = getColorCameraSettingsPtr();
  if (camera_seeting) {
    double roi[4];
    bool ret = camera_seeting->getColorAecROI(roi);
    if(!ret) return false;

    *p1_x = roi[0];
    *p1_y = roi[1];
    *p2_x = roi[2];
    *p2_y = roi[3];
    return true;
  }else
    return false;
}

bool PercipioDevice::getColorExposureTime(int* value) const
{
  percipio::CameraSettings* camera_seeting = getColorCameraSettingsPtr();
  if (camera_seeting)
    return camera_seeting->getExposure(value);
  else
    return false;
}

bool PercipioDevice::getIrExposureTime(int* value) const
{
  percipio::CameraSettings* camera_seeting = getIRCameraSettingsPtr();
  if (camera_seeting)
    return camera_seeting->getExposure(value);
  else
    return false;
}

bool PercipioDevice::getIrAnalogGain(int* value) const
{
  percipio::CameraSettings* camera_seeting = getIRCameraSettingsPtr();
  if (camera_seeting)
    return camera_seeting->getAnalogGain(value);
  else
    return false;
}

bool PercipioDevice::getIrGain(int* value) const
{
  percipio::CameraSettings* camera_seeting = getIRCameraSettingsPtr();
  if (camera_seeting)
    return camera_seeting->getGain(value);
  else
    return false;
}

bool PercipioDevice::getTofDepthChannel(int* value) const
{
  percipio::CameraSettings* camera_seeting = getDepthCameraSettingsPtr();
  if (camera_seeting)
    return camera_seeting->getTOFCamDepthChannel(value);
  else
    return false;
}

bool PercipioDevice::getTofDepthQuality(int* value) const
{
  percipio::CameraSettings* camera_seeting = getDepthCameraSettingsPtr();
  if (camera_seeting)
    return camera_seeting->getTOFCamDepthQuality(value);
  else
    return false;
}

bool PercipioDevice::getTofFilterThreshold(int* value) const
{
  percipio::CameraSettings* camera_seeting = getDepthCameraSettingsPtr();
  if (camera_seeting)
    return camera_seeting->getFilterThreshold(value);
  else
    return false;
}

bool PercipioDevice::getTofModulationThreshold(int* value) const
{
  percipio::CameraSettings* camera_seeting = getDepthCameraSettingsPtr();
  if (camera_seeting)
    return camera_seeting->getModulationThreshold(value);
  else
    return false;
}

bool PercipioDevice::getTofHdrRatio(int* value) const
{
  percipio::CameraSettings* camera_seeting = getDepthCameraSettingsPtr();
  if (camera_seeting)
    return camera_seeting->getTofHdrRatio(value);
  else
    return false;
}

bool PercipioDevice::getTofJitterThreshold(int* value) const
{
  percipio::CameraSettings* camera_seeting = getDepthCameraSettingsPtr();
  if (camera_seeting)
    return camera_seeting->getTofJitterThreshold(value);
  else
    return false;
}

bool PercipioDevice::getTofDepthAntiSunlightIndex(int* value) const
{
  percipio::CameraSettings* camera_seeting = getDepthCameraSettingsPtr();
  if (camera_seeting)
    return camera_seeting->getTofAntiSunlightIndex(value);
  else
    return false;
}

bool PercipioDevice::getDepthFilterSpeckMaxSize(int* value) const
{
  percipio::CameraSettings* camera_seeting = getDepthCameraSettingsPtr();
  if (camera_seeting)
    return camera_seeting->getFilterSpeckMaxSize(value);
  else
    return false;
}

bool PercipioDevice::getDepthFilterSpecMaxDiff(int* value) const
{
  percipio::CameraSettings* camera_seeting = getDepthCameraSettingsPtr();
  if (camera_seeting)
    return camera_seeting->getFilterSpecMaxDiff(value);
  else
    return false;
}

void PercipioDevice::setUseDeviceTimer(bool enable)
{
  if (ir_frame_listener)
    ir_frame_listener->setUseDeviceTimer(enable);

  if (color_frame_listener)
    color_frame_listener->setUseDeviceTimer(enable);

  if (depth_frame_listener)
    depth_frame_listener->setUseDeviceTimer(enable);
}

void PercipioDevice::setIRFrameCallback(FrameCallbackFunction callback)
{
  ir_frame_listener->setCallback(callback);
}

void PercipioDevice::setColorFrameCallback(FrameCallbackFunction callback)
{
  color_frame_listener->setCallback(callback);
}

void PercipioDevice::setDepthFrameCallback(FrameCallbackFunction callback)
{
  depth_frame_listener->setCallback(callback);
}

boost::shared_ptr<percipio::VideoStream> PercipioDevice::getIRVideoStream() const
{
  if (ir_video_stream_.get() == 0)
  {
    if (hasIRSensor())
    {
      ir_video_stream_ = boost::make_shared<percipio::VideoStream>();
      const TY_STATUS rc = ir_video_stream_->create(*percipio_device_, percipio::SENSOR_IR_LEFT);
      if (rc != TY_STATUS_OK)
        THROW_PERCIPIO_EXCEPTION("Couldn't create IR video stream: \n%s\n", percipio::Percipio::getExtendedError(rc));
    }
  }

  return ir_video_stream_;
}

boost::shared_ptr<percipio::VideoStream> PercipioDevice::getColorVideoStream() const
{
  if (color_video_stream_.get() == 0)
  {
    if (hasColorSensor())
    {
      color_video_stream_ = boost::make_shared<percipio::VideoStream>();

      const TY_STATUS rc = color_video_stream_->create(*percipio_device_, percipio::SENSOR_COLOR);
      if (rc != TY_STATUS_OK)
        THROW_PERCIPIO_EXCEPTION("Couldn't create color video stream: \n%s\n", percipio::Percipio::getExtendedError(rc));
    }
  }
  return color_video_stream_;
}

boost::shared_ptr<percipio::VideoStream> PercipioDevice::getDepthVideoStream() const
{
  if (depth_video_stream_.get() == 0)
  {
    if (hasDepthSensor())
    {
      depth_video_stream_ = boost::make_shared<percipio::VideoStream>();

      const TY_STATUS rc = depth_video_stream_->create(*percipio_device_, percipio::SENSOR_DEPTH);
      if (rc != TY_STATUS_OK)
        THROW_PERCIPIO_EXCEPTION("Couldn't create depth video stream: \n%s\n", percipio::Percipio::getExtendedError(rc));
    }
  }
  return depth_video_stream_;
}

bool PercipioDevice::getDepthCalibIntristic(void* data, int* size)
{
	if (TY_STATUS_OK != percipio_device_->getProperty(percipio::TY_DEVICE_PROPERTY_DEPTH_CALIB_INTRISTIC, (void*)data, size))
	{
		std::cout<<"[IR]Failed to load calib intristic!"<<std::endl;
		return false;
	}
	return true;
}

bool PercipioDevice::getColorCalibIntristic(void* data, int* size)
{
  if (TY_STATUS_OK != percipio_device_->getProperty(percipio::TY_DEVICE_PROPERTY_COLOR_CALIB_INTRISTIC, (void*)data, size))
	{
		std::cout<<"[COLOR]Failed to load calib intristic!"<<std::endl;
		return false;
	}
	return true;
}

bool PercipioDevice::getColorCalibDistortion(void* data, int* size)
{
  if (TY_STATUS_OK != percipio_device_->getProperty(percipio::TY_DEVICE_PROPERTY_COLOR_CALIB_DISTORTION, (void*)data, size))
	{
		std::cout<<"[COLOR]Failed to load calib intristic!"<<std::endl;
		return false;
	}
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
