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

#ifndef PERCIPIO_DEVICE_H
#define PERCIPIO_DEVICE_H

#include "percipio_camera/percipio_interface.h"

#include "percipio_camera/percipio_video_mode.h"

#include "percipio_camera/percipio_exception.h"

#include <boost/shared_ptr.hpp>
#include <boost/cstdint.hpp>
#include <boost/bind.hpp>
#include <boost/function.hpp>

#include <sensor_msgs/Image.h>

#include <string>
#include <vector>

namespace percipio
{
class Device;
class DeviceInfo;
class VideoStream;
class SensorInfo;
}

namespace percipio_wrapper
{

typedef boost::function<void(sensor_msgs::ImagePtr image)> FrameCallbackFunction;

class PercipioFrameListener;

class PercipioDevice
{
public:
  PercipioDevice(const std::string& device_URI, const bool auto_reconnect = false);
  virtual ~PercipioDevice();

  const std::string getUri() const;
  const std::string getVendor() const;
  const std::string getName() const;
  uint16_t getUsbVendorId() const;
  uint16_t getUsbProductId() const;

  const std::string getStringID() const;

  bool isValid() const;

  void setGvspResendEnable(bool enabled);

  bool hasIRSensor() const;
  bool hasColorSensor() const;
  bool hasDepthSensor() const;
  bool hasPoint3DSensor() const;

  bool setColorResolution(int w, int h);
  bool setDepthResolutuon(int w, int h);
  
  void startIRStream();
  void startColorStream();
  void startDepthStream();
  void startPoint3DStream();

  void stopAllStreams();

  void stopIRStream();
  void stopColorStream();
  void stopDepthStream();
  void stopPoint3DStream();

  bool isIRStreamStarted();
  bool isColorStreamStarted();
  bool isDepthStreamStarted();
  bool isPoint3DStreamStarted();

  void setColorUndistortion(bool enabled);

  bool setDepthSpecFilterEn(bool en);
  bool getDepthSpecFilterEn();

  bool setDepthSpecFilterSpecSize(int spec_size);
  int getDepthSpecFilterSpecSize();

  bool setDepthSpeckFilterDiff(int spec_diff);
  int getDepthSpeckFilterDiff();

  bool isImageRegistrationModeSupported() const;
  void setImageRegistrationMode(bool enabled);
  percipio::ImageRegistrationMode getImageRegistrationMode() const;

  bool isDeviceRGBDSyncSupported() const;
  void setDeviceRGBDSynchronization(bool enabled);

  const PercipioVideoMode getIRVideoMode();
  const PercipioVideoMode getColorVideoMode();
  const PercipioVideoMode getDepthVideoMode();

  const std::vector<PercipioVideoMode>& getSupportedIRVideoModes() const;
  const std::vector<PercipioVideoMode>& getSupportedColorVideoModes() const;
  const std::vector<PercipioVideoMode>& getSupportedDepthVideoModes() const;

  bool isIRVideoModeSupported(const PercipioVideoMode& video_mode) const;
  bool isColorVideoModeSupported(const PercipioVideoMode& video_mode) const;
  bool isDepthVideoModeSupported(const PercipioVideoMode& video_mode) const;

  void setIRVideoMode(const PercipioVideoMode& video_mode);
  void setColorVideoMode(const PercipioVideoMode& video_mode);
  void setDepthVideoMode(const PercipioVideoMode& video_mode);

  void setIRFrameCallback(FrameCallbackFunction callback);
  void setColorFrameCallback(FrameCallbackFunction callback);
  void setDepthFrameCallback(FrameCallbackFunction callback);
  void setPoint3DFrameCallback(FrameCallbackFunction callback);

  float getIRFocalLength (int output_y_resolution) const;
  float getColorFocalLength (int output_y_resolution) const;
  float getDepthFocalLength (int output_y_resolution) const;
  
  //TODO
  bool hasLaserPower() const;
  bool hasAutoExposure() const;
  bool hasAutoWhiteBalance() const;

  bool hasColorExposureTime() const;
  bool hasColorAnalogGain() const;
  bool hasColorRedGain() const;
  bool hasColorGreenGain() const;
  bool hasColorBlueGain() const;
  bool hasColorAecROI() const;
  bool hasColorAecTargetV() const;
  
  bool hasIrExposureTime() const;
  bool hasIrAnalogGain() const;
  bool hasIrGain() const;

  bool hasTofDepthChannel() const;
  bool hasTofDepthQuality() const;

  bool hasTofFilterThreshold() const;
  bool hasTofModulationThreshold() const;
  bool hasTofHdrRatio() const;
  bool hasTofJitterThreshold() const;

  bool hasTofDepthAntiSunlightIndex() const;
  bool hasDepthFilterSpeckMaxSize() const;
  bool hasDepthFilterSpecMaxDiff() const;
  
  void setLaserPower(int value);
  void setAutoExposure(bool enable);
  void setAutoWhiteBalance(bool enable);

  void setColorExposureTime(int value);
  void setColorAnalogGain(int value);
  void setColorRedGain(int value);
  void setColorGreenGain(int value);
  void setColorBlueGain(int value);
  void setColorAecROI(double p1_x, double p1_y, double p2_x, double p2_y);
  void setColorAecTargetV(int v);
  
  void setIrExposureTime(int value);
  void setIrAnalogGain(int value);
  void setIrGain(int value);

  void setTofDepthChannel(int value);
  void setTofDepthQuality(int value);
  void setTofFilterThreshold(int value);
  void setTofModulationThreshold(int value);
  void setTofHdrRatio(int value);
  void setTofJitterThreshold(int value);

  void setTofDepthAntiSunlightIndex(int value);
  void setTofDepthAntiInterferenceFlag(bool en);

  void setDepthFilterSpeckMaxSize(int value);
  void setDepthFilterSpecMaxDiff(int value);

  void setDepthScale(float scale);

  void setDevicePacketSize(int size);
  void setDevicePacketDelay(int microseconds);
  void setDeiveTimeSyncType(int type);
  void setDeviceNTPServerIP(std::string ip);


  void setSgbmImageChanNumber(int value);
  void setSgbmDispNumber(int value);
  void setSgbmDispOffset(int value);
  void setSgbmMatchWinHeight(int value);
  void setSgbmSemiP1(int value);
  void setSgbmSemiP2(int value);
  void setSgbmUniqueFactor(int value);
  void setSgbmUniqueAbsDiff(int value);
  void setSgbmCostParam(int value);
  void setSgbmHalfWinSizeEn(int value);
  void setSgbmMatchWinWidth(int value);
  void setSgbmMedianFilterEn(int value);
  void setSgbmLRCCheckEn(int value);
  void setSgbmLRCMaxDiff(int value);
  void setSgbmMedianFilterThresh(int value);
  void setSgbmSemiP1Scale(int value);

  /////////////////////////////////////////////  
  bool getLaserPower(int* value) const;
  bool getAutoExposure() const;
  bool getAutoWhiteBalance() const;

  bool getColorExposureTime(int* value) const;
  bool getColorAnalogGain(int* value) const;
  bool getColorRedGain(int* value) const;
  bool getColorGreenGain(int* value) const;
  bool getColorBlueGain(int* value) const;
  bool getColorAecROI(double* p1_x, double* p1_y, double* p2_x, double* p2_y);
  bool getColorAecTargetV(int* value) const;
  
  bool getIrExposureTime(int* value) const;
  bool getIrAnalogGain(int* value) const;
  bool getIrGain(int* value) const;

  bool getTofDepthChannel(int* value) const;
  bool getTofDepthQuality(int* value) const;

  bool getTofFilterThreshold(int* value) const;
  bool getTofModulationThreshold(int* value) const;
  bool getTofHdrRatio(int* value) const;
  bool getTofJitterThreshold(int* value) const;

  bool getTofDepthAntiSunlightIndex(int* value) const;
  bool getDepthFilterSpeckMaxSize(int* value) const;
  bool getDepthFilterSpecMaxDiff(int* value) const;

/*
  bool getAutoExposure() const;
  bool getAutoWhiteBalance() const;
*/
  void setUseDeviceTimer(bool enable);

  //PERCIPIO
  bool getDepthCalibIntristic(void* data, int* size);
  bool getColorCalibIntristic(void* data, int* size);
  bool getColorCalibDistortion(void* data, int* size);

protected:
  void shutdown();

  boost::shared_ptr<percipio::VideoStream> getIRVideoStream() const;
  boost::shared_ptr<percipio::VideoStream> getColorVideoStream() const;
  boost::shared_ptr<percipio::VideoStream> getDepthVideoStream() const;
  boost::shared_ptr<percipio::VideoStream> getPoint3DVideoStream() const;

  percipio::CameraSettings* getPoint3DCameraSettingsPtr() const;
  percipio::CameraSettings* getDepthCameraSettingsPtr() const;
  percipio::CameraSettings* getColorCameraSettingsPtr() const;
  percipio::CameraSettings* getIRCameraSettingsPtr() const;

  boost::shared_ptr<percipio::Device> percipio_device_;
  boost::shared_ptr<percipio::DeviceInfo> device_info_;

  boost::shared_ptr<PercipioFrameListener> ir_frame_listener;
  boost::shared_ptr<PercipioFrameListener> color_frame_listener;
  boost::shared_ptr<PercipioFrameListener> depth_frame_listener;
  boost::shared_ptr<PercipioFrameListener> point3d_frame_listener;

  mutable boost::shared_ptr<percipio::VideoStream> ir_video_stream_;
  mutable boost::shared_ptr<percipio::VideoStream> color_video_stream_;
  mutable boost::shared_ptr<percipio::VideoStream> depth_video_stream_;
  mutable boost::shared_ptr<percipio::VideoStream> point3d_video_stream_;

  mutable std::vector<PercipioVideoMode> ir_video_modes_;
  mutable std::vector<PercipioVideoMode> color_video_modes_;
  mutable std::vector<PercipioVideoMode> depth_video_modes_;

  bool ir_video_started_;
  bool color_video_started_;
  bool depth_video_started_;
  bool point3d_video_started_;

  bool image_registration_activated_;

  bool percipio_device_time_;

};

std::ostream& operator << (std::ostream& stream, const PercipioDevice& device);

}

#endif /* OPENNI_DEVICE_H */
