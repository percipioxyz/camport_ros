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

namespace percipio_wrapper
{
class PercipioFrameListener;
typedef boost::function<void(sensor_msgs::ImagePtr image)> FrameCallbackFunction;
}

namespace percipio
{

class PercipioDevice
{
public:
  PercipioDevice();
  PercipioDevice(const std::string& device_URI, const bool auto_reconnect = false);
  virtual ~PercipioDevice();

  TY_STATUS open(const char* uri, const bool auto_reconnect = false);

  void close();

  const std::string getUri() const;
  const std::string getVendor() const;
  const std::string getName() const;
  uint16_t getUsbVendorId() const;
  uint16_t getUsbProductId() const;
  const std::string getStringID() const;

  const DeviceInfo& getDeviceInfo() const;

  boost::shared_ptr<PercipioDepthCam> DevicePtr();

  bool hasSensor(SensorType sensorType) const;

  bool hasIRSensor() const;
  bool hasColorSensor() const;
  bool hasDepthSensor() const;
  bool hasPoint3DSensor() const;

  bool isValid() const;

  bool ResolutionSetting(SensorType sensorType, int width, int height, const std::string& fmt);

  bool setColorResolution(int w, int h, const std::string& format);
  bool setDepthResolutuon(int w, int h, const std::string& format);

  void setStreamResendEnable(bool enabled);

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
  void setIRUndistortion(bool enabled);

  bool setDepthSpecFilterEn(bool en);
  bool getDepthSpecFilterEn();

  bool setDepthSpecFilterSpecSize(int spec_size);
  int getDepthSpecFilterSpecSize();

  bool setDepthSpeckFilterDiff(int spec_diff);
  int  getDepthSpeckFilterDiff();

  bool setDepthSpeckFilterPhySize(float phy_size);
  float  getDepthSpeckFilterPhySize();

  bool setDepthTimeDomainFilterEn(bool en);
  bool getDepthTimeDomainFilterEn();

  bool setDepthTimeDomainFilterNum(int frames);
  int  getDepthTimeDomainFilterNum();

  void setIREnhancement(IREnhanceModel model, int coeff);

  bool isImageRegistrationModeSupported(ImageRegistrationMode mode) const;
  bool isImageRegistrationModeSupported() const;
  TY_STATUS setImageRegistrationMode(ImageRegistrationMode mode);
  void setImageRegistrationMode(bool enabled);
  ImageRegistrationMode getImageRegistrationMode() const;

  const std::vector<percipio_wrapper::PercipioVideoMode>& getSupportedIRVideoModes() const;
  const std::vector<percipio_wrapper::PercipioVideoMode>& getSupportedColorVideoModes() const;
  const std::vector<percipio_wrapper::PercipioVideoMode>& getSupportedDepthVideoModes() const;

  void ir_stream_topic_enable(bool enabled);
  void color_stream_topic_enable(bool enabled);
  void depth_stream_topic_enable(bool enabled);
  void pointcloud_stream_topic_enable(bool enabled);

  void setIRFrameCallback(boost::shared_ptr<percipio_wrapper::FrameCallbackFunction>& callback);
  void setColorFrameCallback(boost::shared_ptr<percipio_wrapper::FrameCallbackFunction>& callback);
  void setDepthFrameCallback(boost::shared_ptr<percipio_wrapper::FrameCallbackFunction>& callback);
  void setPoint3DFrameCallback(boost::shared_ptr<percipio_wrapper::FrameCallbackFunction>& callback);

  void setDeviceInitCallback(boost::shared_ptr<DeviceInitCallbackFunction>& callback);
  void setDeviceEventCallback(boost::shared_ptr<DeviceEventCallbackFunction>& callback);

  void setUseDeviceTimer(bool enable);

  void setDeviceWorkMode(bool enable_rate, float frame_rate, bool trigger_mode_en);

  int sendSoftTrigger();

  void dynamic_configure(const std::string& str);

  void reset();

  float getDepthScale();
  bool getDepthCalibIntristic(void* data, const size_t size);
  bool getColorCalibIntristic(void* data, const size_t size);
  bool getColorCalibDistortion(void* data, const size_t size);

  int init_parameters_from_xml(const std::string& xml);

  TY_STATUS PropertyGet(TY_COMPONENT_ID comp, TY_FEATURE_ID feat, void* ptr, size_t size);
  TY_STATUS PropertySet(TY_COMPONENT_ID comp, TY_FEATURE_ID feat, void* ptr, size_t size);

protected:
  void shutdown();

  boost::shared_ptr<VideoStream> getIRVideoStream() const;
  boost::shared_ptr<VideoStream> getColorVideoStream() const;
  boost::shared_ptr<VideoStream> getDepthVideoStream() const;
  boost::shared_ptr<VideoStream> getPoint3DVideoStream() const;

  DeviceInfo m_deviceInfo;

  boost::shared_ptr<percipio_wrapper::PercipioFrameListener> ir_frame_listener;
  boost::shared_ptr<percipio_wrapper::PercipioFrameListener> color_frame_listener;
  boost::shared_ptr<percipio_wrapper::PercipioFrameListener> depth_frame_listener;
  boost::shared_ptr<percipio_wrapper::PercipioFrameListener> point3d_frame_listener;

  mutable boost::shared_ptr<VideoStream> ir_video_stream_;
  mutable boost::shared_ptr<VideoStream> color_video_stream_;
  mutable boost::shared_ptr<VideoStream> depth_video_stream_;
  mutable boost::shared_ptr<VideoStream> point3d_video_stream_;

  mutable std::vector<percipio_wrapper::PercipioVideoMode> ir_video_modes_;
  mutable std::vector<percipio_wrapper::PercipioVideoMode> color_video_modes_;
  mutable std::vector<percipio_wrapper::PercipioVideoMode> depth_video_modes_;

  bool ir_video_started_;
  bool color_video_started_;
  bool depth_video_started_;
  bool point3d_video_started_;

  bool image_registration_activated_;

  bool percipio_device_time_;

private:
  bool m_isOwner;
  TY_STATUS DeviceGetInfo();
  bool IPv4_verify(const char *ip);
};

std::ostream& operator << (std::ostream& stream, const PercipioDevice& device);

}

#endif /* PERCIPIO_DEVICE_H */
