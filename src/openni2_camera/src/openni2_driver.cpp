/*
 * Copyright (c) 2013, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the name of the Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
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
 *    Author: Julius Kammerl (jkammerl@willowgarage.com)
 */

#include "openni2_camera/openni2_driver.h"
#include "openni2_camera/openni2_exception.h"

#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/distortion_models.h>

#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/thread/thread.hpp>

namespace openni2_wrapper
{

struct calibIntris{
  uint32_t width;
  uint32_t height;
  float data[9];
};

struct calibIntris ir_cal;

sensor_msgs::CameraInfoPtr info;
bool loadedIRCameraInfo = false;

sensor_msgs::CameraInfoPtr color_info;
bool loadedColorCameraInfo = false;

OpenNI2Driver::OpenNI2Driver(ros::NodeHandle& n, ros::NodeHandle& pnh) :
  nh_(n),
  pnh_(pnh),
  device_manager_(OpenNI2DeviceManager::getSingelton()),
  config_init_(false),
  data_skip_ir_counter_(0),
  data_skip_color_counter_(0),
  data_skip_depth_counter_ (0),
  ir_subscribers_(false),
  color_subscribers_(false),
  depth_subscribers_(false),
  depth_raw_subscribers_(false)
{
  genVideoModeTableMap();

  readConfigFromParameterServer();

  initDevice();

  // Initialize dynamic reconfigure
  reconfigure_server_.reset(new ReconfigureServer(pnh_));
  reconfigure_server_->setCallback(boost::bind(&OpenNI2Driver::configCb, this, _1, _2));

  while (!config_init_)
  {
    ROS_DEBUG("Waiting for dynamic reconfigure configuration.");
    boost::this_thread::sleep(boost::posix_time::milliseconds(100));
  }
  ROS_DEBUG("Dynamic reconfigure configuration received.");

  advertiseROSTopics();
}

void OpenNI2Driver::advertiseROSTopics()
{
  // Allow remapping namespaces rgb, ir, depth, depth_registered
  ros::NodeHandle color_nh(nh_, "rgb");
  image_transport::ImageTransport color_it(color_nh);
  ros::NodeHandle ir_nh(nh_, "ir");
  image_transport::ImageTransport ir_it(ir_nh);
  ros::NodeHandle depth_nh(nh_, "depth");
  image_transport::ImageTransport depth_it(depth_nh);
  ros::NodeHandle depth_raw_nh(nh_, "depth");
  image_transport::ImageTransport depth_raw_it(depth_raw_nh);
  
  // Advertise all published topics

  // Prevent connection callbacks from executing until we've set all the publishers. Otherwise
  // connectCb() can fire while we're advertising (say) "depth/image_raw", but before we actually
  // assign to pub_depth_raw_. Then pub_depth_raw_.getNumSubscribers() returns 0, and we fail to start
  // the depth generator.
  boost::lock_guard<boost::mutex> lock(connect_mutex_);

  // Asus Xtion PRO does not have an RGB camera
  if (device_->hasColorSensor())
  {
    if((color_video_mode_.x_resolution_ > 0) && (color_video_mode_.y_resolution_ > 0))
    {
      image_transport::SubscriberStatusCallback itssc = boost::bind(&OpenNI2Driver::colorConnectCb, this);
      ros::SubscriberStatusCallback rssc = boost::bind(&OpenNI2Driver::colorConnectCb, this);
      pub_color_ = color_it.advertiseCamera("image", 1, itssc, itssc, rssc, rssc);
    }
    else
      ROS_WARN("color video mode err : %d, %d.\n", color_video_mode_.x_resolution_, color_video_mode_.y_resolution_);
  }
  else
    ROS_WARN("Device do not has color sensor.\n");

  if (device_->hasIRSensor())
  {
    if((ir_video_mode_.x_resolution_ > 0) && (ir_video_mode_.y_resolution_ > 0))
    {
      image_transport::SubscriberStatusCallback itssc = boost::bind(&OpenNI2Driver::irConnectCb, this);
      ros::SubscriberStatusCallback rssc = boost::bind(&OpenNI2Driver::irConnectCb, this);
      pub_ir_ = ir_it.advertiseCamera("image", 1, itssc, itssc, rssc, rssc);
    }
  }

  if (device_->hasDepthSensor())
  {
    if((depth_video_mode_.x_resolution_ > 0) && (depth_video_mode_.y_resolution_ > 0))
    {
      image_transport::SubscriberStatusCallback itssc = boost::bind(&OpenNI2Driver::depthConnectCb, this);
      ros::SubscriberStatusCallback rssc = boost::bind(&OpenNI2Driver::depthConnectCb, this);
      pub_depth_raw_ = depth_it.advertiseCamera("image_raw", 1, itssc, itssc, rssc, rssc);
      pub_depth_ = depth_raw_it.advertiseCamera("image", 1, itssc, itssc, rssc, rssc);
    }
  }

  ////////// CAMERA INFO MANAGER

  // Pixel offset between depth and IR images.
  // By default assume offset of (5,4) from 9x7 correlation window.
  // NOTE: These are now (temporarily?) dynamically reconfigurable, to allow tweaking.
  //param_nh.param("depth_ir_offset_x", depth_ir_offset_x_, 5.0);
  //param_nh.param("depth_ir_offset_y", depth_ir_offset_y_, 4.0);

  // The camera names are set to [rgb|depth]_[serial#], e.g. depth_B00367707227042B.
  // camera_info_manager substitutes this for ${NAME} in the URL.
  std::string serial_number = device_->getStringID();
  std::string color_name, depth_name, ir_name;//, ;

  color_name = "rgb_"   + serial_number;
  depth_name  = "depth" + serial_number;
  ir_name    = "ir" + serial_number;

  // Load the saved calibrations, if they exist
  color_info_manager_ = boost::make_shared<camera_info_manager::CameraInfoManager>(color_nh, color_name, color_info_url_);
  depth_info_manager_  = boost::make_shared<camera_info_manager::CameraInfoManager>(depth_nh,  depth_name,  depth_info_url_);
  ir_info_manager     = boost::make_shared<camera_info_manager::CameraInfoManager>(ir_nh,  ir_name,  ir_info_url_);

  get_serial_server = nh_.advertiseService("get_serial", &OpenNI2Driver::getSerialCb,this);

}

bool OpenNI2Driver::getSerialCb(openni2_camera::GetSerialRequest& req, openni2_camera::GetSerialResponse& res) {
  res.serial = device_manager_->getSerial(device_->getUri());
  return true;
}

void OpenNI2Driver::configCb(Config &config, uint32_t level)
{
  bool stream_reset = false;
  if((int)level < 0) {
    if(device_) {
      device_->getLaserPower(&m_laser_power_);
      auto_exposure_ = device_->getAutoExposure();
      auto_white_balance_ = device_->getAutoWhiteBalance();
      device_->getColorAnalogGain(&m_rgb_analog_gain_);
      device_->getColorRedGain(&m_rgb_r_gain_);
      device_->getColorGreenGain(&m_rgb_g_gain_);
      device_->getColorBlueGain(&m_rgb_b_gain_);
      device_->getColorExposureTime(&m_rgb_exposure_time_);
      device_->getIrGain(&m_ir_gain_);
      //
      
      config.laser_power = m_laser_power_;
  
      config.auto_exposure = auto_exposure_;
      config.auto_white_balance = auto_white_balance_;
  
      config.rgb_analog_gain = m_rgb_analog_gain_;
      config.rgb_r_gain = m_rgb_r_gain_;
      config.rgb_g_gain = m_rgb_g_gain_;
      config.rgb_b_gain = m_rgb_b_gain_;
      config.rgb_exposure_time = m_rgb_exposure_time_;
  
      config.ir_gain = m_ir_gain_;
    }
  }
  else {
    if(m_laser_power_ != config.laser_power) {
      m_laser_power_ = config.laser_power;
      if(device_) device_->setLaserPower(m_laser_power_);
    }
  
    if(auto_exposure_ != config.auto_exposure) {
      auto_exposure_ = config.auto_exposure;
      if(device_) device_->setAutoExposure(auto_exposure_);
    }
    
    if(auto_white_balance_ != config.auto_white_balance) {
      auto_white_balance_ = config.auto_white_balance;
      if(device_) device_->setAutoWhiteBalance(auto_white_balance_);
    }
  
    if(m_rgb_analog_gain_ != config.rgb_analog_gain) {
      m_rgb_analog_gain_ = config.rgb_analog_gain;
      if(device_) device_->setColorAnalogGain(m_rgb_analog_gain_);
    }
    
    if(m_rgb_r_gain_ != config.rgb_r_gain) {
      m_rgb_r_gain_ = config.rgb_r_gain;
      if(device_) device_->setColorRedGain(m_rgb_r_gain_);
    }
    
    if(m_rgb_g_gain_ != config.rgb_g_gain) {
      m_rgb_g_gain_ = config.rgb_g_gain;
      if(device_) device_->setColorGreenGain(m_rgb_g_gain_);
    }
    
    if(m_rgb_b_gain_ != config.rgb_b_gain) {
      m_rgb_b_gain_ = config.rgb_b_gain;
      if(device_) device_->setColorBlueGain(m_rgb_b_gain_);
    }
    
    if(m_rgb_exposure_time_ != config.rgb_exposure_time) {
      m_rgb_exposure_time_ = config.rgb_exposure_time;
      if(device_) device_->setColorExposureTime(m_rgb_exposure_time_);
    }
  
    if(m_ir_gain_ != config.ir_gain) {
      m_ir_gain_ = config.ir_gain;
      if(device_) device_->setIrGain(m_ir_gain_);
    }
  }
  
  z_scaling_ = config.z_scaling;

  // assign pixel format
  //ir_video_mode_.pixel_format_ = PIXEL_FORMAT_GRAY16;
  ir_video_mode_.pixel_format_ = PIXEL_FORMAT_GRAY8;
  color_video_mode_.pixel_format_ = PIXEL_FORMAT_RGB888;
  depth_video_mode_.pixel_format_ = PIXEL_FORMAT_DEPTH_1_MM;

  //color_depth_synchronization_ = config.color_depth_synchronization;
  if(depth_registration_ != config.depth_registration){
    depth_registration_ = config.depth_registration;
    loadedIRCameraInfo = false;
  }
  auto_exposure_ = config.auto_exposure;
  auto_white_balance_ = config.auto_white_balance;

  use_device_time_ = config.use_device_time;

  data_skip_ = config.data_skip+1;

  applyConfigToOpenNIDevice();

  config_init_ = true;

  old_config_ = config;
}

OpenNI2VideoMode OpenNI2Driver::getIRVideoMode()
{
  return device_->getIRVideoMode();
}

OpenNI2VideoMode OpenNI2Driver::getColorVideoMode()
{
  return device_->getColorVideoMode();
}

OpenNI2VideoMode OpenNI2Driver::getDepthVideoMode()
{
  return device_->getDepthVideoMode();
}

void OpenNI2Driver::applyConfigToOpenNIDevice()
{
  data_skip_ir_counter_ = 0;
  data_skip_color_counter_= 0;
  data_skip_depth_counter_ = 0;
  
  ir_video_mode_ = getIRVideoMode();
  
  color_video_mode_ = getColorVideoMode();
  depth_video_mode_ = getDepthVideoMode();
  
  if (device_->isImageRegistrationModeSupported())
  {
    try
    {
      if (!config_init_ || (old_config_.depth_registration != depth_registration_))
        device_->setImageRegistrationMode(depth_registration_);
    }
    catch (const OpenNI2Exception& exception)
    {
      ROS_ERROR("Could not set image registration. Reason: %s", exception.what());
    }
  }

  device_->setUseDeviceTimer(use_device_time_);
}

void OpenNI2Driver::colorConnectCb()
{
  boost::lock_guard<boost::mutex> lock(connect_mutex_);

  color_subscribers_ = pub_color_.getNumSubscribers() > 0;

  if (color_subscribers_ && !device_->isColorStreamStarted())
  {
    // Can't stream IR and RGB at the same time. Give RGB preference.
    //if (device_->isIRStreamStarted())
    if(0)
    {
      ROS_ERROR("Cannot stream RGB and IR at the same time. Streaming RGB only.");
      ROS_INFO("Stopping IR stream.");
      device_->stopIRStream();
    }

    device_->setColorFrameCallback(boost::bind(&OpenNI2Driver::newColorFrameCallback, this, _1));

    ROS_INFO("Starting color stream.");
    device_->startColorStream();
  }
  else if (!color_subscribers_ && device_->isColorStreamStarted())
  {
#if 0  
    ROS_INFO("Stopping color stream.");
    device_->stopColorStream();

    // Start IR if it's been blocked on RGB subscribers
    bool need_ir = pub_ir_.getNumSubscribers() > 0;
    if (need_ir && !device_->isIRStreamStarted())
    {
      device_->setIRFrameCallback(boost::bind(&OpenNI2Driver::newIRFrameCallback, this, _1));

      ROS_INFO("Starting IR stream.");
      device_->startIRStream();
      ROS_INFO("Starting IR stream finished.");
    }
#else
    ROS_INFO("Stopping color stream.");
    device_->stopColorStream();
#endif    
  }
}

void OpenNI2Driver::depthConnectCb()
{
  boost::lock_guard<boost::mutex> lock(connect_mutex_);
  depth_subscribers_ = pub_depth_.getNumSubscribers() > 0;
  depth_raw_subscribers_ = pub_depth_raw_.getNumSubscribers() > 0;
  bool need_depth = depth_subscribers_ || depth_raw_subscribers_;

  if (need_depth && !device_->isDepthStreamStarted())
  {
    device_->setDepthFrameCallback(boost::bind(&OpenNI2Driver::newDepthFrameCallback, this, _1));

    ROS_INFO("Starting depth stream.");
    device_->startDepthStream();
    ROS_INFO("Starting depth stream finished.");
  }
  else if (!need_depth && device_->isDepthStreamStarted())
  {
    ROS_INFO("Stopping depth stream.");
    device_->stopDepthStream();
  }
}

void OpenNI2Driver::irConnectCb()
{
  boost::lock_guard<boost::mutex> lock(connect_mutex_);

  ir_subscribers_ = pub_ir_.getNumSubscribers() > 0;

  if (ir_subscribers_ && !device_->isIRStreamStarted())
  {
    // Can't stream IR and RGB at the same time
    //if (device_->isColorStreamStarted())
    if(0)
    {
      ROS_ERROR("Cannot stream RGB and IR at the same time. Streaming RGB only.");
    }
    else
    {
      device_->setIRFrameCallback(boost::bind(&OpenNI2Driver::newIRFrameCallback, this, _1));

      ROS_INFO("Starting IR stream.");
      device_->startIRStream();
    }
  }
  else if (!ir_subscribers_ && device_->isIRStreamStarted())
  {
    ROS_INFO("Stopping IR stream.");
    device_->stopIRStream();
  }
}

void OpenNI2Driver::newIRFrameCallback(sensor_msgs::ImagePtr image)
{
  if ((++data_skip_ir_counter_)%data_skip_==0)
  {
    data_skip_ir_counter_ = 0;

    if (ir_subscribers_)
    {
      image->header.frame_id = ir_frame_id_;
      image->header.stamp = image->header.stamp + ir_time_offset_;

      pub_ir_.publish(image, getIRCameraInfo(image->width, image->height, image->header.stamp));
    }
  }
}

void OpenNI2Driver::newColorFrameCallback(sensor_msgs::ImagePtr image)
{
  if ((++data_skip_color_counter_)%data_skip_==0)
  {
    data_skip_color_counter_ = 0;
    if (color_subscribers_)
    {
      image->header.frame_id = color_frame_id_;
      image->header.stamp = image->header.stamp + color_time_offset_;

      pub_color_.publish(image, getColorCameraInfo(image->width, image->height, image->header.stamp));
    }
  }
}

void OpenNI2Driver::newDepthFrameCallback(sensor_msgs::ImagePtr image)
{
  if ((++data_skip_depth_counter_)%data_skip_==0)
  {
    data_skip_depth_counter_ = 0;

    if (depth_raw_subscribers_||depth_subscribers_)
    {
      image->header.stamp = image->header.stamp;

      if (z_offset_mm_ != 0)
      {
        uint16_t* data = reinterpret_cast<uint16_t*>(&image->data[0]);
        for (unsigned int i = 0; i < image->width * image->height; ++i)
          if (data[i] != 0)
            data[i] += z_offset_mm_;
      }

      if (fabs(z_scaling_ - 1.0) > 1e-6)
      {
        uint16_t* data = reinterpret_cast<uint16_t*>(&image->data[0]);
        for (unsigned int i = 0; i < image->width * image->height; ++i)
          if (data[i] != 0)
        data[i] = static_cast<uint16_t>(data[i] * z_scaling_);
      }

      sensor_msgs::CameraInfoPtr cam_info;
      if (depth_registration_)
      {
        image->header.frame_id = color_frame_id_;
        cam_info = getDepthCameraInfo(image->width,image->height, image->header.stamp);
      } 
      else
      {
        image->header.frame_id = depth_frame_id_;
        cam_info = getDepthCameraInfo(image->width,image->height, image->header.stamp);
      }
      
      if (depth_raw_subscribers_)
      {
        pub_depth_raw_.publish(image, cam_info);
      }

      if (depth_subscribers_ )
      {
        sensor_msgs::ImageConstPtr floating_point_image = rawToFloatingPointConversion(image);
        pub_depth_.publish(floating_point_image, cam_info);
      }
    }
  }
}

// Methods to get calibration parameters for the various cameras
sensor_msgs::CameraInfoPtr OpenNI2Driver::getDefaultCameraInfo(int width, int height, double f) const
{
  sensor_msgs::CameraInfoPtr info = boost::make_shared<sensor_msgs::CameraInfo>();

  info->width  = width;
  info->height = height;

  // No distortion
  info->D.resize(5, 0.0);
  info->distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;

  // Simple camera matrix: square pixels (fx = fy), principal point at center
  info->K.assign(0.0);
  info->K[0] = info->K[4] = f;
  info->K[2] = (width / 2) - 0.5;
  // Aspect ratio for the camera center on Kinect (and other devices?) is 4/3
  // This formula keeps the principal point the same in VGA and SXGA modes
  info->K[5] = (width * (3./8.)) - 0.5;
  info->K[8] = 1.0;

  // No separate rectified image plane, so R = I
  info->R.assign(0.0);
  info->R[0] = info->R[4] = info->R[8] = 1.0;

  // Then P=K(I|0) = (K|0)
  info->P.assign(0.0);
  info->P[0]  = info->P[5] = f; // fx, fy
  info->P[2]  = info->K[2];   // cx
  info->P[6]  = info->K[5];   // cy
  info->P[10] = 1.0;

  return info;
}

/// @todo Use binning/ROI properly in publishing camera infos
sensor_msgs::CameraInfoPtr OpenNI2Driver::getColorCameraInfo(int width, int height, ros::Time time) const
{
  if (loadedColorCameraInfo)
  {
    // Fill in header
    color_info->header.stamp  = time;
    color_info->header.frame_id = color_frame_id_;
  
    return color_info; 
  }

  if (color_info_manager_->isCalibrated())
  {
    color_info = boost::make_shared<sensor_msgs::CameraInfo>(color_info_manager_->getCameraInfo());
    if ( info->width != width )
    {
      // Use uncalibrated values
      ROS_WARN_ONCE("Image resolution doesn't match calibration of the RGB camera. Using default parameters.");
      info = getDefaultCameraInfo(width, height, device_->getColorFocalLength(height));
    }
  }
  else
  {
    // If uncalibrated, fill in default values
    color_info = getDefaultCameraInfo(width, height, device_->getColorFocalLength(height));
  }

  // Fill in header
  color_info->header.stamp  = time;
  color_info->header.frame_id = color_frame_id_;
  
  loadedColorCameraInfo = true;

  return color_info;
}


sensor_msgs::CameraInfoPtr OpenNI2Driver::getIRCameraInfo(int width, int height, ros::Time time) const
{
  if (device_->getVendor() == "Percipio")
  {
    info = boost::make_shared<sensor_msgs::CameraInfo>();

    int size = sizeof(calibIntris);
    ir_cal.width = width;
    ir_cal.height = height;
    
    
    if (!loadedIRCameraInfo) {
      //get intristic from percipio firmware
      device_->getCalibIntristic((void*)&ir_cal, &size);
      loadedIRCameraInfo = true;
    }

    info->width  = width;
    info->height = height;

    // No distortion
    info->D.resize(5, 0.0);
    info->distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;

    // Simple camera matrix: square pixels (fx = fy), principal point at center
    info->K.assign(0.0);
    info->K[0] = ir_cal.data[0];
    info->K[4] = ir_cal.data[4];
    info->K[2] = ir_cal.data[2];
    info->K[5] = ir_cal.data[5];
    info->K[8] = 1.0;

    // No separate rectified image plane, so R = I
    info->R.assign(0.0);
    info->R[0] = info->R[4] = info->R[8] = 1.0;

    // Then P=K(I|0) = (K|0)
    info->P.assign(0.0);
    info->P[0] = info->K[0]; //fx
    info->P[5] = info->K[4]; //fy
    info->P[2] = info->K[2]; // cx
    info->P[6] = info->K[5]; // cy
    info->P[10] = 1.0;

    // Fill in header
    info->header.stamp  = time;
    info->header.frame_id = depth_frame_id_;
    return info;
  }

  if (depth_info_manager_->isCalibrated())
  {
    info = boost::make_shared<sensor_msgs::CameraInfo>(depth_info_manager_->getCameraInfo());
    if ( info->width != width )
    {
      // Use uncalibrated values
      ROS_WARN_ONCE("Image resolution doesn't match calibration of the IR camera. Using default parameters.");
      info = getDefaultCameraInfo(width, height, device_->getIRFocalLength(height));
    }
  }
  else
  {
    // If uncalibrated, fill in default values
    info = getDefaultCameraInfo(width, height, device_->getDepthFocalLength(height));
  }

  // Fill in header
  info->header.stamp  = time;
  info->header.frame_id = depth_frame_id_;

  return info;
}

sensor_msgs::CameraInfoPtr OpenNI2Driver::getDepthCameraInfo(int width, int height, ros::Time time) const
{
  // The depth image has essentially the same intrinsics as the IR image, BUT the
  // principal point is offset by half the size of the hardware correlation window
  // (probably 9x9 or 9x7 in 640x480 mode). See http://www.ros.org/wiki/kinect_calibration/technical

  //double scaling = (double)width / 640;

  sensor_msgs::CameraInfoPtr info = getIRCameraInfo(width, height, time);
  //info->K[2] -= depth_ir_offset_x_*scaling; // cx
  //info->K[5] -= depth_ir_offset_y_*scaling; // cy
  //info->P[2] -= depth_ir_offset_x_*scaling; // cx
  //info->P[6] -= depth_ir_offset_y_*scaling; // cy

  //printf("depth_ir_offset_x = %f\n", depth_ir_offset_x_);
  //printf("depth_ir_offset_y = %f\n", depth_ir_offset_y_);
  //printf("scaling = %f\n", scaling);

  //printf("info->K[2] = %f\n", info->K[2]);
  //printf("info->K[5] = %f\n", info->K[5]);
  //printf("info->P[2] = %f\n", info->P[2]);
  //printf("info->P[6] = %f\n", info->P[6]);

  /// @todo Could put this in projector frame so as to encode the baseline in P[3]
  return info;
}

void OpenNI2Driver::readConfigFromParameterServer()
{
  if (!pnh_.getParam("device_id", device_id_))
  {
    ROS_WARN ("~device_id is not set! Using first device.");
    device_id_ = "#1";
  }

  // Camera TF frames
  pnh_.param("ir_frame_id", ir_frame_id_, std::string("/openni_ir_optical_frame"));
  pnh_.param("rgb_frame_id", color_frame_id_, std::string("/openni_rgb_optical_frame"));
  pnh_.param("depth_frame_id", depth_frame_id_, std::string("/openni_depth_optical_frame"));

  ROS_DEBUG("ir_frame_id = '%s' ", ir_frame_id_.c_str());
  ROS_DEBUG("rgb_frame_id = '%s' ", color_frame_id_.c_str());
  ROS_DEBUG("depth_frame_id = '%s' ", depth_frame_id_.c_str());

  pnh_.param("rgb_camera_info_url", color_info_url_, std::string());
  pnh_.param("depth_camera_info_url", depth_info_url_, std::string());
  pnh_.param("ir_camera_info_url", ir_info_url_, std::string());
}

std::string OpenNI2Driver::resolveDeviceURI(const std::string& device_id) throw(OpenNI2Exception)
{
  // retrieve available device URIs, they look like this: "1d27/0601@1/5"
  // which is <vendor ID>/<product ID>@<bus number>/<device number>
  boost::shared_ptr<std::vector<std::string> > available_device_URIs =
  device_manager_->getConnectedDeviceURIs();

  // look for '#<number>' format
  if (device_id.size() > 1 && device_id[0] == '#')
  {
    std::istringstream device_number_str(device_id.substr(1));
    int device_number;
    device_number_str >> device_number;
    int device_index = device_number - 1; // #1 refers to first device
    if (device_index >= available_device_URIs->size() || device_index < 0)
    {
      THROW_OPENNI_EXCEPTION(
          "Invalid device number %i, there are %zu devices connected.",
          device_number, available_device_URIs->size());
    }
    else
    {
      return available_device_URIs->at(device_index);
    }
  }
  // look for '<bus>@<number>' format
  //   <bus>  is usb bus id, typically start at 1
  //   <number> is the device number, for consistency with openni_camera, these start at 1
  //         although 0 specifies "any device on this bus"
  else if (device_id.size() > 1 && device_id.find('@') != std::string::npos && device_id.find('/') == std::string::npos)
  {
    // get index of @ character
    size_t index = device_id.find('@');
    if (index <= 0)
    {
      THROW_OPENNI_EXCEPTION(
              "%s is not a valid device URI, you must give the bus number before the @.",
              device_id.c_str());
    }
    if (index >= device_id.size() - 1)
    {
      THROW_OPENNI_EXCEPTION(
              "%s is not a valid device URI, you must give a number after the @, specify 0 for first device",
              device_id.c_str());
    }

    // pull out device number on bus
    std::istringstream device_number_str(device_id.substr(index+1));
    int device_number;
    device_number_str >> device_number;

    // reorder to @<bus>
    std::string bus = device_id.substr(0, index);
    bus.insert(0, "@");

    for (size_t i = 0; i < available_device_URIs->size(); ++i)
    {
      std::string s = (*available_device_URIs)[i];
      if (s.find(bus) != std::string::npos)
      {
        // this matches our bus, check device number
        --device_number;
        if (device_number <= 0)
          return s;
      }
    }

    THROW_OPENNI_EXCEPTION("Device not found %s", device_id.c_str());
  }
  else
  {
    // check if the device id given matches a serial number of a connected device
    /*
    for(std::vector<std::string>::const_iterator it = available_device_URIs->begin();
        it != available_device_URIs->end(); ++it)
    {
      try {
        std::string serial = device_manager_->getSerial(*it);
        if (serial.size() > 0 && device_id == serial)
          return *it;
    
      }
      catch (const OpenNI2Exception& exception)
      {
        ROS_WARN("Could not query serial number of device \"%s\":", exception.what());
      }
    }
    */

    // everything else is treated as part of the device_URI
    bool match_found = false;
    std::string matched_uri;
    for (size_t i = 0; i < available_device_URIs->size(); ++i)
    {
      std::string s = (*available_device_URIs)[i];
      if (s.find(device_id) != std::string::npos)
      {
        if (!match_found)
        {
          matched_uri = s;
          match_found = true;
        }
        else
        {
          // more than one match
          THROW_OPENNI_EXCEPTION("Two devices match the given device id '%s': %s and %s.", device_id.c_str(), matched_uri.c_str(), s.c_str());
        }
      }
    }

    if (!match_found)
      THROW_OPENNI_EXCEPTION("Device not found %s", device_id.c_str());
    
    return matched_uri;
  }

  return "INVALID";
}

void OpenNI2Driver::initDevice()
{
  while (ros::ok() && !device_)
  {
    try
    {
      std::string device_URI = resolveDeviceURI(device_id_);
      device_ = device_manager_->getDevice(device_URI);
    }
    catch (const OpenNI2Exception& exception)
    {
      if (!device_)
      {
        ROS_INFO("No matching device found.... waiting for devices. Reason: %s", exception.what());
        boost::this_thread::sleep(boost::posix_time::seconds(3));
        continue;
      }
      else
      {
        ROS_ERROR("Could not retrieve device. Reason: %s", exception.what());
        exit(-1);
      }
    }
  }

  while (ros::ok() && !device_->isValid())
  {
    ROS_DEBUG("Waiting for device initialization..");
    boost::this_thread::sleep(boost::posix_time::milliseconds(100));
  }

}

void OpenNI2Driver::genVideoModeTableMap()
{
  video_modes_lookup_.clear();

  OpenNI2VideoMode video_mode;

  // SXGA_30Hz
  video_mode.x_resolution_ = 1280;
  video_mode.y_resolution_ = 1024;
  video_mode.frame_rate_ = 30;

  video_modes_lookup_[1] = video_mode;

  // SXGA_15Hz
  video_mode.x_resolution_ = 1280;
  video_mode.y_resolution_ = 1024;
  video_mode.frame_rate_ = 15;

  video_modes_lookup_[2] = video_mode;

  // XGA_30Hz
  video_mode.x_resolution_ = 1280;
  video_mode.y_resolution_ = 720;
  video_mode.frame_rate_ = 30;

  video_modes_lookup_[3] = video_mode;

  // XGA_15Hz
  video_mode.x_resolution_ = 1280;
  video_mode.y_resolution_ = 720;
  video_mode.frame_rate_ = 15;

  video_modes_lookup_[4] = video_mode;

  // VGA_30Hz
  video_mode.x_resolution_ = 640;
  video_mode.y_resolution_ = 480;
  video_mode.frame_rate_ = 30;

  video_modes_lookup_[5] = video_mode;

  // VGA_25Hz
  video_mode.x_resolution_ = 640;
  video_mode.y_resolution_ = 480;
  video_mode.frame_rate_ = 25;

  video_modes_lookup_[6] = video_mode;

  // QVGA_25Hz
  video_mode.x_resolution_ = 320;
  video_mode.y_resolution_ = 240;
  video_mode.frame_rate_ = 25;

  video_modes_lookup_[7] = video_mode;

  // QVGA_30Hz
  video_mode.x_resolution_ = 320;
  video_mode.y_resolution_ = 240;
  video_mode.frame_rate_ = 30;

  video_modes_lookup_[8] = video_mode;

  // QVGA_60Hz
  video_mode.x_resolution_ = 320;
  video_mode.y_resolution_ = 240;
  video_mode.frame_rate_ = 60;

  video_modes_lookup_[9] = video_mode;

  // QQVGA_25Hz
  video_mode.x_resolution_ = 160;
  video_mode.y_resolution_ = 120;
  video_mode.frame_rate_ = 25;

  video_modes_lookup_[10] = video_mode;

  // QQVGA_30Hz
  video_mode.x_resolution_ = 160;
  video_mode.y_resolution_ = 120;
  video_mode.frame_rate_ = 30;

  video_modes_lookup_[11] = video_mode;

  // QQVGA_60Hz
  video_mode.x_resolution_ = 160;
  video_mode.y_resolution_ = 120;
  video_mode.frame_rate_ = 60;

  video_modes_lookup_[12] = video_mode;

  // VGA_15Hz
  video_mode.x_resolution_ = 640;
  video_mode.y_resolution_ = 480;
  video_mode.frame_rate_ = 15;

  video_modes_lookup_[13] = video_mode;

  video_mode.x_resolution_ = 1280;
  video_mode.y_resolution_ = 960;
  video_mode.frame_rate_ = 30;

  video_modes_lookup_[14] = video_mode;

  video_mode.x_resolution_ = 2592;
  video_mode.y_resolution_ = 1944;
  video_mode.frame_rate_ = 30;

  video_modes_lookup_[15] = video_mode;
  
  //ADD
  video_mode.x_resolution_ = 1280;
  video_mode.y_resolution_ = 800;
  video_mode.frame_rate_ = 30;
  video_modes_lookup_[16] = video_mode;
  
  video_mode.x_resolution_ = 640;
  video_mode.y_resolution_ = 400;
  video_mode.frame_rate_ = 30;
  video_modes_lookup_[17] = video_mode;
  
  video_mode.x_resolution_ = 320;
  video_mode.y_resolution_ = 200;
  video_mode.frame_rate_ = 30;
  video_modes_lookup_[18] = video_mode;
  
  video_mode.x_resolution_ = 1920;
  video_mode.y_resolution_ = 1080;
  video_mode.frame_rate_ = 30;
  video_modes_lookup_[19] = video_mode;
  
  video_mode.x_resolution_ = 640;
  video_mode.y_resolution_ = 360;
  video_mode.frame_rate_ = 30;
  video_modes_lookup_[20] = video_mode;
}

int OpenNI2Driver::lookupVideoModeFromDynConfig(int mode_nr, OpenNI2VideoMode& video_mode)
{
  int ret = -1;

  std::map<int, OpenNI2VideoMode>::const_iterator it;

  it = video_modes_lookup_.find(mode_nr);

  if (it!=video_modes_lookup_.end())
  {
    video_mode = it->second;
    ret = 0;
  }

  return ret;
}

sensor_msgs::ImageConstPtr OpenNI2Driver::rawToFloatingPointConversion(sensor_msgs::ImageConstPtr raw_image)
{
  static const float bad_point = std::numeric_limits<float>::quiet_NaN ();

  sensor_msgs::ImagePtr new_image = boost::make_shared<sensor_msgs::Image>();

  new_image->header = raw_image->header;
  new_image->width = raw_image->width;
  new_image->height = raw_image->height;
  new_image->is_bigendian = 0;
  new_image->encoding = sensor_msgs::image_encodings::TYPE_32FC1;
  new_image->step = sizeof(float)*raw_image->width;

  std::size_t data_size = new_image->width*new_image->height;
  new_image->data.resize(data_size*sizeof(float));

  const unsigned short* in_ptr = reinterpret_cast<const unsigned short*>(&raw_image->data[0]);
  float* out_ptr = reinterpret_cast<float*>(&new_image->data[0]);

  for (std::size_t i = 0; i<data_size; ++i, ++in_ptr, ++out_ptr)
  {
    if (*in_ptr==0 || *in_ptr==0x7FF)
    {
      *out_ptr = bad_point;
    } 
    else
    {
      *out_ptr = static_cast<float>(*in_ptr)/1000.0f;
    }
  }

  return new_image;
}

}
