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
#include <iostream>
#include <algorithm>

#include "percipio_camera/percipio_driver.h"
#include "percipio_camera/percipio_exception.h"

#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/distortion_models.h>

#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/thread/thread.hpp>

//#include <opencv2/opencv.hpp>
//#include <opencv2/highgui/highgui.hpp>

namespace percipio_wrapper
{
struct calib_data{
  float intrinsic_data[9];
  float distortion_data[12];
};

struct calib_data depth_cal, color_cal;

sensor_msgs::CameraInfoPtr info;

sensor_msgs::CameraInfoPtr color_info;
bool loadedColorCameraInfo = false;

PercipioDriver::PercipioDriver(ros::NodeHandle& n, ros::NodeHandle& pnh) :
  nh_(n),
  pnh_(pnh),
  device_manager_(PercipioDeviceManager::getSingelton()),
  config_init_(false),
  depth_registration_(false),
  gvsp_resend_(false),
  color_depth_synchronization_(true),
  use_device_time_(false),
  tof_depth_channel_(0),
  tof_depth_quality_(4),
  tof_depth_filter_threshold_(0),
  tof_depth_modulation_threshold_(640),
  tof_depth_jitter_threshold_(0),
  tof_depth_hdr_ratio_(1),

  data_skip_ir_counter_(0),
  data_skip_color_counter_(0),
  data_skip_depth_counter_ (0),
  ir_subscribers_(false),
  color_subscribers_(false),
  point3d_subscribers_(false),
  depth_subscribers_(false)
{
  readConfigFromParameterServer();

  initDevice();
  
  if(!device_->hasLaserPower())
    pnh.deleteParam("laser_power");
  if(!device_->hasAutoExposure())
    pnh.deleteParam("auto_exposure");
  if(!device_->hasAutoWhiteBalance())
    pnh.deleteParam("auto_white_balance");
  if(!device_->hasColorExposureTime())
    pnh.deleteParam("rgb_exposure_time");

  if(!device_->hasColorAecROI()) {
    pnh.deleteParam("auto_exposure_p1_x");
    pnh.deleteParam("auto_exposure_p1_y");
    pnh.deleteParam("auto_exposure_p2_x");
    pnh.deleteParam("auto_exposure_p2_y");
  }
  
  if(!device_->hasColorAnalogGain())
    pnh.deleteParam("rgb_analog_gain");
  if(!device_->hasColorRedGain())
    pnh.deleteParam("rgb_r_gain");
  if(!device_->hasColorGreenGain())
    pnh.deleteParam("rgb_g_gain");
  if(!device_->hasColorBlueGain())
    pnh.deleteParam("rgb_b_gain");
  if(!device_->hasColorAecROI())
    pnh.deleteParam("rgb_aec_roi");
  
  if(!device_->hasIrExposureTime())
    pnh.deleteParam("ir_exposure_time");
  if(!device_->hasIrAnalogGain())
    pnh.deleteParam("ir_analog_gain");  
  if(!device_->hasIrGain())
    pnh.deleteParam("ir_gain");

  // Initialize dynamic reconfigure
  reconfigure_server_.reset(new ReconfigureServer(pnh));
  reconfigure_server_->setCallback(boost::bind(&PercipioDriver::configCb, this, _1, _2));

  while (!config_init_)
  {
    ROS_DEBUG("Waiting for dynamic reconfigure configuration.");
    boost::this_thread::sleep(boost::posix_time::milliseconds(100));
  }
  ROS_DEBUG("Dynamic reconfigure configuration received.");

  advertiseROSTopics();
}

void PercipioDriver::advertiseROSTopics()
{
  // Allow remapping namespaces rgb, ir, depth, depth_registered
  ros::NodeHandle color_nh(nh_, "rgb");
  image_transport::ImageTransport color_it(color_nh);
  
  ros::NodeHandle ir_nh(nh_, "ir");
  image_transport::ImageTransport ir_it(ir_nh);
  
  ros::NodeHandle depth_nh(nh_, "depth");
  image_transport::ImageTransport depth_it(depth_nh);
  
  ros::NodeHandle point3d_nh(nh_, "depth");
  image_transport::ImageTransport point3d_it(point3d_nh);
  
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
      image_transport::SubscriberStatusCallback itssc = boost::bind(&PercipioDriver::colorConnectCb, this);
      ros::SubscriberStatusCallback rssc = boost::bind(&PercipioDriver::colorConnectCb, this);
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
      image_transport::SubscriberStatusCallback itssc = boost::bind(&PercipioDriver::irConnectCb, this);
      ros::SubscriberStatusCallback rssc = boost::bind(&PercipioDriver::irConnectCb, this);
      pub_ir_ = ir_it.advertiseCamera("image", 1, itssc, itssc, rssc, rssc);
    }
  }

  if (device_->hasDepthSensor())
  {
    if((depth_video_mode_.x_resolution_ > 0) && (depth_video_mode_.y_resolution_ > 0))
    {
      image_transport::SubscriberStatusCallback itssc = boost::bind(&PercipioDriver::depthConnectCb, this);
      ros::SubscriberStatusCallback rssc = boost::bind(&PercipioDriver::depthConnectCb, this);
      pub_depth_ = depth_it.advertiseCamera("image_raw", 1, itssc, itssc, rssc, rssc);
      //pub_point3d_ = point3d_it.advertiseCamera("image", 1, itssc, itssc, rssc, rssc);
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

  color_name = "rgb"   + serial_number;
  depth_name  = "depth" + serial_number;
  ir_name    = "ir" + serial_number;

  // Load the saved calibrations, if they exist
  color_info_manager_ = boost::make_shared<camera_info_manager::CameraInfoManager>(color_nh, color_name, color_info_url_);
  depth_info_manager_  = boost::make_shared<camera_info_manager::CameraInfoManager>(depth_nh,  depth_name,  depth_info_url_);
  ir_info_manager     = boost::make_shared<camera_info_manager::CameraInfoManager>(ir_nh,  ir_name,  ir_info_url_);

  get_serial_server = nh_.advertiseService("get_serial", &PercipioDriver::getSerialCb,this);

}

bool PercipioDriver::getSerialCb(percipio_camera::GetSerialRequest& req, percipio_camera::GetSerialResponse& res) {
  res.serial = device_manager_->getSerial(device_->getUri());
  return true;
}

void PercipioDriver::configCb(Config &config, uint32_t level)
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
      device_->getColorAecROI(&auto_exposure_p1_x_, &auto_exposure_p1_y_, &auto_exposure_p2_x_, &auto_exposure_p2_y_);

      device_->getIrExposureTime(&m_ir_exposure_time_);
      device_->getIrAnalogGain(&m_ir_analog_gain_);
      device_->getIrGain(&m_ir_gain_);

      depth_speckle_filter_ = device_->getDepthSpecFilterEn();
      max_speckle_size_ = device_->getDepthSpecFilterSpecSize();
      max_speckle_diff_ = device_->getDepthSpeckFilterDiff();

      config.depth_speckle_filter = depth_speckle_filter_;
      config.max_speckle_size = max_speckle_size_;
      config.max_speckle_diff = max_speckle_diff_;
      
      config.laser_power = m_laser_power_;
  
      config.auto_exposure = auto_exposure_;
      config.auto_white_balance = auto_white_balance_;

      config.auto_exposure_p1_x = auto_exposure_p1_x_;
      config.auto_exposure_p1_y = auto_exposure_p1_y_;
      config.auto_exposure_p2_x = auto_exposure_p2_x_;
      config.auto_exposure_p2_y = auto_exposure_p2_y_;
  
      config.rgb_analog_gain = m_rgb_analog_gain_;
      config.rgb_r_gain = m_rgb_r_gain_;
      config.rgb_g_gain = m_rgb_g_gain_;
      config.rgb_b_gain = m_rgb_b_gain_;
      config.rgb_exposure_time = m_rgb_exposure_time_;

      config.ir_exposure_time = m_ir_exposure_time_;
      config.ir_analog_gain = m_ir_analog_gain_;
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

    if((auto_exposure_p1_x_ != config.auto_exposure_p1_x) ||
       (auto_exposure_p1_y_ != config.auto_exposure_p1_y) ||
       (auto_exposure_p2_x_ != config.auto_exposure_p2_x) ||
       (auto_exposure_p2_y_ != config.auto_exposure_p2_y)) {
      auto_exposure_p1_x_ = config.auto_exposure_p1_x;
      auto_exposure_p1_y_ = config.auto_exposure_p1_y;
      auto_exposure_p2_x_ = config.auto_exposure_p2_x;
      auto_exposure_p2_y_ = config.auto_exposure_p2_y;

      ROS_WARN("====ROI : (%f %f) - (%f %f)", auto_exposure_p1_x_, auto_exposure_p1_y_, auto_exposure_p2_x_, auto_exposure_p2_y_);
      if(device_) device_->setColorAecROI(auto_exposure_p1_x_, auto_exposure_p1_y_, auto_exposure_p2_x_, auto_exposure_p2_y_); 
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

    if(m_ir_exposure_time_ != config.ir_exposure_time) {
      m_ir_exposure_time_ = config.ir_exposure_time;
      if(device_) device_->setIrExposureTime(m_ir_exposure_time_);
    }

    if(m_ir_analog_gain_ != config.ir_analog_gain) {
      m_ir_analog_gain_ = config.ir_analog_gain;
      if(device_) device_->setIrAnalogGain(m_ir_analog_gain_);
    }
  
    if(m_ir_gain_ != config.ir_gain) {
      m_ir_gain_ = config.ir_gain;
      if(device_) device_->setIrGain(m_ir_gain_);
    }

    if(depth_speckle_filter_ != config.depth_speckle_filter) {
      depth_speckle_filter_ = config.depth_speckle_filter;
      if(device_) device_->setDepthSpecFilterEn(depth_speckle_filter_);
    }

    if(max_speckle_size_ != config.max_speckle_size) {
      max_speckle_size_ = config.max_speckle_size;
      if(device_) device_->setDepthSpecFilterSpecSize(max_speckle_size_);
    }

    if(max_speckle_diff_ != config.max_speckle_diff) {
      max_speckle_diff_ = config.max_speckle_diff;
      if(device_) device_->setDepthSpeckFilterDiff(max_speckle_diff_);
    }
  }
  
  z_scaling_ = config.z_scaling;

  // assign pixel format
  //ir_video_mode_.pixel_format_ = PIXEL_FORMAT_GRAY16;
  ir_video_mode_.pixel_format_ = PIXEL_FORMAT_GRAY8;
  color_video_mode_.pixel_format_ = PIXEL_FORMAT_RGB888;
  depth_video_mode_.pixel_format_ = PIXEL_FORMAT_DEPTH_1_MM;

  auto_exposure_ = config.auto_exposure;
  auto_exposure_p1_x_ = config.auto_exposure_p1_x;
  auto_exposure_p1_y_ = config.auto_exposure_p1_y;
  auto_exposure_p2_x_ = config.auto_exposure_p2_x;
  auto_exposure_p2_y_ = config.auto_exposure_p2_y;

  auto_white_balance_ = config.auto_white_balance;

  data_skip_ = config.data_skip+1;

  use_device_time_ = config.use_device_time;

  applyConfigToPercipioDevice();

  config_init_ = true;

  old_config_ = config;
}

PercipioVideoMode PercipioDriver::getIRVideoMode()
{
  return device_->getIRVideoMode();
}

PercipioVideoMode PercipioDriver::getColorVideoMode()
{
  return device_->getColorVideoMode();
}

PercipioVideoMode PercipioDriver::getDepthVideoMode()
{
  return device_->getDepthVideoMode();
}

void PercipioDriver::applyConfigToPercipioDevice()
{
  data_skip_ir_counter_ = 0;
  data_skip_color_counter_= 0;
  data_skip_depth_counter_ = 0;
  
  ir_video_mode_ = getIRVideoMode();
  color_video_mode_ = getColorVideoMode();
  depth_video_mode_ = getDepthVideoMode();

  device_->setUseDeviceTimer(use_device_time_);
}

void PercipioDriver::colorConnectCb()
{
  boost::lock_guard<boost::mutex> lock(connect_mutex_);
  color_subscribers_ = pub_color_.getNumSubscribers() > 0;
  if (color_subscribers_ && !device_->isColorStreamStarted())
  { 
    device_->setColorFrameCallback(boost::bind(&PercipioDriver::newColorFrameCallback, this, _1));

    ROS_INFO("Starting color stream.");
    device_->startColorStream();
  }
  else if (!color_subscribers_ && device_->isColorStreamStarted())
  {
    ROS_INFO("Stopping color stream.");
    device_->stopColorStream();
  }
}

void PercipioDriver::depthConnectCb()
{
  boost::lock_guard<boost::mutex> lock(connect_mutex_);
  //point3d_subscribers_ = pub_point3d_.getNumSubscribers() > 0;
  depth_subscribers_ = pub_depth_.getNumSubscribers() > 0;
  //bool need_depth = point3d_subscribers_ || depth_subscribers_;
  bool need_depth = depth_subscribers_;

  if (need_depth && !device_->isDepthStreamStarted())
  {
    device_->setDepthFrameCallback(boost::bind(&PercipioDriver::newDepthFrameCallback, this, _1));

    ROS_INFO("Starting depth stream.");
    device_->startDepthStream();
  }
  else if (!need_depth && device_->isDepthStreamStarted())
  {
    ROS_INFO("Stopping depth stream.");
    device_->stopDepthStream();
  }
}

void PercipioDriver::irConnectCb()
{
  boost::lock_guard<boost::mutex> lock(connect_mutex_);
  ir_subscribers_ = pub_ir_.getNumSubscribers() > 0;

  if (ir_subscribers_ && !device_->isIRStreamStarted())
  {
    device_->setIRFrameCallback(boost::bind(&PercipioDriver::newIRFrameCallback, this, _1));

    ROS_INFO("Starting IR stream.");
    device_->startIRStream();
  }
  else if (!ir_subscribers_ && device_->isIRStreamStarted())
  {
    ROS_INFO("Stopping IR stream.");
    device_->stopIRStream();
  }
}

void PercipioDriver::newIRFrameCallback(sensor_msgs::ImagePtr image)
{
  if ((++data_skip_ir_counter_)%data_skip_==0)
  {
    data_skip_ir_counter_ = 0;

    if (ir_subscribers_)
    {
      image->header.frame_id = ir_frame_id_;
      image->header.stamp = image->header.stamp + ir_time_offset_;
      
      pub_ir_.publish(image, getDepthCameraInfo(image->width, image->height, image->header.stamp));
    }
  }
}

void PercipioDriver::newColorFrameCallback(sensor_msgs::ImagePtr image)
{
  if ((++data_skip_color_counter_)%data_skip_==0)
  {
    data_skip_color_counter_ = 0;
    if (color_subscribers_)
    {
      image->header.frame_id = color_frame_id_;
      image->header.stamp = image->header.stamp + color_time_offset_;

      pub_color_.publish(image, getColorCameraInfo(image->width, image->height, image->header.stamp, true));
    }
  }
}

void PercipioDriver::newDepthFrameCallback(sensor_msgs::ImagePtr image)
{
  if ((++data_skip_depth_counter_)%data_skip_==0)
  {
    data_skip_depth_counter_ = 0;

    //if (depth_subscribers_||point3d_subscribers_)
    if(depth_subscribers_)
    {
      //sensor_msgs::ImageConstPtr floating_point_image;

      //if (point3d_subscribers_ )
      //  floating_point_image = rawToFloatingPointConversion(image, z_scaling_);

      if(std::abs(z_scaling_ - 1.f ) > 0.001)
      {
        uint16_t* data = reinterpret_cast<uint16_t*>(&image->data[0]);
        for (unsigned int i = 0; i < image->width * image->height; ++i)
          if (data[i] != 0)
        data[i] = static_cast<uint16_t>(data[i]  *  z_scaling_) + 0.5f;
      }

      sensor_msgs::CameraInfoPtr cam_info;
      if(percipio::IMAGE_REGISTRATION_DEPTH_TO_COLOR == device_.get()->getImageRegistrationMode())
      {
        image->header.frame_id = color_frame_id_;
        cam_info = getColorCameraInfo(image->width,image->height, image->header.stamp, false);
      }
      else
      {
        image->header.frame_id = depth_frame_id_;
        cam_info = getDepthCameraInfo(image->width,image->height, image->header.stamp);
      }

      if (depth_subscribers_)
      {
        pub_depth_.publish(image, cam_info);
      }

      //if (point3d_subscribers_ )
      //{
      //  if(floating_point_image != nullptr)
      //    pub_point3d_.publish(floating_point_image, cam_info);
      //}
    }
  }
}

// Methods to get calibration parameters for the various cameras
sensor_msgs::CameraInfoPtr PercipioDriver::getDefaultCameraInfo(int width, int height, double f)
{
  sensor_msgs::CameraInfoPtr info = boost::make_shared<sensor_msgs::CameraInfo>();

  info->width  = width;
  info->height = height;

  // No distortion
  info->D.resize(12, 0.0);
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
sensor_msgs::CameraInfoPtr PercipioDriver::getColorCameraInfo(int width, int height, ros::Time time, bool isColor)
{
  //boost::lock_guard<boost::mutex> lock(connect_mutex_);
  if (device_->getVendor() == "Percipio")
  {
    info = boost::make_shared<sensor_msgs::CameraInfo>();

    int size = sizeof(color_cal.intrinsic_data);

    //get intristic from percipio firmware
    device_->getColorCalibIntristic((void*)color_cal.intrinsic_data, &size);
    
    info->width  = width;
    info->height = height;
    
    info->D.resize(12, 0.0);
    //depth stream may need color camera info,but without distortion while map to color coordinate
    if(isColor) {
      size = sizeof(color_cal.distortion_data);
      device_->getColorCalibDistortion((void*)color_cal.distortion_data, &size);
      for(int i = 0; i < 12; i++) {
        info->D[i] = color_cal.distortion_data[i];
      }
    }
    info->distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;

    // Simple camera matrix: square pixels (fx = fy), principal point at center
    info->K.assign(0.0);
    info->K[0] = color_cal.intrinsic_data[0];
    info->K[4] = color_cal.intrinsic_data[4];
    info->K[2] = color_cal.intrinsic_data[2];
    info->K[5] = color_cal.intrinsic_data[5];
    info->K[8] = 1.0;

    // No separate rectified image plane, so R = I
    info->R.assign(0.0);
    info->R[0] = info->R[4] = info->R[8] = 1.0;

    // Then P=K(I|0) = (K|0)
    info->P.assign(0.0);
    info->P[0] = info->K[0]; //fx
    info->P[5] = info->K[4]; //fy
    info->P[2] = info->K[2]; //cx
    info->P[6] = info->K[5]; //cy
    info->P[10] = 1.0;

    // Fill in header
    info->header.stamp  = time;
    info->header.frame_id = color_frame_id_;
    
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
  info->header.frame_id = color_frame_id_;

  return info;
}


sensor_msgs::CameraInfoPtr PercipioDriver::getDepthCameraInfo(int width, int height, ros::Time time)
{
  //boost::lock_guard<boost::mutex> lock(connect_mutex_);
  if (device_->getVendor() == "Percipio")
  {
    info = boost::make_shared<sensor_msgs::CameraInfo>();

    int size = sizeof(depth_cal.intrinsic_data);
    
    //get intristic from percipio firmware
    device_->getDepthCalibIntristic((void*)&depth_cal.intrinsic_data, &size);
    
    info->width  = width;
    info->height = height;
    
    // No distortion
    info->D.resize(12, 0.0);
    info->distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;

    // Simple camera matrix: square pixels (fx = fy), principal point at center
    info->K.assign(0.0);
    info->K[0] = depth_cal.intrinsic_data[0];
    info->K[4] = depth_cal.intrinsic_data[4];
    info->K[2] = depth_cal.intrinsic_data[2];
    info->K[5] = depth_cal.intrinsic_data[5];
    info->K[8] = 1.0;

    // No separate rectified image plane, so R = I
    info->R.assign(0.0);
    info->R[0] = info->R[4] = info->R[8] = 1.0;

    // Then P=K(I|0) = (K|0)
    info->P.assign(0.0);
    info->P[0] = info->K[0]; //fx
    info->P[5] = info->K[4]; //fy
    info->P[2] = info->K[2]; //cx
    info->P[6] = info->K[5]; //cy
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

void PercipioDriver::readConfigFromParameterServer()
{
  if (!pnh_.getParam("device_id", device_id_))
  {
    ROS_WARN ("~device_id is not set! Using first device.");
    device_id_ = "#1";
  }

  if(!pnh_.getParam("gvsp_resend", gvsp_resend_))
  {
    ROS_WARN ("~gvsp resend is not set! Using default.");
    gvsp_resend_ = false;
  }

  if (!pnh_.getParam("rgb_resolution", rgb_resolution_))
  {
    ROS_WARN ("~rgb_resolution is not set! Try using default.");
    rgb_resolution_ = "640x480";
  }

  if (!pnh_.getParam("depth_resolution", depth_resolution_))
  {
    ROS_WARN ("~depth_resolution is not set! Try using default.");
    depth_resolution_ = "640x480";
  }

  if (!pnh_.getParam("color_undistortion", color_undistortion_))
  {
    ROS_WARN ("~color_undistortion is not set! Using default.");
    color_undistortion_ = true;
  }

  if (!pnh_.getParam("depth_registration", depth_registration_))
  {
    ROS_WARN ("~depth_registration is not set! Using default.");
    depth_registration_ = false;
  }

  if (!pnh_.getParam("color_depth_synchronization", color_depth_synchronization_))
  {
    ROS_WARN ("~color_depth_synchronization is not set! Using default.");
    color_depth_synchronization_ = true;
  }

  if (!pnh_.getParam("tof_channel", tof_depth_channel_))
  {
    tof_depth_channel_ = PARAMTER_DEFAULT;
  }

  if (!pnh_.getParam("tof_depth_quality", tof_depth_quality_))
  {
    tof_depth_quality_ = PARAMTER_DEFAULT;
  }

  if (!pnh_.getParam("tof_filter_threshold", tof_depth_filter_threshold_))
  {
    tof_depth_filter_threshold_ = PARAMTER_DEFAULT;
  }

  if (!pnh_.getParam("tof_modulation_threshold", tof_depth_modulation_threshold_))
  {
    tof_depth_modulation_threshold_ = PARAMTER_DEFAULT;
  }

  if (!pnh_.getParam("tof_jitter_threshold", tof_depth_jitter_threshold_))
  {
    tof_depth_jitter_threshold_ = PARAMTER_DEFAULT;
  }

  if (!pnh_.getParam("tof_hdr_ratio", tof_depth_hdr_ratio_))
  {
    tof_depth_hdr_ratio_ = PARAMTER_DEFAULT;
  }

  //SGBM paramters
  {
    pnh_.getParam("sgbm_image_channel_num",             sgbm_image_channel_num_);
    pnh_.getParam("sgbm_disparity_num",                 sgbm_disparity_num_);
    pnh_.getParam("sgbm_disparity_offset",              sgbm_disparity_offset_);
    pnh_.getParam("sgbm_match_window_height",           sgbm_match_window_height_);
    pnh_.getParam("sgbm_semi_global_param_p1",          sgbm_semi_global_param_p1_);
    pnh_.getParam("sgbm_semi_global_param_p2",          sgbm_semi_global_param_p2_);
    pnh_.getParam("sgbm_unique_factor_param",           sgbm_unique_factor_param_);
    pnh_.getParam("sgbm_unique_min_absolute_diff",      sgbm_unique_min_absolute_diff_);
    pnh_.getParam("sgbm_cost_param",                    sgbm_cost_param_);
    pnh_.getParam("sgbm_enable_half_window_size",       sgbm_enable_half_window_size_);
    pnh_.getParam("sgbm_match_window_width",            sgbm_match_window_width_);
    pnh_.getParam("sgbm_enable_median_filter",          sgbm_enable_median_filter_);
    pnh_.getParam("sgbm_enable_lrc_check",              sgbm_enable_lrc_check_);
    pnh_.getParam("sgbm_lrc_max_diff",                  sgbm_lrc_max_diff_);
    pnh_.getParam("sgbm_median_filter_thresh",          sgbm_median_filter_thresh_);
    pnh_.getParam("sgbm_semi_global_param_p1_scale",    sgbm_semi_global_param_p1_scale_);

  }

  // Camera TF frames
  pnh_.param("ir_frame_id", ir_frame_id_, std::string("/percipio_ir_optical_frame"));
  pnh_.param("rgb_frame_id", color_frame_id_, std::string("/percipio_rgb_optical_frame"));
  pnh_.param("depth_frame_id", depth_frame_id_, std::string("/percipio_depth_optical_frame"));

  ROS_DEBUG("ir_frame_id = '%s' ", ir_frame_id_.c_str());
  ROS_DEBUG("rgb_frame_id = '%s' ", color_frame_id_.c_str());
  ROS_DEBUG("depth_frame_id = '%s' ", depth_frame_id_.c_str());

  pnh_.param("rgb_camera_info_url", color_info_url_, std::string());
  pnh_.param("depth_camera_info_url", depth_info_url_, std::string());
  pnh_.param("ir_camera_info_url", ir_info_url_, std::string());
}

std::string PercipioDriver::resolveDeviceURI(const std::string& device_id)
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
#if 0      
      THROW_PERCIPIO_EXCEPTION(
          "Invalid device number %i, there are %zu devices connected.",
          device_number, available_device_URIs->size());
#endif
      percipio::Percipio::initialize();

    }
    else
    {
      return available_device_URIs->at(device_index);
    }
  }
  // look for '<bus>@<number>' format
  //   <bus>  is usb bus id, typically start at 1
  //   <number> is the device number, for consistency with percipio_camera, these start at 1
  //         although 0 specifies "any device on this bus"
  else if (device_id.size() > 1 && device_id.find('@') != std::string::npos && device_id.find('/') == std::string::npos)
  {
    // get index of @ character
    size_t index = device_id.find('@');
    if (index <= 0)
    {
      THROW_PERCIPIO_EXCEPTION(
              "%s is not a valid device URI, you must give the bus number before the @.",
              device_id.c_str());
    }
    if (index >= device_id.size() - 1)
    {
      THROW_PERCIPIO_EXCEPTION(
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

    THROW_PERCIPIO_EXCEPTION("Device not found %s", device_id.c_str());
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
      catch (const PercipioException& exception)
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
          THROW_PERCIPIO_EXCEPTION("Two devices match the given device id '%s': %s and %s.", device_id.c_str(), matched_uri.c_str(), s.c_str());
        }
      }
    }

    if (!match_found)
      THROW_PERCIPIO_EXCEPTION("Device not found %s", device_id.c_str());
    
    return matched_uri;
  }

  return "";
}

bool PercipioDriver::resolveDeviceResolution(const std::string& resolution_, int& width, int& height)
{
  size_t pos = resolution_.find('x');
  if((pos != 0) && (pos != std::string::npos))
  {
    std::string str_width = resolution_.substr(0, pos);
    std::string str_height = resolution_.substr(pos+1, resolution_.length());
    width = atoi(str_width.c_str());
    height = atoi(str_height.c_str());
    return true;
  }
  return false;
}

void PercipioDriver::initDevice()
{
  while (ros::ok() && !device_)
  {
    try
    {
      std::string device_URI = resolveDeviceURI(device_id_);

      if(!device_URI.length()) {
        ROS_INFO("No matching device found.... waiting for devices.");
        boost::this_thread::sleep(boost::posix_time::seconds(3));
        continue;
      }

      int rgb_width, rgb_height;
      int depth_width, depth_height;
      resolveDeviceResolution(rgb_resolution_, rgb_width, rgb_height);
      resolveDeviceResolution(depth_resolution_, depth_width, depth_height);
      
      device_ = device_manager_->getDevice(device_URI);

      device_.get()->setGvspResendEnable(gvsp_resend_);

      device_.get()->setColorResolution(rgb_width, rgb_height);
      device_.get()->setDepthResolutuon(depth_width, depth_height);

      device_.get()->setColorUndistortion(color_undistortion_);

      if(device_.get()->isImageRegistrationModeSupported()) {
        device_.get()->setImageRegistrationMode(depth_registration_);
      }

      if(device_.get()->isDeviceRGBDSyncSupported()) {
        device_.get()->setDeviceRGBDSynchronization(color_depth_synchronization_);
      }

      if(tof_depth_channel_!= PARAMTER_DEFAULT)
        device_->setTofDepthChannel(tof_depth_channel_);
      if(tof_depth_quality_ != PARAMTER_DEFAULT)
        device_->setTofDepthQuality(tof_depth_quality_);
      if(tof_depth_filter_threshold_ != PARAMTER_DEFAULT) {
        device_->setTofFilterThreshold(tof_depth_filter_threshold_);
      }
      if(tof_depth_modulation_threshold_ != PARAMTER_DEFAULT) {
        device_->setTofModulationThreshold(tof_depth_modulation_threshold_);
      }
      if(tof_depth_jitter_threshold_ != PARAMTER_DEFAULT) {
        device_->setTofJitterThreshold(tof_depth_jitter_threshold_);
      }
      if(tof_depth_hdr_ratio_ != PARAMTER_DEFAULT) {
        device_->setTofHdrRatio(tof_depth_hdr_ratio_);
      }
      //SGBM paramters init
      {
        if(sgbm_image_channel_num_!= PARAMTER_DEFAULT)
          device_->setSgbmImageChanNumber(sgbm_image_channel_num_);
        if(sgbm_disparity_num_!= PARAMTER_DEFAULT)
          device_->setSgbmDispNumber(sgbm_disparity_num_);
        if(sgbm_disparity_offset_!= PARAMTER_DEFAULT)
          device_->setSgbmDispOffset(sgbm_disparity_offset_);
        if(sgbm_match_window_height_!= PARAMTER_DEFAULT)
          device_->setSgbmMatchWinHeight(sgbm_match_window_height_);
        if(sgbm_semi_global_param_p1_!= PARAMTER_DEFAULT)
          device_->setSgbmSemiP1(sgbm_semi_global_param_p1_);
        if(sgbm_semi_global_param_p2_!= PARAMTER_DEFAULT)
          device_->setSgbmSemiP2(sgbm_semi_global_param_p2_);
        if(sgbm_unique_factor_param_!= PARAMTER_DEFAULT)
          device_->setSgbmUniqueFactor(sgbm_unique_factor_param_);
        if(sgbm_unique_min_absolute_diff_!= PARAMTER_DEFAULT)
          device_->setSgbmUniqueAbsDiff(sgbm_unique_min_absolute_diff_);
        if(sgbm_cost_param_!= PARAMTER_DEFAULT)
          device_->setSgbmCostParam(sgbm_cost_param_);
        if(sgbm_enable_half_window_size_!= PARAMTER_DEFAULT)
          device_->setSgbmHalfWinSizeEn(sgbm_enable_half_window_size_);
        if(sgbm_match_window_width_!= PARAMTER_DEFAULT)
          device_->setSgbmMatchWinWidth(sgbm_match_window_width_);
        if(sgbm_enable_median_filter_!= PARAMTER_DEFAULT)
          device_->setSgbmMedianFilterEn(sgbm_enable_median_filter_);
        if(sgbm_enable_lrc_check_!= PARAMTER_DEFAULT)
          device_->setSgbmLRCCheckEn(sgbm_enable_lrc_check_);
        if(sgbm_lrc_max_diff_!= PARAMTER_DEFAULT)
          device_->setSgbmLRCMaxDiff(sgbm_lrc_max_diff_);
        if(sgbm_median_filter_thresh_!= PARAMTER_DEFAULT)
          device_->setSgbmMedianFilterThresh(sgbm_median_filter_thresh_);
        if(sgbm_semi_global_param_p1_scale_!= PARAMTER_DEFAULT)
          device_->setSgbmSemiP1Scale(sgbm_semi_global_param_p1_scale_);
      }
    }
    catch (const PercipioException& exception)
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

sensor_msgs::ImageConstPtr PercipioDriver::rawToFloatingPointConversion(sensor_msgs::ImageConstPtr raw_image, float scale)
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
    if (*in_ptr==0)
    {
      *out_ptr = bad_point;
    } 
    else
    {
      *out_ptr = static_cast<float>(*in_ptr) * scale;
    }
  }

  return new_image;
}

}
