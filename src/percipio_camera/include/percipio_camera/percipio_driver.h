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

#ifndef PERCIPIO_DRIVER_H
#define PERCIPIO_DRIVER_H

#include <boost/shared_ptr.hpp>
#include <boost/cstdint.hpp>
#include <boost/bind.hpp>
#include <boost/function.hpp>

#include <sensor_msgs/Image.h>

#include <dynamic_reconfigure/server.h>
#include <percipio_camera/PercipioConfig.h>

#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>

#include <string>
#include <vector>

#include "percipio_camera/percipio_device_manager.h"
#include "percipio_camera/percipio_device.h"
#include "percipio_camera/percipio_video_mode.h"
#include "percipio_camera/GetSerial.h"

#include <ros/ros.h>

#include "percipio_camera/percipio_driver.h"
#include <nodelet/nodelet.h>

namespace percipio_wrapper
{

class PercipioDriver
{
public:
  PercipioDriver(ros::NodeHandle& n, ros::NodeHandle& pnh) ;

private:
  typedef percipio_camera::PercipioConfig Config;
  typedef dynamic_reconfigure::Server<Config> ReconfigureServer;

  void newIRFrameCallback(sensor_msgs::ImagePtr image);
  void newColorFrameCallback(sensor_msgs::ImagePtr image);
  void newDepthFrameCallback(sensor_msgs::ImagePtr image);

  // Methods to get calibration parameters for the various cameras
  sensor_msgs::CameraInfoPtr getDefaultCameraInfo(int width, int height, double f);
  sensor_msgs::CameraInfoPtr getColorCameraInfo(int width, int height, ros::Time time, bool isColor);
  sensor_msgs::CameraInfoPtr getDepthCameraInfo(int width, int height, ros::Time time);

  void readConfigFromParameterServer();

  // resolves non-URI device IDs to URIs, e.g. '#1' is resolved to the URI of the first device
  std::string resolveDeviceURI(const std::string& device_id);
  bool resolveDeviceResolution(const std::string& resolution_, int& width, int& height);
  void initDevice();

  void advertiseROSTopics();

  void colorConnectCb();
  void depthConnectCb();
  void irConnectCb();

  bool getSerialCb(percipio_camera::GetSerialRequest& req, percipio_camera::GetSerialResponse& res);

  void configCb(Config &config, uint32_t level);

  void applyConfigToPercipioDevice();

  sensor_msgs::ImageConstPtr rawToFloatingPointConversion(sensor_msgs::ImageConstPtr raw_image, float scale);

  //void setIRVideoMode(const PercipioVideoMode& ir_video_mode);
  //void setColorVideoMode(const PercipioVideoMode& color_video_mode);
  //void setDepthVideoMode(const PercipioVideoMode& depth_video_mode);
  
  PercipioVideoMode getIRVideoMode();
  PercipioVideoMode getColorVideoMode();
  PercipioVideoMode getDepthVideoMode();

  ros::NodeHandle& nh_;
  ros::NodeHandle& pnh_;

  boost::shared_ptr<PercipioDeviceManager> device_manager_;
  boost::shared_ptr<PercipioDevice> device_;

  std::string device_id_;
  std::string rgb_resolution_;
  std::string depth_resolution_;

  bool color_undistortion_;

  bool depth_registration_;

  bool color_depth_synchronization_;

  /** \brief get_serial server*/
  ros::ServiceServer get_serial_server;

  /** \brief reconfigure server*/
  boost::shared_ptr<ReconfigureServer> reconfigure_server_;
  bool config_init_;

  boost::mutex connect_mutex_;
  // published topics
  image_transport::CameraPublisher pub_color_;
  image_transport::CameraPublisher pub_depth_;
  image_transport::CameraPublisher pub_point3d_;
  image_transport::CameraPublisher pub_ir_;
  ros::Publisher pub_projector_info_;

  /** \brief Camera info manager objects. */
  boost::shared_ptr<camera_info_manager::CameraInfoManager> color_info_manager_, depth_info_manager_, ir_info_manager;

  PercipioVideoMode ir_video_mode_;
  PercipioVideoMode color_video_mode_;
  PercipioVideoMode depth_video_mode_;

  std::string ir_frame_id_;
  std::string color_frame_id_;
  std::string depth_frame_id_ ;

  std::string color_info_url_, depth_info_url_, ir_info_url_;

  // dynamic reconfigure config
  int m_laser_power_;
  
  bool auto_exposure_;
  bool auto_white_balance_;
  
  int m_rgb_analog_gain_;
  int m_rgb_r_gain_;
  int m_rgb_g_gain_;
  int m_rgb_b_gain_;
  int m_rgb_exposure_time_;
  
  int m_ir_gain_;
  //-

  double depth_ir_offset_x_;
  double depth_ir_offset_y_;
  float z_scaling_;

  #define PARAMTER_DEFAULT   (0xFFFFFFFF)
  int tof_depth_channel_;
  int tof_depth_quality_;

  int sgbm_image_channel_num_  = PARAMTER_DEFAULT;
  int sgbm_disparity_num_  = PARAMTER_DEFAULT;
  int sgbm_disparity_offset_  = PARAMTER_DEFAULT;
  int sgbm_match_window_height_  = PARAMTER_DEFAULT;
  int sgbm_semi_global_param_p1_  = PARAMTER_DEFAULT;
  int sgbm_semi_global_param_p2_  = PARAMTER_DEFAULT;
  int sgbm_unique_factor_param_  = PARAMTER_DEFAULT;
  int sgbm_unique_min_absolute_diff_  = PARAMTER_DEFAULT;
  int sgbm_cost_param_  = PARAMTER_DEFAULT;
  int sgbm_enable_half_window_size_  = PARAMTER_DEFAULT;
  int sgbm_match_window_width_  = PARAMTER_DEFAULT;
  int sgbm_enable_median_filter_  = PARAMTER_DEFAULT;
  int sgbm_enable_lrc_check_  = PARAMTER_DEFAULT;
  int sgbm_lrc_max_diff_  = PARAMTER_DEFAULT;
  int sgbm_median_filter_thresh_  = PARAMTER_DEFAULT;
  int sgbm_semi_global_param_p1_scale_  = PARAMTER_DEFAULT;
  
  ros::Duration ir_time_offset_;
  ros::Duration color_time_offset_;
  ros::Duration depth_time_offset_;

  int data_skip_;

  bool use_device_time_;

  int data_skip_ir_counter_;
  int data_skip_color_counter_;
  int data_skip_depth_counter_;

  bool ir_subscribers_;
  bool color_subscribers_;
  bool point3d_subscribers_;
  bool depth_subscribers_;

  Config old_config_;
};

}


namespace percipio_camera
{

class PercipioDriverNodelet : public nodelet::Nodelet
{
public:
  PercipioDriverNodelet()  {};

  ~PercipioDriverNodelet() {}
  
private:
  virtual void onInit()
  {
    lp.reset(new percipio_wrapper::PercipioDriver(getNodeHandle(), getPrivateNodeHandle()));
  };

  boost::shared_ptr<percipio_wrapper::PercipioDriver> lp;
};

}


#endif
