#ifndef PERCIPIO_DRIVER_H
#define PERCIPIO_DRIVER_H

#include <boost/shared_ptr.hpp>
#include <boost/cstdint.hpp>
#include <boost/bind.hpp>
#include <boost/function.hpp>

#include <sensor_msgs/Image.h>
#include <std_msgs/Empty.h>
#include <std_msgs/String.h>

#include <dynamic_reconfigure/server.h>
#include <percipio_camera/PercipioConfig.h>

#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>

#include <geometry_msgs/TransformStamped.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>

#include <string>
#include <vector>

#include "percipio_camera/percipio_device_manager.h"
#include "percipio_camera/percipio_device.h"
#include "percipio_camera/percipio_video_mode.h"
#include "percipio_camera/GetSerial.h"

#include <ros/ros.h>

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
  void newPoint3DFrameCallback(sensor_msgs::ImagePtr image);
  
  void SoftTriggerCb(const std_msgs::EmptyConstPtr& msg);
  void ResetCb(const std_msgs::EmptyConstPtr& msg);
  void DynamicConfigureCb(const std_msgs::String::ConstPtr& msg);

  // Methods to get calibration parameters for the various cameras
  sensor_msgs::CameraInfoPtr getDefaultCameraInfo(int width, int height, double f);
  sensor_msgs::CameraInfoPtr getColorCameraInfo(int width, int height, ros::Time time, bool isColor);
  sensor_msgs::CameraInfoPtr getDepthCameraInfo(int width, int height, ros::Time time);

  void readConfigFromParameterServer();

  // resolves non-URI device IDs to URIs, e.g. '#1' is resolved to the URI of the first device
  std::string resolveDeviceURI(const std::string& device_id);
  bool resolveDeviceResolution(const std::string& resolution_, int& width, int& height);
  void setupDevice();
  void initDevice();
  void DeviceEvent(const char* event);

  void advertiseROSTopics();

  void colorConnectCb();
  void depthConnectCb();
  void irConnectCb();
  void cloud3DConnectCb();

  bool getSerialCb(percipio_camera::GetSerialRequest& req, percipio_camera::GetSerialResponse& res);

  void configCb(Config &config, uint32_t level);

  void applyConfigToPercipioDevice();

  sensor_msgs::ImageConstPtr rawToFloatingPointConversion(sensor_msgs::ImageConstPtr raw_image, float scale);

  ros::NodeHandle& nh_;
  ros::NodeHandle& pnh_;

  boost::shared_ptr<PercipioDeviceManager> device_manager_;
  boost::shared_ptr<PercipioDevice> device_;

  boost::shared_ptr<FrameCallbackFunction> color_frame_callback;
  boost::shared_ptr<FrameCallbackFunction> depth_frame_callback;
  boost::shared_ptr<FrameCallbackFunction> p3d_frame_callback;
  boost::shared_ptr<FrameCallbackFunction> ir_frame_callback;
  
  std::string camera_name_;
  std::string device_id_;
  std::string rgb_resolution_;
  std::string depth_resolution_;
  std::string rgb_format_;
  std::string depth_format_;

  bool frame_rate_control_;
  float frame_rate_;
  
  bool trigger_mode_;
  std::string soft_trigger_topic_;

  std::string device_reset_topic_;

  std::string dynamic_configure_topic_;

  bool device_log_enable_;
  std::string device_log_level_;
  int device_log_server_port_;

  bool reconnection_flag_;

  bool color_undistortion_;

  bool depth_registration_;

  bool ir_undistortion_;

  /** \brief get_serial server*/
  ros::ServiceServer get_serial_server;

  /** \brief reconfigure server*/
  boost::shared_ptr<ReconfigureServer> reconfigure_server_;
  bool config_init_;

  boost::mutex connect_mutex_;
  // published topics
  image_transport::CameraPublisher pub_color_;
  image_transport::CameraPublisher pub_depth_;
  image_transport::CameraPublisher pub_ir_;
  
  sensor_msgs::PointCloud2 pub_point3d_cloud;
  ros::Publisher pub_point3d_;

  ros::Publisher device_event_pub_;

  std::vector<geometry_msgs::TransformStamped> static_tf_msgs_;
  tf2_ros::StaticTransformBroadcaster static_tf_broadcaster_;
  
  ros::Subscriber soft_trigger_sub_;
  ros::Subscriber device_reset_sub_;
  ros::Subscriber device_dynamic_sub_;

  /** \brief Camera info manager objects. */
  boost::shared_ptr<camera_info_manager::CameraInfoManager> color_info_manager_, depth_info_manager_, ir_info_manager;

  std::string ir_frame_id_;
  std::string color_frame_id_;
  std::string depth_frame_id_ ;

  std::string color_info_url_, depth_info_url_, ir_info_url_;

  //
  bool depth_speckle_filter_;
  int max_speckle_size_;
  int max_speckle_diff_;
  float max_physical_size_;

  bool depth_time_domain_filter_;
  int  depth_time_domain_num_;

  percipio::IREnhanceModel ir_enhancement_;
  int ir_enhancement_coefficient_;

  void publishStaticTF(const ros::Time &t, const tf2::Vector3 &trans,
    const tf2::Quaternion &q, const std::string &from,
    const std::string &to);
  void publishStaticTransforms();

  bool use_device_time_;

  bool ir_subscribers_;
  bool color_subscribers_;
  bool point3d_subscribers_;
  bool depth_subscribers_;

  std::string xml_content;
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