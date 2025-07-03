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

static struct calib_data depth_cal, color_cal;

PercipioDriver::PercipioDriver(ros::NodeHandle& n, ros::NodeHandle& pnh) :
  nh_(n),
  pnh_(pnh),
  device_manager_(PercipioDeviceManager::getSingelton()),
  config_init_(false),
  depth_registration_(false),
  use_device_time_(false),

  ir_subscribers_(false),
  color_subscribers_(false),
  point3d_subscribers_(false),
  depth_subscribers_(false)
{
  readConfigFromParameterServer();

  initDevice();

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

void PercipioDriver::publishStaticTF(const ros::Time &t, const tf2::Vector3 &trans,
  const tf2::Quaternion &q, const std::string &from,
  const std::string &to) 
{
  geometry_msgs::TransformStamped msg;
  msg.header.stamp = t;
  msg.header.frame_id = from;
  msg.child_frame_id = to;
  msg.transform.translation.x = trans[2] / 1000.0;
  msg.transform.translation.y = -trans[0] / 1000.0;
  msg.transform.translation.z = -trans[1] / 1000.0;
  msg.transform.rotation.x = q.getX();
  msg.transform.rotation.y = q.getY();
  msg.transform.rotation.z = q.getZ();
  msg.transform.rotation.w = q.getW();

  static_tf_msgs_.push_back(msg);
}

void PercipioDriver::publishStaticTransforms() {
  tf2::Vector3 trans(0.0, 0.0, 0.0);
  tf2::Quaternion Q = tf2::Quaternion(0.0, 0.0, 0.0, 1.0);

  tf2::Quaternion quaternion_optical;
  quaternion_optical.setRPY(-M_PI / 2, 0.0, -M_PI / 2);
  tf2::Vector3 zero_trans(0.0, 0.0, 0.0);
  
  std::string stream_name[] = {"depth", "rgb", "ir"};
  for(size_t i = 0; i < sizeof(stream_name) / sizeof(std::string); i++) {
    auto timestamp = ros::Time::now();//node_->now();
    
    std::string camera_link_frame_id_ = "camera_link"; //camera_name_ + "_link";
    std::string frame_id_ = "camera_" + stream_name[i] + "_frame";//camera_name_ + "_" + stream_name[index] + "_frame";
    std::string optical_frame_id_ = "camera_" + stream_name[i] + "_optical_frame";//camera_name_ + "_" + stream_name[index] + "_optical_frame";
    publishStaticTF(timestamp, trans,      Q,                  camera_link_frame_id_,   frame_id_);
    publishStaticTF(timestamp, zero_trans, quaternion_optical, frame_id_,               optical_frame_id_);
  }
  static_tf_broadcaster_.sendTransform(static_tf_msgs_);
}


void PercipioDriver::advertiseROSTopics()
{
  publishStaticTransforms();

  // Allow remapping namespaces rgb, ir, depth, depth_registered
  ros::NodeHandle color_nh(nh_, "rgb");
  image_transport::ImageTransport color_it(color_nh);
  
  ros::NodeHandle ir_nh(nh_, "ir");
  image_transport::ImageTransport ir_it(ir_nh);
  
  ros::NodeHandle depth_nh(nh_, "depth");
  image_transport::ImageTransport depth_it(depth_nh);

  // Advertise all published topics
  // Prevent connection callbacks from executing until we've set all the publishers. Otherwise
  // connectCb() can fire while we're advertising (say) "depth/image", but before we actually
  // assign to pub_depth_raw_. Then pub_depth_raw_.getNumSubscribers() returns 0, and we fail to start
  // the depth generator.
  boost::lock_guard<boost::mutex> lock(connect_mutex_);

  // Asus Xtion PRO does not have an RGB camera
  if (device_->hasColorSensor())
  {
    image_transport::SubscriberStatusCallback itssc = boost::bind(&PercipioDriver::colorConnectCb, this);
    ros::SubscriberStatusCallback rssc = boost::bind(&PercipioDriver::colorConnectCb, this);
    pub_color_ = color_it.advertiseCamera("image", 1, itssc, itssc, rssc, rssc);
  } else {
    ROS_WARN("The device lacks an RGB component and does not support RGBD alignment mode!\n");
  }

  if (device_->hasIRSensor())
  {
    image_transport::SubscriberStatusCallback itssc = boost::bind(&PercipioDriver::irConnectCb, this);
    ros::SubscriberStatusCallback rssc = boost::bind(&PercipioDriver::irConnectCb, this);
    pub_ir_ = ir_it.advertiseCamera("image", 1, itssc, itssc, rssc, rssc);
  }

  if (device_->hasDepthSensor())
  {
    image_transport::SubscriberStatusCallback itssc = boost::bind(&PercipioDriver::depthConnectCb, this);
    ros::SubscriberStatusCallback rssc = boost::bind(&PercipioDriver::depthConnectCb, this);
    pub_depth_ = depth_it.advertiseCamera("image", 1, itssc, itssc, rssc, rssc);

    const ros::SubscriberStatusCallback& rssc2 = boost::bind(&PercipioDriver::cloud3DConnectCb, this);
    pub_point3d_ = nh_.advertise<sensor_msgs::PointCloud2>("PointCloud2", 1, rssc2, rssc2);
  }

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
  use_device_time_ = config.use_device_time;

  applyConfigToPercipioDevice();

  config_init_ = true;
}

void PercipioDriver::applyConfigToPercipioDevice()
{
  device_->setUseDeviceTimer(use_device_time_);
}

void PercipioDriver::colorConnectCb()
{
  boost::lock_guard<boost::mutex> lock(connect_mutex_);
  color_subscribers_ = pub_color_.getNumSubscribers() > 0;
  if (color_subscribers_ && !device_->isColorStreamStarted())
  { 
    color_frame_callback = boost::make_shared<FrameCallbackFunction>(boost::bind(&PercipioDriver::newColorFrameCallback, this, _1));
    device_->setColorFrameCallback(color_frame_callback);

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
  depth_subscribers_ = pub_depth_.getNumSubscribers() > 0;
  bool need_depth = depth_subscribers_;
  if (need_depth && !device_->isDepthStreamStarted())
  {
    depth_frame_callback = boost::make_shared<FrameCallbackFunction>(boost::bind(&PercipioDriver::newDepthFrameCallback, this, _1));
    device_->setDepthFrameCallback(depth_frame_callback);

    ROS_INFO("Starting depth stream.");
    device_->startDepthStream();
  }
  else if (!need_depth && device_->isDepthStreamStarted())
  {
    ROS_INFO("Stopping depth stream.");
    device_->stopDepthStream();
  }
}

void PercipioDriver::cloud3DConnectCb()
{
  boost::lock_guard<boost::mutex> lock(connect_mutex_);
  point3d_subscribers_ = pub_point3d_.getNumSubscribers() > 0;
  bool need_p3d = point3d_subscribers_;
  if (need_p3d && !device_->isPoint3DStreamStarted())
  {
    p3d_frame_callback = boost::make_shared<FrameCallbackFunction>(boost::bind(&PercipioDriver::newPoint3DFrameCallback, this, _1));
    device_->setPoint3DFrameCallback(p3d_frame_callback);

    ROS_INFO("Starting point3d stream.");
    device_->startPoint3DStream();
  }
  else if (!need_p3d && device_->isPoint3DStreamStarted())
  {
    ROS_INFO("Stopping depth stream.");
    device_->stopPoint3DStream();
  }
}

void PercipioDriver::irConnectCb()
{
  boost::lock_guard<boost::mutex> lock(connect_mutex_);
  ir_subscribers_ = pub_ir_.getNumSubscribers() > 0;

  if (ir_subscribers_ && !device_->isIRStreamStarted())
  {
    ir_frame_callback = boost::make_shared<FrameCallbackFunction>(boost::bind(&PercipioDriver::newIRFrameCallback, this, _1));
    device_->setIRFrameCallback(ir_frame_callback);

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
  if (ir_subscribers_)
  {
    image->header.frame_id = ir_frame_id_;
    image->header.stamp = image->header.stamp;
    
    pub_ir_.publish(image, getDepthCameraInfo(image->width, image->height, image->header.stamp));
  }
}

void PercipioDriver::newColorFrameCallback(sensor_msgs::ImagePtr image)
{
  if (color_subscribers_)
  {
    image->header.frame_id = color_frame_id_;
    image->header.stamp = image->header.stamp;
    pub_color_.publish(image, getColorCameraInfo(image->width, image->height, image->header.stamp, true));
  }
}

void PercipioDriver::newDepthFrameCallback(sensor_msgs::ImagePtr image)
{
  if (depth_subscribers_)
  {
    sensor_msgs::CameraInfoPtr cam_info;
    if(percipio::IMAGE_REGISTRATION_DEPTH_TO_COLOR == device_->getImageRegistrationMode())
    {
      image->header.frame_id = color_frame_id_;
      cam_info = getColorCameraInfo(image->width,image->height, image->header.stamp, false);
    }
    else
    {
      image->header.frame_id = depth_frame_id_;
      cam_info = getDepthCameraInfo(image->width,image->height, image->header.stamp);
    }

    pub_depth_.publish(image, cam_info);
  }
}

void PercipioDriver::newPoint3DFrameCallback(sensor_msgs::ImagePtr image)
{
  if (point3d_subscribers_)
  {
    if(image->encoding == sensor_msgs::image_encodings::TYPE_16UC3)
    {
      //TODO
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr  point_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);

      float f_scale_unit = device_->getDepthScale();
      int16_t* data = reinterpret_cast<int16_t*>(&image->data[0]);
      for(int i = 0; i < image->width * image->height; i++) {
        if(data[3*i] != 0) {
          pcl::PointXYZRGB  p;
          p.x = f_scale_unit * data[3*i] / 1000.f;
          p.y = f_scale_unit * data[3*i + 1] / 1000.f;
          p.z = f_scale_unit * data[3*i + 2] / 1000.f;
          p.r = 255;
          p.g = 255;
          p.b = 255;
          point_cloud->points.push_back(p);
        }
      }

      point_cloud->width = 1;
      point_cloud->height = point_cloud->points.size();
    
      pcl::toROSMsg(*point_cloud, pub_point3d_cloud);
      pub_point3d_cloud.header.frame_id = depth_frame_id_;
      pub_point3d_cloud.header.stamp = image->header.stamp;

      pub_point3d_.publish(pub_point3d_cloud);
      ros::spinOnce();
    } 
    else if(image->encoding == sensor_msgs::image_encodings::TYPE_16UC1)
    {
      sensor_msgs::CameraInfoPtr cam_info;
      if(percipio::IMAGE_REGISTRATION_DEPTH_TO_COLOR == device_->getImageRegistrationMode())
      {
        image->header.frame_id = color_frame_id_;
        cam_info = getColorCameraInfo(image->width,image->height, image->header.stamp, false);
      }
      else
      {
        image->header.frame_id = depth_frame_id_;
        cam_info = getDepthCameraInfo(image->width,image->height, image->header.stamp);
      }

      pcl::PointCloud<pcl::PointXYZRGB>::Ptr  point_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
      float f_scale_unit = device_->getDepthScale();
      uint16_t* data = reinterpret_cast<uint16_t*>(&image->data[0]);
      float fx = cam_info->K[0]; //fx
      float fy = cam_info->K[4]; //fy
      float cx = cam_info->K[2]; //cx
      float cy = cam_info->K[5]; //cy

      for(int i = 0; i < image->width * image->height; i++) {
        int m_pix_x = i % image->width;
        int m_pix_y = i / image->width;
        if(data[i] != 0) {
          pcl::PointXYZRGB  p;
          p.x = f_scale_unit * (m_pix_x - cx) * data[i] / (1000.f * fx);
          p.y = f_scale_unit * (m_pix_y - cy) * data[i] / (1000.f * fy);
          p.z = f_scale_unit * data[i] / 1000.f;
          p.r = 255;
          p.g = 255;
          p.b = 255;
          point_cloud->points.push_back(p);
        }
      }

      point_cloud->width = 1;
      point_cloud->height = point_cloud->points.size();
    
      pcl::toROSMsg(*point_cloud, pub_point3d_cloud);
      pub_point3d_cloud.header.frame_id = image->header.frame_id;
      pub_point3d_cloud.header.stamp = image->header.stamp;

      pub_point3d_.publish(pub_point3d_cloud);
      ros::spinOnce();
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
  sensor_msgs::CameraInfoPtr info = boost::make_shared<sensor_msgs::CameraInfo>();
  if (device_->getVendor() == "Percipio")
  {
    size_t size = sizeof(color_cal.intrinsic_data);

    //get intristic from percipio firmware
    device_->getColorCalibIntristic((void*)color_cal.intrinsic_data, size);
    
    info->width  = width;
    info->height = height;
    
    info->D.resize(12, 0.0);
    //depth stream may need color camera info,but without distortion while map to color coordinate
    if(isColor && !color_undistortion_) {
      size = sizeof(color_cal.distortion_data);
      device_->getColorCalibDistortion((void*)color_cal.distortion_data, size);
      for(int i = 0; i < 12; i++) {
        info->D[i] = color_cal.distortion_data[i];
      }
    }
    info->distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;

    // Simple camera matrix: square pixels (fx = fy), principal point at center
    info->K.assign(0.0);
    info->K[0] = color_cal.intrinsic_data[0];
    info->K[2] = color_cal.intrinsic_data[2];
    info->K[4] = color_cal.intrinsic_data[4];
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
  }

  // Fill in header
  info->header.stamp  = time;
  info->header.frame_id = color_frame_id_;

  return info;
}

sensor_msgs::CameraInfoPtr PercipioDriver::getDepthCameraInfo(int width, int height, ros::Time time)
{
  sensor_msgs::CameraInfoPtr info = boost::make_shared<sensor_msgs::CameraInfo>();
  if (device_->getVendor() == "Percipio")
  {
    size_t size = sizeof(depth_cal.intrinsic_data);
    
    //get intristic from percipio firmware
    device_->getDepthCalibIntristic((void*)&depth_cal.intrinsic_data, size);
    
    info->width  = width;
    info->height = height;
    
    // No distortion
    info->D.resize(12, 0.0);
    info->distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;

    // Simple camera matrix: square pixels (fx = fy), principal point at center
    info->K.assign(0.0);
    info->K[0] = depth_cal.intrinsic_data[0];
    info->K[2] = depth_cal.intrinsic_data[2];
    info->K[4] = depth_cal.intrinsic_data[4];
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
  }

  return info;
}

void PercipioDriver::readConfigFromParameterServer()
{
  if (!pnh_.getParam("device_id", device_id_))
  {
    ROS_WARN ("~device_id is not set! Using first device.");
    device_id_ = "#1";
  }

  if (!pnh_.getParam("device_reconnection", reconnection_flag_))
  {
    ROS_WARN ("~reconnection flag is not set! Using default.");
    reconnection_flag_ = false;
  }

  if (!pnh_.getParam("rgb_resolution", rgb_resolution_))
  {
    ROS_WARN ("~rgb_resolution is not set! Try using default.");
    rgb_resolution_ = "640x480";
  }

  if (!pnh_.getParam("rgb_format", rgb_format_))
  {
    ROS_WARN ("~rgb_format is not set! Try using default.");
    rgb_format_ = "";
  }

  if (!pnh_.getParam("depth_resolution", depth_resolution_))
  {
    ROS_WARN ("~depth_resolution is not set! Try using default.");
    depth_resolution_ = "640x480";
  }

  if (!pnh_.getParam("depth_format", depth_format_))
  {
    depth_format_ = "";
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

  pnh_.getParam("depth_speckle_filter",             depth_speckle_filter_);
  pnh_.getParam("max_speckle_size",                 max_speckle_size_);
  pnh_.getParam("max_speckle_diff",                 max_speckle_diff_);

  pnh_.getParam("depth_time_domain_filter",         depth_time_domain_filter_);
  pnh_.getParam("depth_time_domain_num",            depth_time_domain_num_);

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
  if(0 == device_id.size() && available_device_URIs->size()) {
    return available_device_URIs->at(0);
  }
  else if (device_id.size() > 1 && device_id[0] == '#')
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

void PercipioDriver::cfgDevice()
{
  int rgb_width, rgb_height;
  int depth_width, depth_height;
  if(resolveDeviceResolution(rgb_resolution_, rgb_width, rgb_height)) {
    device_->setColorResolution(rgb_width, rgb_height, rgb_format_);
  }

  if(resolveDeviceResolution(depth_resolution_, depth_width, depth_height)) {
    device_->setDepthResolutuon(depth_width, depth_height, depth_format_);
  }
  //do rgb undistortion
  device_->setColorUndistortion(color_undistortion_);

  //Enabling the alignment function requires concurrent support for color and depth stream output.
  if(device_->isImageRegistrationModeSupported()) {
    device_->setImageRegistrationMode(depth_registration_);
  }

  device_->setDepthSpecFilterEn(depth_speckle_filter_);
  device_->setDepthSpecFilterSpecSize(max_speckle_size_);
  device_->setDepthSpeckFilterDiff(max_speckle_diff_);

  device_->setDepthTimeDomainFilterEn(depth_time_domain_filter_);
  device_->setDepthTimeDomainFilterNum(depth_time_domain_num_);
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

      device_ = device_manager_->getDevice(device_URI, reconnection_flag_);

      cfgDevice();

      auto cb = boost::make_shared<percipio::DeviceCfgCallbackFunction>(boost::bind(&PercipioDriver::cfgDevice, this));
      device_->setDeviceCfgInitCallback(cb);
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
