#include "TYApi.h"

#include "percipio_camera/percipio_frame_listener.h"
#include "percipio_camera/percipio_timer_filter.h"

#include <sensor_msgs/image_encodings.h>

#include <ros/ros.h>

#define TIME_FILTER_LENGTH 15

namespace percipio_wrapper
{

PercipioFrameListener::PercipioFrameListener() :
    callback_(0),
    user_device_timer_(false),
    timer_filter_(new PercipioTimerFilter(TIME_FILTER_LENGTH)),
    prev_time_stamp_(0.0)
{
  ros::Time::init();
}

void PercipioFrameListener::setUseDeviceTimer(bool enable)
{
  user_device_timer_ = enable;

  if (user_device_timer_)
    timer_filter_->clear();
}

void PercipioFrameListener::onNewFrame(percipio::VideoStream& stream)
{
  if(TY_STATUS_OK != stream.readFrame(&m_frame)) {
    ROS_WARN("onNewFrame : readFrame end< invalid frame>!\n");
    return;
  }

  if (m_frame.isValid() && callback_)
  {
    sensor_msgs::ImagePtr image(new sensor_msgs::Image);

    ros::Time ros_now = ros::Time::now();

    if (!user_device_timer_)
    {
      image->header.stamp = ros_now;

      ROS_DEBUG("Time interval between frames: %.4f ms", (float)((ros_now.toSec()-prev_time_stamp_)*1000.0));

      prev_time_stamp_ = ros_now.toSec();
    } 
    else
    {
      uint64_t device_time = m_frame.getTimestamp();

      double device_time_in_sec = static_cast<double>(device_time)/1000000.0;
#if 0
      double ros_time_in_sec = ros_now.toSec();

      double time_diff = ros_time_in_sec-device_time_in_sec;

      timer_filter_->addSample(time_diff);

      double filtered_time_diff = timer_filter_->getMedian();

      double corrected_timestamp = device_time_in_sec+filtered_time_diff;
#else
      double corrected_timestamp = device_time_in_sec;
#endif

      image->header.stamp.fromSec(corrected_timestamp);

      ROS_DEBUG("Time interval between frames: %.4f ms", (float)((corrected_timestamp-prev_time_stamp_)*1000.0));
      prev_time_stamp_ = corrected_timestamp;
    }

    image->width = m_frame.getWidth();
    image->height = m_frame.getHeight();

    std::size_t data_size = m_frame.getDataSize();
    image->data.resize(data_size);
    memcpy(&image->data[0], m_frame.getData(), data_size);
    image->is_bigendian = 0;

    const percipio::VideoMode video_mode = m_frame.getVideoMode();
    switch (video_mode.getPixelFormat())
    {
      case TYPixelFormatCoord3D_C16:
        image->encoding = sensor_msgs::image_encodings::TYPE_16UC1;
        image->step = sizeof(unsigned char) * 2 * image->width;
        break;
      case TYPixelFormatRGB8:
        image->encoding = sensor_msgs::image_encodings::RGB8;
        image->step = sizeof(unsigned char) * 3 * image->width;
        break;
      case TYPixelFormatMono8:
        image->encoding = sensor_msgs::image_encodings::MONO8;
        image->step = sizeof(unsigned char) * 1 * image->width;
        break;
      case TYPixelFormatMono16:
        image->encoding = sensor_msgs::image_encodings::MONO16;
        image->step = sizeof(unsigned char) * 2 * image->width;
        break;
      case TYPixelFormatCoord3D_ABC16:
        image->encoding = sensor_msgs::image_encodings::TYPE_16UC3;
        image->step = sizeof(unsigned short) * 3 * image->width;
        break;
      default:
        ROS_ERROR("Invalid image encoding.");
        break;
    }
    callback_(image);
  }
}

}

