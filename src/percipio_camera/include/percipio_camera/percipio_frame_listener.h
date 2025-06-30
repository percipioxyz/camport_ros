#ifndef PERCIPIO_FRAME_LISTENER_H_
#define PERCIPIO_FRAME_LISTENER_H_

#include "percipio_camera/percipio_device.h"

#include <sensor_msgs/Image.h>

#include <vector>

#include "TYApi.h"
#include "percipio_interface.h"

namespace percipio_wrapper
{

class PercipioTimerFilter;

class PercipioFrameListener : public percipio::VideoStream::NewFrameListener
{
public:
  PercipioFrameListener();

  virtual ~PercipioFrameListener()
  { };

  void onNewFrame(percipio::VideoStream& stream);

  void setCallback(FrameCallbackFunction& callback)
  {
    callback_ = callback;
  }

  void setUseDeviceTimer(bool enable);

private:
  percipio::VideoFrameRef m_frame;

  FrameCallbackFunction callback_;

  bool user_device_timer_;
  boost::shared_ptr<PercipioTimerFilter> timer_filter_;

  double prev_time_stamp_;
};

}

#endif
