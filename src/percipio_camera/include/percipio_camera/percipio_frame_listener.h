#ifndef PERCIPIO_FRAME_LISTENER_H_
#define PERCIPIO_FRAME_LISTENER_H_

#include "percipio_camera/percipio_device.h"

#include "TYApi.h"
#include "percipio_interface.h"

namespace percipio_wrapper
{

class PercipioFrameListener : public percipio::VideoStream::NewFrameListener
{
public:
  PercipioFrameListener(const std::string& desc);

  virtual ~PercipioFrameListener(){ }

  virtual void onNewFrame(percipio::VideoStream& stream);

  void setCallback(boost::shared_ptr<FrameCallbackFunction>& callback)
  {
    callback_ = callback;
  }

private:
  percipio::VideoFrameRef m_frame;

  boost::shared_ptr<FrameCallbackFunction> callback_;

  double prev_time_stamp_;

  const std::string stream_desc;
};

}

#endif
