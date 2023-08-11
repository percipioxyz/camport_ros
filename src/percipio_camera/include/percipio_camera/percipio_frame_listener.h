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

  void setPercipioDeviceTimer(bool enable);

private:
  percipio::VideoFrameRef m_frame;

  FrameCallbackFunction callback_;

  bool user_device_timer_;
  boost::shared_ptr<PercipioTimerFilter> timer_filter_;

  double prev_time_stamp_;
};

}

#endif
