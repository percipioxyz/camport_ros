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

#include "percipio_camera/percipio_convert.h"
#include "percipio_camera/percipio_exception.h"

#include <boost/make_shared.hpp>

#include <string>

namespace percipio_wrapper
{

const PercipioDeviceInfo percipio_convert(const percipio::DeviceInfo* pInfo)
{
  if (!pInfo)
    THROW_PERCIPIO_EXCEPTION("percipio_convert called with zero pointer\n");

  PercipioDeviceInfo output;

  output.name_       = pInfo->getName();
  output.uri_        = pInfo->getUri();
  output.vendor_     = pInfo->getVendor();
  output.ip_         = pInfo->getIp();
  output.product_id_ = pInfo->getUsbProductId();
  output.vendor_id_  = pInfo->getUsbVendorId();


  return output;
}


const PercipioVideoMode percipio_convert(const percipio::VideoMode& input)
{
  PercipioVideoMode output;

  output.x_resolution_ = input.getResolutionX();
  output.y_resolution_ = input.getResolutionY();
  output.frame_rate_ = input.getFps();
  output.pixel_format_ = static_cast<PixelFormat>(input.getPixelFormat());

  return output;
}

const percipio::VideoMode percipio_convert(const PercipioVideoMode& input)
{

  percipio::VideoMode output;

  output.setResolution(input.x_resolution_, input.y_resolution_);
  output.setFps(input.frame_rate_);
  output.setPixelFormat(static_cast<percipio::PixelFormat>(input.pixel_format_));

  return output;
}


const std::vector<PercipioVideoMode> percipio_convert(const percipio::Array<percipio::VideoMode>& input)
{
  std::vector<PercipioVideoMode> output;

  int size = input.getSize();

  output.reserve(size);

  for (int i=0; i<size; ++i) {
    output.push_back(percipio_convert(input[i]));
  }
  return output;
}

}


