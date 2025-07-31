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


