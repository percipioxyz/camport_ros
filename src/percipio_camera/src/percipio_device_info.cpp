#include "percipio_camera/percipio_device_info.h"

namespace percipio_wrapper
{


std::ostream& operator << (std::ostream& stream, const PercipioDeviceInfo& device_info) {
  if(strlen(device_info.ip_.c_str())) {
    stream << "Uri: " << device_info.uri_ << " (Vendor: " << device_info.vendor_ <<
                                           ", Name: " << device_info.name_ <<
                                           ", IP: " << device_info.ip_ <<
                                             ")" << std::endl;
  } else {
    stream << "Uri: " << device_info.uri_ << " (Vendor: " << device_info.vendor_ <<
                                           ", Name: " << device_info.name_ <<
                                           ", Vendor ID: " << std::hex << device_info.vendor_id_ <<
                                           ", Product ID: " << std::hex << device_info.product_id_ <<
                                             ")" << std::endl;
  }
  return stream;
}



} //namespace percipio_wrapper
