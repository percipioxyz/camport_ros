#ifndef PERCIPIO_DEVICE_INFO_H_
#define PERCIPIO_DEVICE_INFO_H_

#include <ostream>
#include <string.h>
#include <boost/cstdint.hpp>

namespace percipio_wrapper
{

struct PercipioDeviceInfo
{
  std::string uri_;
  std::string vendor_;
  std::string name_;
  std::string ip_;
  uint16_t vendor_id_;
  uint16_t product_id_;
};

std::ostream& operator << (std::ostream& stream, const PercipioDeviceInfo& device_info);

}

#endif /* DRIVER_H_ */
