#ifndef PERCIPIO_DEVICE_MANAGER_H_
#define PERCIPIO_DEVICE_MANAGER_H_

#include "percipio_camera/percipio_device_info.h"

#include <boost/thread/mutex.hpp>

#include <vector>
#include <string>
#include <ostream>

namespace percipio_wrapper
{

class PercipioDeviceListener;
class PercipioDevice;

class PercipioDeviceManager
{
public:
  PercipioDeviceManager();
  virtual ~PercipioDeviceManager();

  static boost::shared_ptr<PercipioDeviceManager> getSingelton();

  int init_tycam_log_server(bool enable, const std::string& level, int32_t port);

  boost::shared_ptr<std::vector<PercipioDeviceInfo> > getConnectedDeviceInfos() const;
  boost::shared_ptr<std::vector<std::string> > getConnectedDeviceURIs() const;
  std::size_t getNumOfConnectedDevices() const;

  boost::shared_ptr<PercipioDevice> getAnyDevice();
  boost::shared_ptr<PercipioDevice> getDevice(const std::string& device_URI, const bool reconnection = false);
  bool setNetworkConfiguration(const std::string& device_URI, const std::string& ip, const std::string& netmask, const std::string& gateway);

  std::string getSerial(const std::string& device_URI) const;

protected:
  boost::shared_ptr<PercipioDeviceListener> device_listener_;

  static boost::shared_ptr<PercipioDeviceManager> singelton_;
};


std::ostream& operator <<(std::ostream& stream, const PercipioDeviceManager& device_manager);

}

#endif
