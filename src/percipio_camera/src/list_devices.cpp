/*
 * @Description: 
 * @Author: zxy
 * @Date: 2023-08-04 13:54:30
 * @LastEditors: zxy
 * @LastEditTime: 2023-08-14 17:48:02
 */
#include <iostream>
#include "percipio_camera/percipio_device_manager.h"
#include "percipio_camera/percipio_exception.h"

using percipio_wrapper::PercipioDeviceManager;
using percipio_wrapper::PercipioDeviceInfo;
using percipio_wrapper::PercipioException;

int main(int arc, char** argv)
{
  percipio_wrapper::PercipioDeviceManager manager;
  boost::shared_ptr<std::vector<percipio_wrapper::PercipioDeviceInfo> > device_infos = manager.getConnectedDeviceInfos();
  std::cout << "Found " << device_infos->size() << " devices:" << std::endl << std::endl;
  for (size_t i = 0; i < device_infos->size(); ++i)
  {
    std::cout << "Device #" << i << ":" << std::endl;
    std::cout << device_infos->at(i) << std::endl;
    try {
      std::string serial = manager.getSerial(device_infos->at(i).uri_);
      std::cout << "Serial number: " << serial << std::endl;
    }
    catch (const PercipioException& exception)
    {
      std::cerr << "Could not retrieve serial number: " << exception.what() << std::endl;
    }
  }
  return 0;
}

