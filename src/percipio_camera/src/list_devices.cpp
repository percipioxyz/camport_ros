/*
 * @Description: 
 * @Author: zxy
 * @Date: 2023-08-04 13:54:30
 * @LastEditors: zxy
 * @LastEditTime: 2023-08-14 17:48:02
 */
#include <iostream>
#include <iomanip>
#include <string>
#include <vector>
#include <memory>
#include <sstream>

#include "percipio_camera/percipio_device_manager.h"
#include "percipio_camera/percipio_exception.h"

using percipio_wrapper::PercipioDeviceManager;
using percipio_wrapper::PercipioDeviceInfo;
using percipio_wrapper::PercipioException;

int main(int argc, char** argv)
{
  std::cout << "\n";
  std::cout << std::string(80, '=') << std::endl;
  std::cout << std::setw(40) << std::right << "PERCIPIO DEVICE ENUMERATION TOOL" << std::endl;
  std::cout << std::string(80, '=') << std::endl;
  
  // Command line arguments
  /*
  if (argc > 0) {
    std::cout << "\n[COMMAND LINE ARGUMENTS]" << std::endl;
    std::cout << std::string(40, '-') << std::endl;
    for(int i = 0; i < argc; i++) {
      std::cout << "  " << std::setw(2) << i << ": " << argv[i] << std::endl;
    }
  }
  */
  std::cout << "\n[DEVICE SCAN]" << std::endl;
  std::cout << std::string(40, '-') << std::endl;
  
  struct DeviceInfo {
    size_t index;
    std::string status;
    std::string status_symbol;
    std::string uri;
    std::string serial;
    std::string details;
  };
  
  std::vector<DeviceInfo> device_list;
  
  percipio_wrapper::PercipioDeviceManager manager;
  auto device_infos = manager.getConnectedDeviceInfos();
  
  if (device_infos->empty()) {
    std::cout << "  Status: No devices found\n" << std::endl;
    return 0;
  }
  
  std::cout << "  Scanning devices..." << std::endl;
  std::cout << std::string(60, '-') << std::endl;
  
  for (size_t i = 0; i < device_infos->size(); ++i) {
    const auto& device = device_infos->at(i);
    DeviceInfo info;
    info.index = i + 1;
    info.uri = device.uri_;
    
    std::stringstream details_ss;
    details_ss << device;
    info.details = details_ss.str();
    
    try {
      info.serial = manager.getSerial(device.uri_);
      info.status = "Online";
      info.status_symbol = "[OK]";
    }
    catch (const PercipioException& e) {
      info.serial = "Failed: " + std::string(e.what());
      info.status = "Error";
      info.status_symbol = "[ERR]";
    }
    
    device_list.push_back(info);
  }
  
  std::cout << std::string(60, '-') << std::endl;
  std::cout << "\n[DEVICE LIST - " << device_list.size() << " DEVICES]" << std::endl;
  std::cout << std::string(60, '=') << std::endl;
  
  std::cout << "\n" << std::left
            << std::setw(6) << "No."
            << std::setw(10) << "Status"
            << std::setw(45) << "URI"
            << std::setw(20) << "Serial" 
            << std::endl;
  
  std::cout << std::string(90, '-') << std::endl;
  
  for (const auto& device : device_list) {
    std::string display_uri = device.uri;
    if (display_uri.length() > 42) {
      display_uri = display_uri.substr(0, 39) + "...";
    }
    
    std::string display_serial = device.serial;
    if (display_serial.length() > 17) {
      display_serial = display_serial.substr(0, 14) + "...";
    }
    
    std::cout << std::left
              << std::setw(6) << device.index
              << std::setw(10) << device.status_symbol
              << std::setw(45) << display_uri
              << std::setw(20) << display_serial
              << std::endl;
  }
  
  std::cout << "\n[DETAILED INFORMATION]" << std::endl;
  std::cout << std::string(60, '-') << std::endl;
  
  for (const auto& device : device_list) {
    std::cout << "\nDevice #" << device.index << ":" << std::endl;
    std::cout << "  URI: " << device.uri << std::endl;
    std::cout << "  Status: " << device.status_symbol << " " << device.status << std::endl;
    std::cout << "  Serial: " << device.serial << std::endl;
    
    if (!device.details.empty()) {
      std::cout << "  Details:" << std::endl;
      std::istringstream details_stream(device.details);
      std::string line;
      while (std::getline(details_stream, line)) {
        std::cout << "    " << line << std::endl;
      }
    }
    
    std::cout << std::string(40, '-') << std::endl;
  }
  
  std::cout << "\n[SUMMARY]" << std::endl;
  std::cout << std::string(60, '-') << std::endl;
  
  int success_count = 0;
  int error_count = 0;
  
  for (const auto& device : device_list) {
    if (device.status_symbol == "[OK]") {
      success_count++;
    } else if (device.status_symbol == "[ERR]") {
      error_count++;
    }
  }
  
  std::cout << "• Total devices: " << device_list.size() << std::endl;
  std::cout << "• Successful: " << success_count << std::endl;
  std::cout << "• With errors: " << error_count << std::endl;
  std::cout << "• Time: " << __TIME__ << std::endl;
  std::cout << std::string(80, '=') << std::endl;
  
  return 0;
}