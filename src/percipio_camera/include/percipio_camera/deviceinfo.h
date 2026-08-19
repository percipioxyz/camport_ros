/*
 * @Description: 
 * @Author: zxy
 * @Date: 2023-08-08 13:27:11
 * @LastEditors: zxy
 * @LastEditTime: 2024-02-18 17:18:51
 */
#ifndef _DEVICE_INFO_H_
#define _DEVICE_INFO_H_

#include "TYApi.h"
#include <string>

namespace percipio
{
  class DeviceInfo
  {
    public:
      DeviceInfo(){}
      DeviceInfo(const char* serial_num, const char* vendor_name, const char* modle_name, int pid, int vid)
      : uri(serial_num),
        vendor(vendor_name),
        name(modle_name),
        ip(std::string()),
        usbVendorId(vid),
        usbProductId(pid)
      {
      }

      DeviceInfo(const char* serial_num, const char* vendor_name, const char* modle_name, const char* ip_addr)
        : uri(serial_num),
          vendor(vendor_name),
          name(modle_name),
          ip(ip_addr),
          usbVendorId(0),
          usbProductId(0)
      {
      }

      DeviceInfo& operator=(const DeviceInfo& other) {
        if (this != &other) {
          uri = other.uri;
          vendor = other.vendor;
          name = other.name;
          ip = other.ip;
          usbVendorId = other.usbVendorId;
          usbProductId = other.usbProductId;
        }
        return *this;
      }

      const char* getUri() const { return uri.c_str(); }
      const char* getVendor() const { return vendor.c_str(); }
      const char* getIp() const { return ip.c_str(); }
      const char* getName() const { return name.c_str(); }
      uint16_t getUsbVendorId() const { return usbVendorId; }
      uint16_t getUsbProductId() const { return usbProductId; }

    private:
      std::string uri;
      std::string vendor;
      std::string name;
      std::string ip;
      uint16_t    usbVendorId = 0;
      uint16_t    usbProductId = 0;
  };
}
#endif