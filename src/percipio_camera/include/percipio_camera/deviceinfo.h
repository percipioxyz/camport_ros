/*
 * @Description: 
 * @Author: zxy
 * @Date: 2023-08-08 13:27:11
 * @LastEditors: zxy
 * @LastEditTime: 2023-08-09 10:21:29
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
      DeviceInfo(std::string sn, std::string vd, std::string na)
      {
        strcpy(uri, sn.c_str());
        strcpy(vendor, vd.c_str());
        strcpy(name, na.c_str());
      }
      const char* getUri() const { return uri; }
      const char* getVendor() const { return vendor; }
      const char* getName() const { return name; }
      uint16_t getUsbVendorId() const { return usbVendorId; }
      uint16_t getUsbProductId() const { return usbProductId; }

      void setUri(char* sn) {strcpy(uri, sn); }
      void setVendor(char* v) { strcpy(vendor, v); }
      void setName(char* n) { strcpy(name, n);}
      void setUsbVendorId(uint16_t v) { usbVendorId = v;}
      void setUsbProductId(uint16_t p) { usbProductId = p;}

    private:
      char uri[64];
      char vendor[64];
      char name[64];
      uint16_t    usbVendorId  = 0;
      uint16_t    usbProductId = 0;
  };
}
#endif