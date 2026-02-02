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

#include "percipio_camera/Utils.hpp"

std::ostream& operator<<(std::ostream& os, const TY_DEVICE_BASE_INFO& dev) {
    std::stringstream ss;
    ss << "Interface Name: " << dev.iface.name << std::endl;
    ss << "Interface ID: " << dev.iface.id << std::endl;
    ss << "Interface Type: " << (TYIsNetworkInterface(dev.iface.type) ? "Network" : "USB") << std::endl;
    ss << "Device ID: " << dev.id << std::endl;
    ss << "Vendor Name: " << dev.vendorName << std::endl;
    ss << "User Defined Name: " << dev.userDefinedName << std::endl;
    ss << "Model Name: " << dev.modelName << std::endl;
    ss << "Hardware Version: " << dev.hardwareVersion.major << "." 
       << dev.hardwareVersion.minor << "." 
       << dev.hardwareVersion.patch << std::endl;
    ss << "Firmware Version: " << dev.firmwareVersion.major << "." 
       << dev.firmwareVersion.minor << "." 
       << dev.firmwareVersion.patch << std::endl;
    
    if (TYIsNetworkInterface(dev.iface.type)) {
        ss << "Network Info:" << std::endl;
        ss << "  MAC: " << dev.netInfo.mac << std::endl;
        ss << "  IP: " << dev.netInfo.ip << std::endl;
        ss << "  Netmask: " << dev.netInfo.netmask << std::endl;
        ss << "  Gateway: " << dev.netInfo.gateway << std::endl;
        ss << "  Broadcast: " << dev.netInfo.broadcast << std::endl;
        ss << "  TL Version: " << dev.netInfo.tlversion << std::endl;
    } else {
        ss << "USB Info:" << std::endl;
        ss << "  Bus: " << dev.usbInfo.bus << std::endl;
        ss << "  Address: " << dev.usbInfo.addr << std::endl;
        ss << "  TL Version: " << dev.usbInfo.tlversion << std::endl;
    }
    
    ss << "Build Hash: " << dev.buildHash << std::endl;
    ss << "Config Version: " << dev.configVersion << std::endl;
    
    os << ss.str();
    return os;
}

int main() {
    std::cout << "\n";
    std::cout << std::string(80, '=') << std::endl;
    std::cout << std::setw(40) << std::right << "PERCIPIO DEVICE ENUMERATION TOOL" << std::endl;
    std::cout << std::string(80, '=') << std::endl;
  
    std::cout << "\n[DEVICE SCAN]" << std::endl;
    std::cout << std::string(40, '-') << std::endl;
  
    ASSERT_OK(TYInitLib());
    TY_VERSION_INFO ver;
    ASSERT_OK(TYLibVersion(&ver));

    TYSetLogLevel(TY_LOG_TYPE_STD, TY_LOG_LEVEL_NEVER);

    ASSERT_OK(TYUpdateInterfaceList());

    std::cout << "  Scanning devices..." << std::endl;
    std::cout << std::string(60, '-') << std::endl;

    uint32_t n = 0;
    ASSERT_OK(TYGetInterfaceNumber(&n));
    if (n == 0) {
        std::cout << "  Status: No interfaces found\n" << std::endl;
        ASSERT_OK(TYDeinitLib());
        return TY_STATUS_ERROR;
    }

    std::vector<TY_DEVICE_BASE_INFO> devices;

    std::vector<TY_INTERFACE_INFO> ifaces(n);
    ASSERT_OK(TYGetInterfaceList(&ifaces[0], n, &n));
    ASSERT(n == ifaces.size());
    std::vector<TY_INTERFACE_HANDLE> hIfaces;
    for (uint32_t i = 0; i < n; i++) {
        TY_INTERFACE_HANDLE hIface;
        ASSERT_OK(TYOpenInterface(ifaces[i].id, &hIface));
        hIfaces.push_back(hIface);
    }
    updateDevicesParallel(hIfaces);
    for (uint32_t i = 0; i < n; i++) {
        TY_INTERFACE_HANDLE hIface = hIfaces[i];

        uint32_t n_devices = 0;
        TYGetDeviceNumber(hIface, &n_devices);
        if (n_devices == 0) continue;

        std::vector<TY_DEVICE_BASE_INFO> devs(n_devices);
        TYGetDeviceList(hIface, &devs[0], n_devices, &n_devices);
        for (uint32_t j = 0; j < n_devices; j++) {
            if (TYIsNetworkInterface(devs[j].iface.type)) {
                devices.push_back(devs[j]);
            } else {
                TY_DEV_HANDLE handle;
                int32_t ret = TYOpenDevice(hIface, devs[j].id, &handle);
                if (ret == 0) {
                    TYGetDeviceInfo(handle, &devs[j]);
                    TYCloseDevice(handle);
                }
                devices.push_back(devs[j]);
            }
        }
        TYCloseInterface(hIface);
    }
    
    std::cout << std::string(60, '-') << std::endl;
    
    if (devices.empty()) {
        std::cout << "  Status: No devices found\n" << std::endl;
        ASSERT_OK(TYDeinitLib());
        return 0;
    }
    
    struct DeviceInfo {
        size_t index;
        std::string status;
        std::string status_symbol;
        std::string uri;
        std::string serial;
        std::string type;
        std::string vendor;
        std::string model;
        std::string details;
    };
    
    std::vector<DeviceInfo> device_list;
    
    for (size_t i = 0; i < devices.size(); ++i) {
        const auto& device = devices[i];
        DeviceInfo info;
        info.index = i + 1;
        info.serial = device.id;
        info.vendor = device.vendorName;
        info.model = device.modelName;
        
        // Build URI
        if (TYIsNetworkInterface(device.iface.type)) {
            info.uri = "net://" + std::string(device.netInfo.ip);
            info.type = "Network";
        } else {
            info.uri = "usb://" + std::to_string(device.usbInfo.bus) + 
                      "-" + std::to_string(device.usbInfo.addr);
            info.type = "USB";
        }
        
        info.status = "Online";
        info.status_symbol = "[OK]";
        
        std::stringstream details_ss;
        details_ss << device;
        info.details = details_ss.str();
        
        device_list.push_back(info);
    }
    
    std::cout << "\n[DEVICE LIST - " << device_list.size() << " DEVICES]" << std::endl;
    std::cout << std::string(60, '=') << std::endl;
    
    std::cout << "\n" << std::left
              << std::setw(6) << "No."
              << std::setw(10) << "Status"
              << std::setw(10) << "Type"
              << std::setw(25) << "Vendor"
              << std::setw(25) << "Model"
              << std::setw(20) << "Serial" 
              << std::setw(25) << "URI"
              << std::endl;
    
    std::cout << std::string(120, '-') << std::endl;
    
    for (const auto& device : device_list) {
        std::string display_uri = device.uri;
        if (display_uri.length() > 22) {
            display_uri = display_uri.substr(0, 19) + "...";
        }
        
        std::string display_serial = device.serial;
        if (display_serial.length() > 17) {
            display_serial = display_serial.substr(0, 14) + "...";
        }
        
        std::string display_vendor = device.vendor;
        if (display_vendor.length() > 22) {
            display_vendor = display_vendor.substr(0, 19) + "...";
        }
        
        std::string display_model = device.model;
        if (display_model.length() > 22) {
            display_model = display_model.substr(0, 19) + "...";
        }
        
        std::cout << std::left
                  << std::setw(6) << device.index
                  << std::setw(10) << device.status_symbol
                  << std::setw(10) << device.type
                  << std::setw(25) << display_vendor
                  << std::setw(25) << display_model
                  << std::setw(20) << display_serial
                  << std::setw(25) << display_uri
                  << std::endl;
    }
    
    std::cout << "\n[DETAILED INFORMATION]" << std::endl;
    std::cout << std::string(60, '-') << std::endl;
    
    for (const auto& device : device_list) {
        std::cout << "\nDevice #" << device.index << ":" << std::endl;
        std::cout << "  URI: " << device.uri << std::endl;
        std::cout << "  Status: " << device.status_symbol << " " << device.status << std::endl;
        std::cout << "  Type: " << device.type << std::endl;
        std::cout << "  Serial: " << device.serial << std::endl;
        std::cout << "  Vendor: " << device.vendor << std::endl;
        std::cout << "  Model: " << device.model << std::endl;
        
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
    
    int network_count = 0;
    int usb_count = 0;
    
    for (const auto& device : device_list) {
        if (device.type == "Network") {
            network_count++;
        } else if (device.type == "USB") {
            usb_count++;
        }
    }
    
    std::cout << "• Total devices: " << device_list.size() << std::endl;
    std::cout << "• Network devices: " << network_count << std::endl;
    std::cout << "• USB devices: " << usb_count << std::endl;
    std::cout << "• Library Version: " << ver.major << "." << ver.minor << "." << ver.patch << std::endl;
    std::cout << std::string(80, '=') << std::endl;
    
    ASSERT_OK(TYDeinitLib());
    return 0;
}