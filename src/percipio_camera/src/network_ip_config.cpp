#include <iostream>
#include <iomanip>
#include <string>
#include <vector>
#include <memory>
#include <sstream>
#include <regex>
#include <algorithm>

#include "percipio_camera/Utils.hpp"

std::string trim(const std::string& str) {
    size_t first = str.find_first_not_of(' ');
    if (first == std::string::npos) return "";
    size_t last = str.find_last_not_of(' ');
    return str.substr(first, (last - first + 1));
}

std::string toLower(const std::string& str) {
    std::string result = str;
    std::transform(result.begin(), result.end(), result.begin(), ::tolower);
    return result;
}

bool isValidIP(const std::string& ip) {
    std::regex ip_pattern("^([0-9]{1,3}\\.){3}[0-9]{1,3}$");
    if (!std::regex_match(ip, ip_pattern)) {
        return false;
    }
    
    //Check each octet is in range 0-255
    std::stringstream ss(ip);
    std::string segment;
    while (std::getline(ss, segment, '.')) {
        int num = std::stoi(segment);
        if (num < 0 || num > 255) {
            return false;
        }
    }
    return true;
}

bool isValidNetmask(const std::string& netmask) {
    return isValidIP(netmask); // Simplified validation
}

// Interactive menu for network configuration mode
bool getNetworkModeFromUser(bool& use_dhcp, std::string& ip, 
                           std::string& netmask, std::string& gateway) {
    std::cout << "\nSelect network configuration mode:\n";
    std::cout << "  1. Static IP (manual configuration)\n";
    std::cout << "  2. DHCP (automatic IP assignment)\n";
    std::cout << "  0. Cancel\n";
    std::cout << "Choice (1, 2, or 0): ";
    
    std::string choice;
    std::getline(std::cin, choice);
    choice = trim(choice);
    
    if (choice == "0") {
        return false; // Cancelled
    }
    else if (choice == "1") {
        use_dhcp = false;
        
        // Get static IP configuration
        while (true) {
            std::cout << "\nEnter static IP configuration:\n";
            std::cout << "  IP Address (e.g. 192.168.1.100): ";
            std::getline(std::cin, ip);
            ip = trim(ip);
            
            if (isValidIP(ip) && toLower(ip) != "dhcp" && toLower(ip) != "auto") {
                break;
            } else {
                std::cout << "  Invalid IP address format. Please try again.\n";
            }
        }
        
        while (true) {
            std::cout << "  Netmask (e.g. 255.255.255.0): ";
            std::getline(std::cin, netmask);
            netmask = trim(netmask);
            
            if (isValidNetmask(netmask) && toLower(netmask) != "dhcp" && toLower(netmask) != "auto") {
                break;
            } else {
                std::cout << "  Invalid netmask format. Please try again.\n";
            }
        }
        
        while (true) {
            std::cout << "  Gateway (e.g. 192.168.1.1, or leave empty): ";
            std::getline(std::cin, gateway);
            gateway = trim(gateway);
            
            if (gateway.empty() || (isValidIP(gateway) && toLower(gateway) != "dhcp" && toLower(gateway) != "auto")) {
                break;
            } else {
                std::cout << "  Invalid gateway format. Please try again or leave empty.\n";
            }
        }
        
        return true;
    }
    else if (choice == "2") {
        use_dhcp = true;
        ip = "";
        netmask = "";
        gateway = "";
        return true;
    }
    else {
        std::cout << "Invalid choice. Please try again.\n";
        return getNetworkModeFromUser(use_dhcp, ip, netmask, gateway);
    }
}

bool setCameraIP(const TY_DEVICE_BASE_INFO& dev, const std::string& ip, const std::string& netmask, const std::string& gateway) {
    std::cout << "Setting camera IP configuration..." << std::endl;
    std::cout << "  Device Serial: " << dev.id << std::endl;
    std::cout << "  IP Address: " << ip << std::endl;
    std::cout << "  Netmask: " << netmask << std::endl;
    std::cout << "  Gateway: " << gateway << std::endl;

    TY_INTERFACE_HANDLE hIface;
    TY_STATUS rc = TYOpenInterface(dev.iface.id, &hIface);
    if(rc) {
        std::cout << "Failed to open interface, error: " << rc << std::endl;
        return false;
    }

    const char* mac = dev.netInfo.mac;
    std::string newIP = "0.0.0.0";
    std::string newNetmask = "0.0.0.0";
    std::string newGateway = "0.0.0.0";
    if(ip.length()) newIP = ip;
    if(netmask.length()) newNetmask = netmask;
    if(gateway.length()) newGateway = gateway;
    if (TYForceDeviceIP(hIface, mac, newIP.c_str(), newNetmask.c_str(), newGateway.c_str()) == TY_STATUS_OK) {
        if(newIP == "0.0.0.0" && newNetmask == "0.0.0.0" && newGateway == "0.0.0.0") {
            std::cout << "**** Set Temporary IP/Netmask/Gateway ...Done! ****" << std::endl;
            TYCloseInterface(hIface);
            return true;
        }

        TYUpdateDeviceList(hIface);

        TY_DEV_HANDLE hDev;
        rc = TYOpenDeviceWithIP(hIface, ip.c_str(), &hDev);
        if(rc == TY_STATUS_OK) {
            int32_t ip_i[4];
            uint8_t ip_b[4];
            int32_t ip32;
            sscanf(ip.c_str(), "%d.%d.%d.%d", &ip_i[0], &ip_i[1], &ip_i[2], &ip_i[3]);
            ip_b[0] = ip_i[0];ip_b[1] = ip_i[1];ip_b[2] = ip_i[2];ip_b[3] = ip_i[3];
            ip32 = TYIPv4ToInt(ip_b);
            std::cout << "Set persistent IP 0x" << std::hex << ip32 << "(" << std::dec 
                    << static_cast<int>(ip_b[0]) << "."
                    << static_cast<int>(ip_b[1]) << "."
                    << static_cast<int>(ip_b[2]) << "."
                    << static_cast<int>(ip_b[3]) << ")" << std::endl;

            ASSERT_OK( TYSetInt(hDev, TY_COMPONENT_DEVICE, TY_INT_PERSISTENT_IP, ip32) );

            sscanf(netmask.c_str(), "%d.%d.%d.%d", &ip_i[0], &ip_i[1], &ip_i[2], &ip_i[3]);
            ip_b[0] = ip_i[0];ip_b[1] = ip_i[1];ip_b[2] = ip_i[2];ip_b[3] = ip_i[3];
            ip32 = TYIPv4ToInt(ip_b);
            std::cout << "Set persistent Netmask 0x" << std::hex << ip32 << "(" << std::dec 
                    << static_cast<int>(ip_b[0]) << "."
                    << static_cast<int>(ip_b[1]) << "."
                    << static_cast<int>(ip_b[2]) << "."
                    << static_cast<int>(ip_b[3]) << ")" << std::endl;

            ASSERT_OK( TYSetInt(hDev, TY_COMPONENT_DEVICE, TY_INT_PERSISTENT_SUBMASK, ip32) );

            sscanf(gateway.c_str(), "%d.%d.%d.%d", &ip_i[0], &ip_i[1], &ip_i[2], &ip_i[3]);
            ip_b[0] = ip_i[0];ip_b[1] = ip_i[1];ip_b[2] = ip_i[2];ip_b[3] = ip_i[3];
            ip32 = TYIPv4ToInt(ip_b);
            std::cout << "Set persistent Gateway 0x" << std::hex << ip32 << "(" << std::dec 
                    << static_cast<int>(ip_b[0]) << "."
                    << static_cast<int>(ip_b[1]) << "."
                    << static_cast<int>(ip_b[2]) << "."
                    << static_cast<int>(ip_b[3]) << ")" << std::endl;

            ASSERT_OK( TYSetInt(hDev, TY_COMPONENT_DEVICE, TY_INT_PERSISTENT_GATEWAY, ip32) );

            std::cout << "**** Set Persistent IP/Netmask/Gateway ...Done! ****" << std::endl;
            TYCloseInterface(hIface);
            return true;
        } else {
            std::cerr << "Failed to open device " << dev.netInfo.ip << "(" << rc << ")" << std::endl;
            goto ForceFailed;
        }
    } else {
        std::cerr << "Force ip failed on interface " << dev.iface.id << std::endl;
        goto ForceFailed;
    }

ForceFailed:
    TYCloseInterface(hIface);
    return false;  
}

int main() {
    std::cout << "\n";
    std::cout << std::string(80, '=') << std::endl;
    std::cout << std::setw(45) << std::right << "PERCIPIO NETWORK CAMERA IP CONFIGURATION TOOL" << std::endl;
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

    std::vector<TY_DEVICE_BASE_INFO> network_cameras;

    std::vector<TY_INTERFACE_INFO> ifaces(n);
    ASSERT_OK(TYGetInterfaceList(&ifaces[0], n, &n));
    ASSERT(n == ifaces.size());
    std::vector<TY_INTERFACE_HANDLE> hIfaces;
    for (uint32_t i = 0; i < n; i++) {
        if(TYIsNetworkInterface(ifaces[i].type)) {
            TY_INTERFACE_HANDLE hIface;
            ASSERT_OK(TYOpenInterface(ifaces[i].id, &hIface));
            hIfaces.push_back(hIface);
        }
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
                network_cameras.push_back(devs[j]);
            } else {
                TY_DEV_HANDLE handle;
                int32_t ret = TYOpenDevice(hIface, devs[j].id, &handle);
                if (ret == 0) {
                    TYGetDeviceInfo(handle, &devs[j]);
                    TYCloseDevice(handle);
                }
                network_cameras.push_back(devs[j]);
            }
        }
        TYCloseInterface(hIface);
    }
    
    std::cout << std::endl << std::endl;

    if (network_cameras.empty()) {
        std::cout << "\n  No network cameras detected!" << std::endl;
        std::cout << "  Please ensure network cameras are connected and powered on." << std::endl;
        std::cout << std::string(80, '=') << std::endl;
        return 0;
    }
    
    std::cout << "\n[NETWORK CAMERA LIST - " << network_cameras.size() << " DEVICES]" << std::endl;
    std::cout << std::string(60, '=') << std::endl;
    
    std::cout << "\n" << std::left
              << std::setw(6) << "No."
              << std::setw(18) << "Current IP"
              << std::setw(15) << "Serial" 
              << std::endl;
    
    std::cout << std::string(90, '-') << std::endl;
    
    for (const auto& camera : network_cameras) {

        static int camera_index = 0;
        std::string display_serial = camera.id;
        if (display_serial.length() > 12) {
            display_serial = display_serial.substr(0, 9) + "...";
        }
        
        std::cout << std::left
                  << std::setw(6) << ++camera_index
                  << std::setw(18) << camera.netInfo.ip
                  << std::setw(35) << display_serial
                  << std::endl;
    }
    
    //User camera selection
    std::cout << "\n" << std::string(60, '-') << std::endl;
    std::cout << "IP ADDRESS CONFIGURATION" << std::endl;
    std::cout << std::string(60, '-') << std::endl;
    
    int selected_index = 0;
    while (true) {
        std::cout << "\nSelect camera number to configure IP (1-" << network_cameras.size() 
                  << "), or enter 0 to exit: ";
        
        std::string input;
        std::getline(std::cin, input);
        
        try {
            selected_index = std::stoi(input);
            
            if (selected_index == 0) {
                std::cout << "Exiting program." << std::endl;
                return 0;
            }
            
            if (selected_index >= 1 && selected_index <= static_cast<int>(network_cameras.size())) {
                break;
            } else {
                std::cout << "Invalid selection. Please try again." << std::endl;
            }
        } catch (const std::exception&) {
            std::cout << "Invalid input. Please enter a number." << std::endl;
        }
    }
    
    const auto& selected_camera = network_cameras[selected_index - 1];
    
    std::cout << "\nSelected camera #" << selected_index << ":" << std::endl;
    std::cout << "  Serial: " << selected_camera.id << std::endl;
    std::cout << "  Current IP: " << selected_camera.netInfo.ip << std::endl;
    
    // Get network configuration mode from user
    bool use_dhcp = false;
    std::string new_ip, new_netmask, new_gateway;
    if (!getNetworkModeFromUser(use_dhcp, new_ip, new_netmask, new_gateway)) {
        std::cout << "Operation cancelled." << std::endl;
        return 0;
    }

    //Confirm configuration
    std::cout << "\n" << std::string(60, '=') << std::endl;
    std::cout << "CONFIRM CONFIGURATION" << std::endl;
    std::cout << std::string(60, '-') << std::endl;
    std::cout << "Device: " << selected_camera.id << std::endl;
    std::cout << "New IP Configuration:" << std::endl;
    if(use_dhcp) {
        std::cout << "  The device will begin to obtain a new IP address from the DHCP server or link-local." << std::endl;
    } else {
        std::cout << "  IP Address: " << new_ip << std::endl;
        std::cout << "  Netmask: " << new_netmask << std::endl;
        std::cout << "  Gateway: " << (new_gateway.empty() ? "Not set" : new_gateway) << std::endl;
    }
    std::cout << "\nApply these settings? (y/n): ";
    std::string confirm;
    std::getline(std::cin, confirm);
    if (confirm == "y" || confirm == "Y" || confirm == "yes" || confirm == "YES") {
        
        //Set IP address
        bool success = setCameraIP(selected_camera, new_ip, new_netmask, new_gateway);
        if (success) {
            std::cout << "\nIP configuration applied successfully!" << std::endl;
        } else {
            std::cout << "\nFailed to apply IP configuration." << std::endl;
        }
    } else {
        std::cout << "Operation cancelled." << std::endl;
    }


    return 0;
}