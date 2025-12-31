#include <iostream>
#include <iomanip>
#include <string>
#include <vector>
#include <memory>
#include <sstream>
#include <regex>
#include <algorithm>

#include "percipio_camera/percipio_device_manager.h"
#include "percipio_camera/percipio_exception.h"

using percipio_wrapper::PercipioDeviceManager;
using percipio_wrapper::PercipioDeviceInfo;
using percipio_wrapper::PercipioException;

// Trim string helper
std::string trim(const std::string& str) {
    size_t first = str.find_first_not_of(' ');
    if (first == std::string::npos) return "";
    size_t last = str.find_last_not_of(' ');
    return str.substr(first, (last - first + 1));
}

// Convert string to lowercase
std::string toLower(const std::string& str) {
    std::string result = str;
    std::transform(result.begin(), result.end(), result.begin(), ::tolower);
    return result;
}

//Validate IP address format
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

//Validate netmask format
bool isValidNetmask(const std::string& netmask) {
    return isValidIP(netmask); // Simplified validation
}

//Check if device is a network camera based on URI pattern
bool isNetworkCamera(const PercipioDeviceInfo& dev) {
    std::string ip = dev.ip_;
    return isValidIP(ip);
}

//Set network camera IP address
bool setCameraIP(PercipioDeviceManager& manager, const std::string& uri, 
                 const std::string& ip, const std::string& netmask, const std::string& gateway) {
    try {
        //Note: Actual API may differ. This assumes existence of network configuration methods
        std::cout << "Setting camera IP configuration..." << std::endl;
        std::cout << "  Device URI: " << uri << std::endl;
        std::cout << "  IP Address: " << ip << std::endl;
        std::cout << "  Netmask: " << netmask << std::endl;
        std::cout << "  Gateway: " << gateway << std::endl;
        
        manager.setNetworkConfiguration(uri, ip, netmask, gateway);
        
        
        //std::cout << "IP configuration command sent (Note: API calls may need adjustment based on actual SDK)" << std::endl;
        return true;
    } catch (const PercipioException& e) {
        std::cerr << "Error setting IP: " << e.what() << std::endl;
        return false;
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return false;
    }
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
        ip = "0.0.0.0";
        netmask = "0.0.0.0";
        gateway = "0.0.0.0";
        return true;
    }
    else {
        std::cout << "Invalid choice. Please try again.\n";
        return getNetworkModeFromUser(use_dhcp, ip, netmask, gateway);
    }
}

int main(int argc, char** argv) {
    std::cout << "\n";
    std::cout << std::string(80, '=') << std::endl;
    std::cout << std::setw(45) << std::right << "PERCIPIO NETWORK CAMERA IP CONFIGURATION TOOL" << std::endl;
    std::cout << std::string(80, '=') << std::endl;
    
    std::cout << "\n[DEVICE SCAN]" << std::endl;
    std::cout << std::string(40, '-') << std::endl;
    
    struct NetworkCameraInfo {
        size_t index;
        std::string status;
        std::string status_symbol;
        std::string uri;
        std::string serial;
        std::string details;
        std::string current_ip; //Current IP address if available 
    };
    
    std::vector<NetworkCameraInfo> network_cameras;
    
    try {
        percipio_wrapper::PercipioDeviceManager manager;
        auto device_infos = manager.getConnectedDeviceInfos();
        
        if (device_infos->empty()) {
            std::cout << "  Status: No devices found\n" << std::endl;
            return 0;
        }
        
        std::cout << "  Scanning devices..." << std::endl;
        
        //Filter for network cameras only
        for (size_t i = 0; i < device_infos->size(); ++i) {
            const auto& device = device_infos->at(i);
            
            if (!isNetworkCamera(device)) {
                continue; //Skip non-network cameras
            }
            
            NetworkCameraInfo info;
            info.index = network_cameras.size() + 1; //Renumber for network cameras only
            info.uri = device.uri_;
            
            std::stringstream details_ss;
            details_ss << device;
            info.details = details_ss.str();
            
            //Extract current IP from URI if possible
            std::regex ip_extract("([0-9]{1,3}\\.[0-9]{1,3}\\.[0-9]{1,3}\\.[0-9]{1,3})");
            std::smatch match;
            if (std::regex_search(device.ip_, match, ip_extract)) {
                info.current_ip = match.str();
            } else {
                info.current_ip = "Unknown";
            }
            
            try {
                info.serial = manager.getSerial(device.uri_);
                info.status = "Online";
                info.status_symbol = "[✓]";
            } catch (const PercipioException& e) {
                info.serial = "Failed: " + std::string(e.what());
                info.status = "Error";
                info.status_symbol = "[✗]";
            }
            
            network_cameras.push_back(info);
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
                  << std::setw(10) << "Status"
                  << std::setw(18) << "Current IP"
                  << std::setw(35) << "URI"
                  << std::setw(15) << "Serial" 
                  << std::endl;
        
        std::cout << std::string(90, '-') << std::endl;
        
        for (const auto& camera : network_cameras) {
            std::string display_uri = camera.uri;
            if (display_uri.length() > 32) {
                display_uri = display_uri.substr(0, 29) + "...";
            }
            
            std::string display_serial = camera.serial;
            if (display_serial.length() > 12) {
                display_serial = display_serial.substr(0, 9) + "...";
            }
            
            std::cout << std::left
                      << std::setw(6) << camera.index
                      << std::setw(10) << camera.status_symbol
                      << std::setw(18) << camera.current_ip
                      << std::setw(35) << display_uri
                      << std::setw(15) << display_serial
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
        std::cout << "  URI: " << selected_camera.uri << std::endl;
        std::cout << "  Serial: " << selected_camera.serial << std::endl;
        std::cout << "  Current IP: " << selected_camera.current_ip << std::endl;
        
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
        std::cout << "Device: " << selected_camera.uri << std::endl;
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
            percipio_wrapper::PercipioDeviceManager manager_for_setting;
            
            //Set IP address
            bool success = setCameraIP(manager_for_setting, selected_camera.uri, new_ip, new_netmask, new_gateway);
            if (success) {
                std::cout << "\nIP configuration applied successfully!" << std::endl;
            } else {
                std::cout << "\nFailed to apply IP configuration." << std::endl;
            }
        } else {
            std::cout << "Operation cancelled." << std::endl;
        }
        
    } catch (const PercipioException& e) {
        std::cerr << "\nPercipio SDK Error: " << e.what() << std::endl;
        return 1;
    } catch (const std::exception& e) {
        std::cerr << "\nError: " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
}
