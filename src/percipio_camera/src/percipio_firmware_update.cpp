#include <string>
#include <vector>
#include <iostream>
#include <sstream>
#include <iomanip>
#include "percipio_firmware_update.h"

static std::string g_targetSn;

void callback(const char* sn, const char* log)
{
    if (g_targetSn.empty() || g_targetSn == sn) {
        std::cout << "[" << sn << "] " << log << std::endl;
    }
}

void printUsage() {
    std::cout << "Firmware Update Tool" << std::endl;
    std::cout << "Usage: firmware_update <firmware_file> -id <serial_number>" << std::endl;
    std::cout << std::endl;
    std::cout << "Arguments:" << std::endl;
    std::cout << "  <firmware_file>    Path to firmware file" << std::endl;
    std::cout << "  -id <serial>       Device serial number" << std::endl;
    std::cout << std::endl;
    std::cout << "Example:" << std::endl;
    std::cout << "  firmware_update firmware.bin -id 207xxxxxxxxx" << std::endl;
}

int main(int argc, char* argv[])
{
    std::cout << "=== Firmware Update Tool ===" << std::endl;
    
    if (argc < 4) {
        std::cerr << "Error: Insufficient arguments" << std::endl;
        printUsage();
        return 1;
    }
    
    std::string filename;
    std::string serialNumber;
    
    // args check
    for (int i = 1; i < argc; i++) {
        std::string arg = argv[i];
        
        if (arg == "-id") {
            if (i + 1 < argc) {
                serialNumber = argv[++i];
            } else {
                std::cerr << "Error: -id requires a serial number" << std::endl;
                printUsage();
                return 1;
            }
        } else if (arg[0] != '-') {
            filename = arg;
        }
    }
    
    if (filename.empty()) {
        std::cerr << "Error: No firmware file specified" << std::endl;
        printUsage();
        return 1;
    }
    
    if (serialNumber.empty()) {
        std::cerr << "Error: No device serial number specified" << std::endl;
        printUsage();
        return 1;
    }
    
    g_targetSn = serialNumber;
    
    std::cout << "[Config]" << std::endl;
    std::cout << "  Firmware: " << filename << std::endl;
    std::cout << "  Device:   " << serialNumber << std::endl;
    std::cout << std::endl;
    
    //discovering devices
    std::cout << "[1/4] Discovering devices..." << std::endl;
    uint32_t count = 0;
    upgrade_get_device_count(&count);
    
    if (count == 0) {
        std::cerr << "Error: No devices found" << std::endl;
        return -1;
    }
    
    std::cout << "  Found " << count << " device(s)" << std::endl;
    
    std::vector<ty_dev_base_info> devs(count);
    upgrade_get_device_list(&devs[0], count, &count);
    
    ty_dev_base_info* targetDev = nullptr;
    for(size_t i = 0; i < count; i++) {
        if(devs[i].sn == serialNumber) {
            targetDev = &devs[i];
            break;
        }
    }
    
    if(!targetDev) {
        std::cerr << "Error: Device \"" << serialNumber << "\" not found" << std::endl;
        std::cerr << "Available devices:" << std::endl;
        for(const auto& dev : devs) {
            std::cerr << "  - " << dev.sn << std::endl;
        }
        return -1;
    }
    
    std::cout << "  Target device found: " << targetDev->sn << std::endl;
    
    //set upgrade log callback
    set_global_log_callback(callback);
    
    //query firmware-supported upgrade modes
    std::cout << "\n[2/4] Querying upgrade modes..." << std::endl;
    const int MAX_MODES = 16;
    TY_UPGRADE_MODE_INFO modes[MAX_MODES];
    uint32_t modeCount = 0;
    int queryRet = ty_query_upgrade_modes(targetDev, filename.c_str(), modes, MAX_MODES, &modeCount);
    if (queryRet != 0) {
        std::cerr << "Error: Failed to query upgrade modes for firmware file. Return code: " << queryRet << std::endl;
        return -1;
    }
    if (modeCount == 0) {
        std::cerr << "Error: No upgrade modes available for the firmware file." << std::endl;
        return -1;
    }
    
    std::cout << "  Found " << modeCount << " upgrade mode(s):" << std::endl;
    for (uint32_t i = 0; i < modeCount; ++i) {
        std::cout << "    [" << i << "] " << modes[i].mode << std::endl;
    }
    
    //select upgrade mode: use the only mode directly, otherwise let user choose
    TY_UPGRADE_MODE_INFO selectedMode;
    if (modeCount == 1) {
        selectedMode = modes[0];
        std::cout << "  Only one mode available, using: " << selectedMode.mode << std::endl;
    } else {
        for (uint32_t i = 0; i < modeCount; ++i) {
            std::cout << "\n  --- Mode [" << i << "] " << modes[i].mode << " ---" << std::endl;
            std::cout << "  Description:" << std::endl;
            std::cout << "    " << modes[i].name << std::endl;
        }
        
        int selectedIndex = -1;
        while (true) {
            std::cout << "\n  Please select an upgrade mode [0-" << (modeCount - 1) << "]: ";
            std::string input;
            if (!std::getline(std::cin, input)) {
                std::cerr << "Error: Failed to read input." << std::endl;
                return -1;
            }
            try {
                selectedIndex = std::stoi(input);
            } catch (...) {
                std::cerr << "  Invalid input, please enter a number." << std::endl;
                continue;
            }
            if (selectedIndex < 0 || selectedIndex >= (int)modeCount) {
                std::cerr << "  Index out of range, please enter a number between 0 and " << (modeCount - 1) << "." << std::endl;
                continue;
            }
            break;
        }
        selectedMode = modes[selectedIndex];
        std::cout << "  Selected mode: " << selectedMode.mode << std::endl;
    }
    
    //execute firmware upgrade with the selected mode
    std::cout << "\n[3/4] Starting firmware update..." << std::endl;
    std::cout << "  Device:   " << targetDev->sn << std::endl;
    std::cout << "  File:     " << filename << std::endl;
    std::cout << "  Mode:     " << selectedMode.mode << std::endl;
    std::cout << "  Progress:" << std::endl;
    
    FW_UPDATE_STATUS ret = ty_execute_upgrade_with_mode(targetDev, filename.c_str(), selectedMode, true);
    
    //show result
    std::cout << "\n[4/4] Result" << std::endl;
    
    if(ret == UPDATE_STATUS_OK) {
        std::cout << "  ✓ Firmware update successful!" << std::endl;
        std::cout << "  Device " << targetDev->sn << " firmware has been successfully updated." << std::endl;
        std::cout << "  The device is now restarting, which may take from a few seconds to several tens of seconds to complete." << std::endl;
    } else {
        std::cerr << "  ✗ Firmware update failed!" << std::endl;
        std::cerr << "  Device: " << targetDev->sn << std::endl;
        std::cerr << "  Error code:   " << ret << std::endl;
        const char* err = get_last_error(targetDev);
        if (err) {
            std::cerr << "  Error detail: " << err << std::endl;
        }
    }
    
    std::cout << std::endl << "=== Operation completed ===" << std::endl;
    
    return ret;
}
