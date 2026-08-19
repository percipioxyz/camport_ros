#ifndef _PERCIPIO_FIRMWARE_UPDATE_
#define _PERCIPIO_FIRMWARE_UPDATE_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifdef _WIN32
    #ifdef PERCIPIO_FW_UPDATE_EXPORTS
        #define PERCIPIO_FW_UPDATE_API __declspec(dllexport)
    #else
        #define PERCIPIO_FW_UPDATE_API __declspec(dllimport)
    #endif
#else
    #define PERCIPIO_FW_UPDATE_API
#endif

#define TY_UPGRADE_MODE_ID_LEN   64
#define TY_UPGRADE_MODE_NAME_LEN 512

constexpr const char kStdUpdateNameStandard[] = "std-upgrade-standard";
constexpr const char kStdUpdateNameFullErase[] = "std-upgrade-full_erase";
constexpr const char kStdUpdateNamePreserveUserPartition[] = "std-upgrade-preserve_user_partition";


enum ty_dev_type {
    ty_dev_genicam = 0,
    ty_dev_usb3vision,
    ty_dev_invalid = -1
};


enum FW_UPDATE_STATUS
{
    UPDATE_STATUS_OK = 0,

    UPDATE_STATUS_FW_UPGRADE_FALED           = -1001,

    UPDATE_STATUS_CFG_UPGRADE_FALED          = -1002,

    UPDATE_STATUS_BACKUP_FAILED              = -1003,
    
    UPDATE_STATUS_RESTORE_FAILED             = -1004,

    UPDATE_STATUS_U3V_CMD_ERROR              = -1005,
    
    UPDATE_STATUS_SN_UPGRADE_FALED           = -1006,
    
    // Self-describing firmware upgrade errors
    UPDATE_STATUS_XML_PARSE_FAILED           = -2001,
    UPDATE_STATUS_INVALID_FIRMWARE_PACKAGE   = -2002,
    UPDATE_STATUS_NO_MATCHING_RULE           = -2003,
    UPDATE_STATUS_CONDITION_EVAL_FAILED      = -2004,
    UPDATE_STATUS_COMMAND_EXEC_FAILED        = -2005,
    UPDATE_STATUS_PACKAGE_EXTRACT_FAILED     = -2006,  // system error: disk full / permission / I/O

    // 
    NET_CONFIGURATION_CLEAR_FAILED           = -3001
};

struct ty_dev_base_info {
    ty_dev_type type;       //Device interface type
    char        sn[32];     //Device Serial Number
    char        ip[32];     //Device IP Address (Note: Only valid for network interface cameras) 
    char        model[32];  //Device model
};

typedef struct TY_UPGRADE_MODE_INFO {
    char            mode[TY_UPGRADE_MODE_ID_LEN];
    char            name[TY_UPGRADE_MODE_NAME_LEN];
} TY_UPGRADE_MODE_INFO;

typedef void (*UpdateLogCallback)(const char* dev, const char* log);

PERCIPIO_FW_UPDATE_API void upgrade_get_device_count(uint32_t* count);

PERCIPIO_FW_UPDATE_API void upgrade_get_device_list(ty_dev_base_info* devices, uint32_t count, uint32_t* filledCount);

PERCIPIO_FW_UPDATE_API FW_UPDATE_STATUS upgrade_device_firmware_from_file(const ty_dev_base_info* dev, const char* file, const bool force = false, const char* name = kStdUpdateNameStandard);

PERCIPIO_FW_UPDATE_API FW_UPDATE_STATUS upgrade_device_configuration_from_file(const ty_dev_base_info* dev, const char* file);

PERCIPIO_FW_UPDATE_API FW_UPDATE_STATUS upgrade_device_SN_from_file(const ty_dev_base_info* dev, const char* file);

PERCIPIO_FW_UPDATE_API const char* get_last_error(const ty_dev_base_info* dev);

PERCIPIO_FW_UPDATE_API FW_UPDATE_STATUS export_net_device_user_data_to_file(const ty_dev_base_info* dev, const char* file);

PERCIPIO_FW_UPDATE_API FW_UPDATE_STATUS restore_net_device_user_data_from_file(const ty_dev_base_info* dev, const char* file, const bool force = false);

PERCIPIO_FW_UPDATE_API FW_UPDATE_STATUS genicam_write_file_with_selector(const ty_dev_base_info* dev,
                                                                          const char* file,
                                                                          const char* file_selector_name);

PERCIPIO_FW_UPDATE_API FW_UPDATE_STATUS genicam_open_device(const ty_dev_base_info* dev);

PERCIPIO_FW_UPDATE_API FW_UPDATE_STATUS genicam_close_device(const ty_dev_base_info* dev);

PERCIPIO_FW_UPDATE_API FW_UPDATE_STATUS genicam_write_feature_with_type(const ty_dev_base_info* dev,
                                                                         const char* feature_name,
                                                                         const char* feature_type,
                                                                         const char* feature_value);

PERCIPIO_FW_UPDATE_API FW_UPDATE_STATUS genicam_read_feature_with_type(const ty_dev_base_info* dev,
                                                                        const char* feature_name,
                                                                        const char* feature_type,
                                                                        char* out_value,
                                                                        uint32_t out_value_size);

PERCIPIO_FW_UPDATE_API const char* device_run_cmd(const ty_dev_base_info* dev, const char* cmd);


PERCIPIO_FW_UPDATE_API FW_UPDATE_STATUS device_clear_net_configuration(const ty_dev_base_info* dev);


PERCIPIO_FW_UPDATE_API void set_global_log_callback(UpdateLogCallback cb);




/**
 * @brief Query the upgrade modes supported by a firmware package.
 *
 * Inspects the firmware file at @p fw_path and fills @p modes[] with the
 * modes available for the package.
 *
 * For self-describing (.tyfw) packages the list reflects the procedures
 * defined in control.xml (standard / backup / full_erase / custom).
 * For legacy packages exactly one entry is returned:
 * TY_UPGRADE_MODE_STANDARD.
 *
 * @param fw_path   Path to the firmware package file.
 * @param modes     Caller-allocated array to receive the results.
 * @param capacity  Number of elements @p modes can hold.
 * @param count     On return: number of entries written to @p modes.
 * @return 0 on success, -1 on failure (e.g. file not found / parse error).
 */
PERCIPIO_FW_UPDATE_API int ty_query_upgrade_modes(
                                                   const ty_dev_base_info* dev,
                                                   const char*          fw_path,
                                                   TY_UPGRADE_MODE_INFO* modes,
                                                   uint32_t              capacity,
                                                   uint32_t*             count);

/**
 * @brief Execute an upgrade using the mode selected by the caller.
 *
 * Routes internally to the appropriate backend:
 *  - Self-describing (.tyfw): selects the matching procedure and calls
 *    ty_self_desc_firmware_upgrade().
 *  - Legacy format + STANDARD: calls upgrade_device_firmware_from_file().
 *  - Any other combination returns UPDATE_STATUS_FW_UPGRADE_FALED.
 *
 * @param dev       Target device.
 * @param fw_path   Path to the firmware package file.
 * @param mode      Mode previously returned by ty_query_upgrade_modes().
 * @param mode_name For TY_UPGRADE_MODE_CUSTOM: the name field from
 *                  TY_UPGRADE_MODE_INFO (used to look up the procedure).
 *                  For all other modes this parameter is ignored and may
 *                  be NULL.
 * @param force     If true, skip device-model / version rule matching.
 * @return UPDATE_STATUS_OK on success, error code on failure.
 */
PERCIPIO_FW_UPDATE_API FW_UPDATE_STATUS ty_execute_upgrade_with_mode(
                                                   const ty_dev_base_info* dev,
                                                   const char*             fw_path,
                                                   TY_UPGRADE_MODE_INFO         mode,
                                                   bool                    force);

PERCIPIO_FW_UPDATE_API FW_UPDATE_STATUS ty_execute_uset_upgrade(
                                        const ty_dev_base_info* dev,
                                        const char*             uset_path,
                                        bool                    force);
#ifdef __cplusplus
}
#endif
#endif
