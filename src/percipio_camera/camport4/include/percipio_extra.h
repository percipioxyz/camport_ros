#ifndef _PERCIPIO_EXTRA_
#define _PERCIPIO_EXTRA_

#include <stdint.h>
#include "TYDefs.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifdef _WIN32
    #ifdef PERCIPIO_EXTRA_EXPORTS
        #define PERCIPIO_EXTRA_API __declspec(dllexport)
    #else
        #define PERCIPIO_EXTRA_API __declspec(dllimport)
    #endif
#else
    #define PERCIPIO_EXTRA_API
#endif

/**
 * @brief Status codes returned by extra API functions.
 */
enum PERCIPIO_EXTRA_STATUS
{
    PERCIPIO_EXTRA_STATUS_OK = 0,                 /**< Operation succeeded. */
    PERCIPIO_EXTRA_STATUS_DEVICE_ERROR = -1,      /**< Device handle invalid or operation failed. */
    PERCIPIO_EXTRA_STATUS_JSON_STR_INVALID = -2,  /**< JSON string is malformed or invalid. */
    PERCIPIO_EXTRA_STATUS_JSON_FILE_INVALID = -3, /**< JSON file does not exist or is malformed. */
    PERCIPIO_EXTRA_STATUS_INVALID_PARAM = -4,     /**< Invalid parameter passed to the function. */
    PERCIPIO_EXTRA_STATUS_MODEL_MISMATCH = -5,    /**< Model name in JSON file does not match device model. */
    PERCIPIO_EXTRA_STATUS_UNKNOWN = -6            /**< Unknown error or unexpected state. */
};

/**
 * @brief Load camera parameters from a JSON string to the specified device.
 * @param hDevice    Device handle (TY_DEV_HANDLE).
 * @param jsonString Null-terminated JSON string.
 * @return Status code:
 *         - PERCIPIO_EXTRA_STATUS_OK                on success
 *         - PERCIPIO_EXTRA_STATUS_DEVICE_ERROR      if device is invalid or operation fails
 *         - PERCIPIO_EXTRA_STATUS_JSON_STR_INVALID  if JSON string is invalid
 */
PERCIPIO_EXTRA_API PERCIPIO_EXTRA_STATUS PercipioExtra_LoadParamsFromString(TY_DEV_HANDLE hDevice, const char* jsonString);

/**
 * @brief Load camera parameters from a specified JSON file to the device.
 * @param hDevice    Device handle (TY_DEV_HANDLE).
 * @param filePath   Full path to the JSON file (UTF-8 encoded).
 * @return Status code:
 *         - PERCIPIO_EXTRA_STATUS_OK                on success
 *         - PERCIPIO_EXTRA_STATUS_DEVICE_ERROR      if device is invalid or operation fails
 *         - PERCIPIO_EXTRA_STATUS_JSON_FILE_INVALID if file not found or content is invalid
 */
PERCIPIO_EXTRA_API PERCIPIO_EXTRA_STATUS PercipioExtra_LoadParamsFromFile(TY_DEV_HANDLE hDevice, const char* filePath);

#ifdef __cplusplus
}
#endif

#endif /* _PERCIPIO_EXTRA_ */
