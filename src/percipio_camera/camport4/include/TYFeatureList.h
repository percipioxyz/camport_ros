/**
 * \file TYFeatureList.h
 * \brief Camera feature enumeration list
 * 
 * This file contains all feature names and their detailed descriptions extracted from PercipioStdGenICam.xml
 * Integrates XML file content from both PMD and SWIFT versions
 */

#ifndef TY_FEATURE_LIST_H
#define TY_FEATURE_LIST_H

#include "PFNC.h"

#ifdef _WIN32
# ifndef _STDINT_H
#  if defined(_MSC_VER) && _MSC_VER < 1600
    typedef __int8            int8_t;
    typedef __int16           int16_t;
    typedef __int32           int32_t;
    typedef __int64           int64_t;
    typedef unsigned __int8   uint8_t;
    typedef unsigned __int16  uint16_t;
    typedef unsigned __int32  uint32_t;
    typedef unsigned __int64  uint64_t;
#  else
#   include <stdint.h>
#  endif
# endif
#else
# include <stdint.h>
#endif

/**
 * \enum TY_FEATURE_LIST
 * \brief List of all camera-supported features
 */

/* Device control related features */
#define TY_DEV_TL_VER_MAJ                    "DeviceTLVersionMajor"                                  ///< Integer. Major version of the Transport Layer of the device
#define TY_DEV_TL_VER_MIN                    "DeviceTLVersionMinor"                                  ///< Integer. Minor version of the Transport Layer of the device

/* DeviceType */
#define TY_DEV_TYPE                          "DeviceType"                                            ///< Enumeration. Returns the device type
/* enumerations */
typedef enum TY_DEV_TYPE_LIST :uint32_t
{
    DEV_TYPE_TRANSMITTER,                                                                     ///< Data stream transmitter device
    DEV_TYPE_RECEIVER   ,                                                                     ///< Data stream receiver device
    DEV_TYPE_TRANSCEIVER,                                                                     ///< Data stream receiver and transmitter device
    DEV_TYPE_PERIPHERAL ,                                                                     ///< Controlable device (with no data stream handling)
} TY_DEV_TYPE_LIST;
static inline const char* TYGetDeviceTypeName (uint32_t type)
{
    switch (type) 
    {
        case DEV_TYPE_TRANSMITTER:              return "Transmitter";                                           ///< Data stream transmitter device
        case DEV_TYPE_RECEIVER:                 return "Receiver";                                              ///< Data stream receiver device
        case DEV_TYPE_TRANSCEIVER:              return "Transceiver";                                           ///< Data stream receiver and transmitter device
        case DEV_TYPE_PERIPHERAL:               return "Peripheral";                                            ///< Controlable device (with no data stream handling)

        default: return "Unknown";
    }
}

#define TY_DEV_VENDOR                        "DeviceVendorName"                                      ///< String. Name of the device vendor
#define TY_DEV_MODEL                         "DeviceModelName"                                       ///< String. Model of the device
#define TY_DEV_MANU_INFO                     "DeviceManufacturerInfo"                                ///< String. Manufacturer information about the device
#define TY_DEV_VER                           "DeviceVersion"                                         ///< String. Version of the device
#define TY_DEV_HW_MODEL                      "DeviceHardWareModel"                                   ///< String. Hardware model of the device
#define TY_DEV_HW_VER                        "DeviceHardwareVersion"                                 ///< String. Version of the Board Hardware
#define TY_DEV_FW_VER                        "DeviceFirmwareVersion"                                 ///< String. Version of the firmware in the device
#define TY_DEV_SN                            "DeviceSerialNumber"                                    ///< String. Serial number of the device
#define TY_DEV_BUILD_HASH                    "DeviceBuildHash"                                       ///< String. Build hash of the device
#define TY_DEV_CFG_VER                       "DeviceConfigVersion"                                   ///< String. Config version of the device
#define TY_DEV_TECH_MODEL                    "DeviceTechModel"                                       ///< String. Technical model of the device
#define TY_DEV_GEN_TIME                      "DeviceGeneratedTime"                                   ///< String. Generated time of the device
#define TY_DEV_CAL_TIME                      "DeviceCalibrationTime"                                 ///< String. Calibration time of the device
#define TY_DEV_CONN_SEL                      "DeviceConnectionSelector"                              ///< Integer. Selects which Connection of the device to control
#define TY_DEV_LINK_SEL                      "DeviceLinkSelector"                                    ///< Integer. Selects which Link of the device to control
#define TY_DEV_LINK_SPEED                    "DeviceLinkSpeed"                                       ///< Integer. Speed of the selected link

/* DeviceLinkHeartbeatMode */
#define TY_DEV_LINK_HB_MODE                  "DeviceLinkHeartbeatMode"                               ///< Enumeration. Activate or deactivate the Link`s heartbeat
/* enumerations */
typedef enum TY_DEV_LINK_HB_MODE_LIST :uint32_t
{
    DEV_LINK_HB_ON,                                                                           ///< Speed of the selected link
    DEV_LINK_HB_OFF,                                                                          ///< Speed of the selected link
} TY_DEV_LINK_HB_MODE_LIST;
static inline const char* TYGetDeviceLinkHeartbeatModeName (uint32_t mode)
{
    switch (mode) 
    {
        case DEV_LINK_HB_ON:                    return "On";                                                    ///< Speed of the selected link
        case DEV_LINK_HB_OFF:                   return "Off";                                                   ///< Speed of the selected link

        default: return "Unknown";
    }
}

#define TY_DEV_LINK_HB_TIMEOUT               "DeviceLinkHeartbeatTimeout"                            ///< Integer. Speed of the selected link
#define TY_DEV_STREAM_CH_COUNT               "DeviceStreamChannelCount"                              ///< Integer. Indicates the number of streaming channels supported by the device
#define TY_DEV_STREAM_CH_SEL                 "DeviceStreamChannelSelector"                           ///< Integer. Selects the stream channel to configure

/* DeviceStreamChannelType */
#define TY_DEV_STREAM_CH_TYPE                "DeviceStreamChannelType"                               ///< Enumeration. Type of the selected stream channel
/* enumerations */
typedef enum TY_DEV_STREAM_CH_TYPE_LIST :uint32_t
{
    DEV_STREAM_CH_TRANSMITTER,                                                                ///< Data stream transmitter channel
    DEV_STREAM_CH_RECEIVER,                                                                   ///< Data stream receiver channel
} TY_DEV_STREAM_CH_TYPE_LIST;
static inline const char* TYGetDeviceStreamChannelTypeName (uint32_t type)
{
    switch (type) 
    {
        case DEV_STREAM_CH_TRANSMITTER:         return "Transmitter";                                           ///< Data stream transmitter channel
        case DEV_STREAM_CH_RECEIVER:            return "Receiver";                                              ///< Data stream receiver channel

        default: return "Unknown";
    }
}

#define TY_DEV_STREAM_CH_LINK                "DeviceStreamChannelLink"                               ///< Integer. Link associated with the selected stream channel
#define TY_DEV_STREAM_CH_PKT_SIZE            "DeviceStreamChannelPacketSize"                         ///< Integer. Packet size used on the selected stream channel

/* DeviceCharacterSet */
#define TY_DEV_CHARSET                       "DeviceCharacterSet"                                    ///< Enumeration. Character set used by the strings of the device's bootstrap registers
/* enumerations */
typedef enum TY_DEV_CHARSET_LIST :uint32_t
{
    DEV_CHARSET_UTF8 = 1,                                                                     ///< Device use UTF8 character set
    DEV_CHARSET_ASCII,                                                                        ///< Device use ASCII character set
} TY_DEV_CHARSET_LIST;
static inline const char* TYGetDeviceCharacterSetName (uint32_t code)
{
    switch (code) 
    {
        case DEV_CHARSET_UTF8:                  return "UTF8";                                                  ///< Device use UTF8 character set
        case DEV_CHARSET_ASCII:                 return "ASCII";                                                 ///< Device use ASCII character set
        
        default: return "Unknown";
    }
}

#define TY_DEV_RESET                         "DeviceReset"                                           ///< Command. Reset the device to the state it had after power-up
#define TY_DEV_ERR_CODE                      "DeviceErrCode"                                         ///< Integer. Error code of the device
#define TY_DEV_FRAME_TIMEOUT                 "DeviceFrameRecvTimeOut"                                ///< Integer. The waiting timeout for the device to send a single frame of data (unit: ms)

/* DeviceTemperatureSelector */
#define TY_DEV_TEMP_SEL                      "DeviceTemperatureSelector"                             ///< Enumeration. Selects the temperature sensor to read
/* enumerations */
typedef enum TY_DEV_TEMP_SEL_LIST :uint32_t
{
    DEV_TEMP_SEL_MAINBOARD,                                                                   ///< Temperature of the mainboard
    DEV_TEMP_SEL_LEFT,                                                                        ///< Temperature of the left binocular module
    DEV_TEMP_SEL_RIGHT,                                                                       ///< Temperature of the right binocular module
    DEV_TEMP_SEL_TEXTURE,                                                                     ///< Temperature of the texture module
    DEV_TEMP_SEL_LASER,                                                                       ///< Temperature of the laser
    DEV_TEMP_SEL_IRFLASH,                                                                     ///< Temperature of the ir flash
    DEV_TEMP_SEL_TEXTURE_FLASH,                                                               ///< Temperature of the Texture flash
    DEV_TEMP_SEL_CPUCORE = 8,                                                                 ///< Temperature of the CPU core
} TY_DEV_TEMP_SEL_LIST;
static inline const char* TYGetDeviceTemperatureSelectorName (uint32_t sel)
{
    switch (sel) 
    {
        case DEV_TEMP_SEL_MAINBOARD:            return "Mainboard";                                             ///< Temperature of the mainboard
        case DEV_TEMP_SEL_LEFT:                 return "Left";                                                  ///< Temperature of the left binocular module
        case DEV_TEMP_SEL_RIGHT:                return "Right";                                                 ///< Temperature of the right binocular module
        case DEV_TEMP_SEL_TEXTURE:              return "Texture";                                               ///< Temperature of the texture module
        case DEV_TEMP_SEL_LASER:                return "Laser";                                                 ///< Temperature of the laser
        case DEV_TEMP_SEL_IRFLASH:              return "IRFlash";                                               ///< Temperature of the ir flash
        case DEV_TEMP_SEL_TEXTURE_FLASH:        return "ColorFlash";                                            ///< Temperature of the color flash
        case DEV_TEMP_SEL_CPUCORE:              return "CpuCore";                                               ///< Temperature of the CPU core

        default: return "Unknown";
    }
}

#define TY_DEV_TEMP                          "DeviceTemperature"                                     ///< Float. Temperature of the selected by DeviceTemperatureSelector
#define TY_DEV_TEMP_STR                      "DeviceTemperatureString"                               ///< String. Temperature of the selected by DeviceTemperatureSelector

/* DeviceStreamAsyncMode */
#define TY_DEV_STREAM_ASYNC                  "DeviceStreamAsyncMode"                                 ///< Enumeration. Controls stream asynchronous mode
/* enumerations */
typedef enum TY_DEV_STREAM_ASYNC_LIST :uint32_t
{
    DEV_STREAM_ASYNC_OFF,                                                                     ///< None of the data streams are output asynchronously
    DEV_STREAM_ASYNC_DEPTH,                                                                   ///< The acquisition of range (distance)  data is output asynchronously
    DEV_STREAM_ASYNC_RGB,                                                                     ///< The acquisition of intensity  data is output asynchronously
    DEV_STREAM_ASYNC_DEPTHANDRGB,                                                             ///< Intensity and range (distance) data are acquired and output asynchronously
    DEV_STREAM_ASYNC_ALL = 255,                                                                     ///< All of the data streams are output asynchronously
} TY_DEV_STREAM_ASYNC_LIST;
static inline const char* TYGetDeviceStreamAsyncModeName (uint32_t mode)
{
    switch (mode) 
    {
        case DEV_STREAM_ASYNC_OFF:              return "AsyncOff";                                              ///< None of the data streams are output asynchronously
        case DEV_STREAM_ASYNC_DEPTH:            return "AsyncDepth";                                            ///< The acquisition of range (distance)  data is output asynchronously
        case DEV_STREAM_ASYNC_RGB:              return "AsyncRGB";                                              ///< The acquisition of intensity  data is output asynchronously
        case DEV_STREAM_ASYNC_DEPTHANDRGB:      return "AsyncDepthAndRGB";                                      ///< Intensity and range (distance) data are acquired and output asynchronously
        case DEV_STREAM_ASYNC_ALL:              return "AsyncAll";                                              ///< All of the data streams are output asynchronously

        default: return "Unknown";
    }
}

/* DeviceTimeSyncMode */
#define TY_DEV_TIME_SYNC                     "DeviceTimeSyncMode"                                    ///< Enumeration. Selects the time synchronization mode
/*enumerations */
typedef enum TY_DEV_TIME_SYNC_LIST :uint32_t
{
    DEV_TIME_SYNC_NONE,                                                                       ///< No synchronization
    DEV_TIME_SYNC_HOST,                                                                       ///< Host synchronization
    DEV_TIME_SYNC_NTP,                                                                        ///< NTP synchronization
    DEV_TIME_SYNC_PTP_SLAVE,                                                                  ///< PTP slave synchronization
    DEV_TIME_SYNC_CAN,                                                                        ///< CAN synchronization
    DEV_TIME_SYNC_PTP_MASTER,                                                                 ///< PTP master synchronization
} TY_DEV_TIME_SYNC_LIST;
static inline const char* TYGetDeviceTimeSyncModeName (uint32_t mode)
{
    switch (mode) 
    {
        case DEV_TIME_SYNC_NONE:                return "SyncTypeNone";                                          ///< No synchronization
        case DEV_TIME_SYNC_HOST:                return "SyncTypeHost";                                          ///< Host synchronization
        case DEV_TIME_SYNC_NTP:                 return "SyncTypeNTP";                                           ///< NTP synchronization
        case DEV_TIME_SYNC_PTP_SLAVE:           return "SyncTypePTPSlave";                                      ///< PTP slave synchronization
        case DEV_TIME_SYNC_CAN:                 return "SyncTypeCAN";                                           ///< CAN synchronization
        case DEV_TIME_SYNC_PTP_MASTER:          return "SyncTypePTPMaster";                                     ///< PTP master synchronization

        default: return "Unknown";
    }
}

#define TY_NTP_SRV_IP                        "NTPServerIP"                                           ///< Integer. NTP server IP address

/* DeviceDataCompressType */
#define TY_DEV_COMPRESS                      "DeviceDataCompressType"                                ///< Enumeration. Selects the data compression type
/* enumerations */
typedef enum TY_DEV_COMPRESS_LIST :uint32_t
{
    DEV_COMPRESS_NONE = 1,                                                                    ///< No compression
    DEV_COMPRESS_HW,                                                                          ///< LZ4 hardware compression
} TY_DEV_COMPRESS_LIST;
static inline const char* TYGetDeviceDataCompressTypeName (uint32_t type)
{
    switch (type) 
    {
        case DEV_COMPRESS_NONE:                 return "None";                                                  ///< No compression
        case DEV_COMPRESS_HW:                   return "LZ4";                                                   ///< LZ4 hardware compression

        default: return "Unknown";
    }
}

/* Acquisition control related features */
/* AcquisitionMode */
#define TY_ACQ_MODE                          "AcquisitionMode"                                       ///< Enumeration. Sets the acquisition mode of the device
/* enumerations */
typedef enum TY_ACQ_MODE_LIST :uint32_t
{
    ACQ_MODE_SINGLE_FRAME,                                                                    ///< Single frame acquisition mode
    ACQ_MODE_MULTI_FRAME,                                                                     ///< Multi frame acquisition mode
    ACQ_MODE_CONTINUOUS,                                                                      ///< Continuous acquisition mode
} TY_ACQ_MODE_LIST;
static inline const char* TYGetAcquisitionModeName (uint32_t mode)
{
    switch (mode) 
    {
        case ACQ_MODE_SINGLE_FRAME:             return "SingleFrame";                                           ///< Single frame acquisition mode
        case ACQ_MODE_MULTI_FRAME:              return "MultiFrame";                                            ///< Multi frame acquisition mode
        case ACQ_MODE_CONTINUOUS:               return "Continuous";                                            ///< Continuous acquisition mode

        default: return "Unknown";
    }
}

#define TY_ACQ_START                         "AcquisitionStart"                                      ///< Command. Starts the acquisition of images
#define TY_ACQ_STOP                          "AcquisitionStop"                                       ///< Command. Stops the acquisition of images
#define TY_ACQ_FPS                           "AcquisitionFrameRate"                                  ///< Float. Controls the acquisition frame rate
#define TY_ACQ_FPS_EN                        "AcquisitionFrameRateEnable"                            ///< Boolean. Enables the acquisition frame rate control

#define TY_EXP_AUTO                          "ExposureAuto"                                          ///< Boolean. Sets the automatic exposure mode
#define TY_EXP_AUTO_MIN                      "ExposureAutoLowerLimit"                                ///< Float. Lower limit of the auto exposure time
#define TY_EXP_AUTO_MAX                      "ExposureAutoUpperLimit"                                ///< Float. Upper limit of the auto exposure time
#define TY_EXP_TARGET                        "ExposureTargetBrightness"                              ///< Float. Target brightness for auto exposure
#define TY_EXP_TIME                          "ExposureTime"                                          ///< Float. Sets the exposure time
#define TY_HDR_EN                            "HDREnable"                                             ///< Boolean. Enables HDR mode
#define TY_HDR_CTRL                          "HDRControl"                                            ///< Bytearray. Controls HDR parameters

/* Trigger control related features */
/* TriggerSelector */
#define TY_TRIG_SEL                          "TriggerSelector"                                       ///< Enumeration. Selects the trigger type to configure
/* enumerations */
typedef enum TY_TRIG_SEL_LIST :uint32_t
{
    TRIG_SEL_FRAME_ACQUISITIONSTART,                                                          ///< Trigger for acquisition start
    TRIG_SEL_FRAME_ACQUISITIONEND,                                                            ///< Trigger for acquisition end
    TRIG_SEL_FRAME_ACQUISITIONACTIVE,                                                         ///< Trigger for acquisition active
    TRIG_SEL_FRAME_FRAMESTART,                                                                ///< Trigger for frame start
    TRIG_SEL_FRAME_FRAMEEND,                                                                  ///< Trigger for frame end
    TRIG_SEL_FRAME_FRAMEACTIVE,                                                               ///< Trigger for frame active
    TRIG_SEL_FRAME_FRAMEBURSTSTART,                                                           ///< Trigger for frame burst start
    TRIG_SEL_FRAME_FRAMEBURSTEND,                                                             ///< Trigger for frame burst end
    TRIG_SEL_FRAME_FRAMEBURSTACTIVE,                                                          ///< Trigger for frame burst active
    TRIG_SEL_FRAME_LINESTART,                                                                 ///< Trigger for line start
    TRIG_SEL_FRAME_EXPOSURESTART,                                                             ///< Trigger for exposure start
    TRIG_SEL_FRAME_EXPOSUREEND,                                                               ///< Trigger for exposure end
    TRIG_SEL_FRAME_EXPOSUREACTIVE,                                                            ///< Trigger for exposure active
} TY_TRIG_SEL_LIST;
static inline const char* TYGetTriggerSelectorName (uint32_t sel)
{
    switch (sel) 
    {
        case TRIG_SEL_FRAME_ACQUISITIONSTART:   return "AcquisitionStart";                                      ///< Trigger for acquisition start
        case TRIG_SEL_FRAME_ACQUISITIONEND:     return "AcquisitionEnd";                                        ///< Trigger for acquisition end
        case TRIG_SEL_FRAME_ACQUISITIONACTIVE:  return "AcquisitionActive";                                     ///< Trigger for acquisition active
        case TRIG_SEL_FRAME_FRAMESTART:         return "FrameStart";                                            ///< Trigger for frame start
        case TRIG_SEL_FRAME_FRAMEEND:           return "FrameEnd";                                              ///< Trigger for frame end
        case TRIG_SEL_FRAME_FRAMEACTIVE:        return "FrameActive";                                           ///< Trigger for frame active
        case TRIG_SEL_FRAME_FRAMEBURSTSTART:    return "FrameBurstStart";                                       ///< Trigger for frame burst start
        case TRIG_SEL_FRAME_FRAMEBURSTEND:      return "FrameBurstEnd";                                         ///< Trigger for frame burst end
        case TRIG_SEL_FRAME_FRAMEBURSTACTIVE:   return "FrameBurstActive";                                      ///< Trigger for frame burst active
        case TRIG_SEL_FRAME_LINESTART:          return "LineStart";                                             ///< Trigger for line start
        case TRIG_SEL_FRAME_EXPOSURESTART:      return "ExposureStart";                                         ///< Trigger for exposure start
        case TRIG_SEL_FRAME_EXPOSUREEND:        return "ExposureEnd";                                           ///< Trigger for exposure end
        case TRIG_SEL_FRAME_EXPOSUREACTIVE:     return "ExposureActive";                                        ///< Trigger for exposure active

        default: return "Unknown";
    }
}

/* TriggerMode */
#define TY_TRIG_MODE                         "TriggerMode"                                           ///< Enumeration. Controls whether the selected trigger is active
/* enumerations */
typedef enum TY_TRIG_MODE_LIST :uint32_t
{
    TRIG_MODE_OFF,                                                                            ///< Disables the selected trigger
    TRIG_MODE_ON,                                                                             ///< Enable the selected trigger
} TY_TRIG_MODE_LIST;
static inline const char* TYGetTriggerModeName (uint32_t mode)
{
    switch (mode) 
    {
        case TRIG_MODE_OFF:                     return "Off";                                                   ///< Disables the selected trigger
        case TRIG_MODE_ON:                      return "On";                                                    ///< Enable the selected trigger

        default: return "Unknown";
    }
}

#define TY_TRIG_SW                           "TriggerSoftware"                                       ///< Command. Generates a software trigger signal

/* TriggerSource */
#define TY_TRIG_SRC                          "TriggerSource"                                         ///< Enumeration. Specifies the internal signal or physical input line to use as the trigger source
/* enumerations */
typedef enum TY_TRIG_SRC_LIST :uint32_t
{
    TRIG_SRC_LINE0,                                                                           ///< Trigger source line 0
    TRIG_SRC_LINE1,                                                                           ///< Trigger source line 1
    TRIG_SRC_LINE2,                                                                           ///< Trigger source line 2
    TRIG_SRC_LINE3,                                                                           ///< Trigger source line 3
    TRIG_SRC_LINE4,                                                                           ///< Trigger source line 4
    TRIG_SRC_LINE5,                                                                           ///< Trigger source line 5
    TRIG_SRC_LINE6,                                                                           ///< Trigger source line 6
    TRIG_SRC_LINE7,                                                                           ///< Trigger source line 7
    TRIG_SRC_SOFTWARE,                                                                        ///< Software trigger source
} TY_TRIG_SRC_LIST;
static inline const char* TYGetTriggerSourceName (uint32_t source)
{
    switch (source) 
    {
        case TRIG_SRC_LINE0:                    return "Line0";                                                 ///< Trigger source line 0
        case TRIG_SRC_LINE1:                    return "Line1";                                                 ///< Trigger source line 1
        case TRIG_SRC_LINE2:                    return "Line2";                                                 ///< Trigger source line 2
        case TRIG_SRC_LINE3:                    return "Line3";                                                 ///< Trigger source line 3
        case TRIG_SRC_LINE4:                    return "Line4";                                                 ///< Trigger source line 4
        case TRIG_SRC_LINE5:                    return "Line5";                                                 ///< Trigger source line 5
        case TRIG_SRC_LINE6:                    return "Line6";                                                 ///< Trigger source line 6
        case TRIG_SRC_LINE7:                    return "Line7";                                                 ///< Trigger source line 7
        case TRIG_SRC_SOFTWARE:                 return "Software";                                              ///< Software trigger source

        default: return "Unknown";
    }
}

/* TriggerActivation */
#define TY_TRIG_ACT                          "TriggerActivation"                                     ///< Enumeration. Specifies the activation mode of the trigger
/* enumerations */
typedef enum TY_TRIG_ACT_LIST :uint32_t
{
    TRIG_ACT_RISING_EDGE,                                                                     ///< Rising edge trigger activation
    TRIG_ACT_FALLING_EDGE,                                                                    ///< Falling edge trigger activation
    TRIG_ACT_ANY_EDGE,                                                                        ///< Any edge trigger activation
    TRIG_ACT_LEVEL_HIGH,                                                                      ///< High level trigger activation
    TRIG_ACT_LEVEL_LOW,                                                                       ///< Low level trigger activation
} TY_TRIG_ACT_LIST;
static inline const char* TYGetTriggerActivationName (uint32_t activation)
{
    switch (activation) 
    {
        case TRIG_ACT_RISING_EDGE:              return "RisingEdge";                                            ///< Rising edge trigger activation
        case TRIG_ACT_FALLING_EDGE:             return "FallingEdge";                                           ///< Falling edge trigger activation
        case TRIG_ACT_ANY_EDGE:                 return "AnyEdge";                                               ///< Any edge trigger activation
        case TRIG_ACT_LEVEL_HIGH:               return "LevelHigh";                                             ///< High level trigger activation
        case TRIG_ACT_LEVEL_LOW:                return "LevelLow";                                              ///< Low level trigger activation

        default: return "Unknown";
    }
}

#define TY_TRIG_DELAY                        "TriggerDelay"                                          ///< Float. Specifies the delay in microseconds to apply after the trigger reception before activating it
#define TY_TRIG_OUT                          "TriggerOutIO"                                          ///< Boolean. Controls trigger output IO
#define TY_TRIG_DUR                          "TriggerDurationUs"                                     ///< Integer. Specifies the duration in microseconds of the trigger signal
#define TY_CMOS_SYNC                         "CmosSync"                                              ///< Boolean. Controls CMOS synchronization
#define TY_TIME_SYNC_ACK                     "TimeSyncAck"                                           ///< Boolean. Time synchronization acknowledge

/* Analog control related features */
/* GainSelector */
#define TY_GAIN_SEL                          "GainSelector"                                          ///< Enumeration. Selects which gain to control
/* enumerations */
typedef enum TY_GAIN_SEL_LIST :uint32_t
{
    GAIN_SEL_ANALOG_ALL,                                                                      ///< All analog gains
    GAIN_SEL_DIGITAL_ALL,                                                                     ///< All digital gains
    GAIN_SEL_DIGITALRED,                                                                      ///< Digital red gain
    GAIN_SEL_DIGITALGREEN,                                                                    ///< Digital green gain
    GAIN_SEL_DIGITALBLUE,                                                                     ///< Digital blue gain
} TY_GAIN_SEL_LIST;
static inline const char* TYGetGainSelectorName (uint32_t sel)
{
    switch (sel) 
    {
        case GAIN_SEL_ANALOG_ALL:               return "AnalogAll";                                             ///< All analog gains
        case GAIN_SEL_DIGITAL_ALL:              return "DigitalAll";                                            ///< All digital gains
        case GAIN_SEL_DIGITALRED:               return "DigitalRed";                                            ///< Digital red gain
        case GAIN_SEL_DIGITALGREEN:             return "DigitalGreen";                                          ///< Digital green gain
        case GAIN_SEL_DIGITALBLUE:              return "DigitalBlue";                                           ///< Digital blue gain

        default: return "Unknown";
    }
}

#define TY_GAIN                              "Gain"                                                  ///< Float. Controls the selected gain as a factor
#define TY_WB_AUTO                           "BalanceWhiteAuto"                                      ///< Boolean. Balance White Auto is the 'automatic' counterpart of the manual white balance feature
#define TY_AF_AOI_X                          "AutoFunctionAOIOffsetX"                                ///< Integer. Horizontal offset of the auto function AOI
#define TY_AF_AOI_Y                          "AutoFunctionAOIOffsetY"                                ///< Integer. Vertical offset of the auto function AOI
#define TY_AF_AOI_W                          "AutoFunctionAOIWidth"                                  ///< Integer. Width of the auto function AOI
#define TY_AF_AOI_H                          "AutoFunctionAOIHeight"                                 ///< Integer. Height of the auto function AOI

/* Image format control related features */
#define TY_SENSOR_W                          "SensorWidth"                                           ///< Integer. Width of the camera sensor in pixels
#define TY_SENSOR_H                          "SensorHeight"                                          ///< Integer. Height of the camera sensor in pixels

/* PixelFormat */
#define TY_PIX_FMT                           "PixelFormat"                                           ///< Enumeration. Format of the pixels provided by the device
/* enumerations */
/*
// Pixel Format Naming Convention. Please refer to the PFNC.h file
#define PFNC_Mono8                                      "Mono8"
#define PFNC_BayerGB8                                   "BayerGB8"
#define PFNC_BayerBG8                                   "BayerBG8"
#define PFNC_BayerGR8                                   "BayerGR8"
#define PFNC_BayerRG8                                   "BayerRG8"
#define PFNC_Mono10p                                    "Mono10p"
#define PFNC_BayerGB10p                                 "BayerGB10p"
#define PFNC_BayerBG10p                                 "BayerBG10p"
#define PFNC_BayerGR10p                                 "BayerGR10p"
#define PFNC_BayerRG10P                                 "BayerRG10P"
#define PFNC_Mono12p                                    "Mono12p"
#define PFNC_BayerGB12p                                 "BayerGB12p"
#define PFNC_BayerBG12p                                 "BayerBG12p"
#define PFNC_BayerGR12p                                 "BayerGR12p"
#define PFNC_BayerRG12p                                 "BayerRG12p"
#define PFNC_Mono16                                     "Mono16"
#define PFNC_BayerGB16                                  "BayerGB16"
#define PFNC_BayerBG16                                  "BayerBG16"
#define PFNC_BayerGR16                                  "BayerGR16"
#define PFNC_BayerRG16                                  "BayerRG16"
#define PFNC_RGB8                                       "RGB8"
#define PFNC_BGR8                                       "BGR8"
#define PFNC_YUV422_8                                   "YUV422_8"
#define PFNC_YUV422_8_UYVY                              "YUV422_8_UYVY"
#define PFNC_YCbCr420_8_YY_CbCr_Planar                  "YCbCr420_8_YY_CbCr_Planar"
#define PFNC_YCbCr420_8_YY_CrCb_Planar                  "YCbCr420_8_YY_CrCb_Planar"
#define PFNC_YCbCr420_8_YY_CbCr_Semiplanar              "YCbCr420_8_YY_CbCr_Semiplanar"
#define PFNC_YCbCr420_8_YY_CrCb_Semiplanar              "YCbCr420_8_YY_CrCb_Semiplanar"
#define PFNC_Coord3D_C16                                "Coord3D_C16"
#define PFNC_Coord3D_ABC16                              "Coord3D_ABC16"
#define PFNC_Coord3D_ABC32f                             "Coord3D_ABC32f"
*/

/* Private Pixel Format Definitions of TY. Please refer to the XXX file for more details. */
typedef enum TY_PIX_FMT_LIST :uint32_t
{
    CSIMono10P             = 0x810A0046,   ///< Camera Serial Interface Packed Mono10
    CSIBayerGB10P         = 0x810A0054,   ///< Camera Serial Interface Packed BayerGB10
    CSIBayerBG10P         = 0x810A0052,   ///< Camera Serial Interface Packed BayerBG10
    CSIBayerGR10P         = 0x810A0056,   ///< Camera Serial Interface Packed BayerGR10
    CSIBayerRG10P         = 0x810A0058,   ///< Camera Serial Interface Packed BayerRG10
    CSIMono12P             = 0x810C0047,   ///< Camera Serial Interface Packed Mono12
    CSIBayerGB12P         = 0x810C0055,   ///< Camera Serial Interface Packed BayerGB12
    CSIBayerBG12P         = 0x810C0053,   ///< Camera Serial Interface Packed BayerBG12
    CSIBayerGR12P         = 0x810C0057,   ///< Camera Serial Interface Packed BayerGR12
    CSIBayerRG12P         = 0x810C0059,   ///< Camera Serial Interface Packed BayerRG12
    CSIMono14P             = 0x810E0104,   ///< Camera Serial Interface Packed Mono14
    CSIBayerGB14P         = 0x810E0107,   ///< Camera Serial Interface Packed BayerGB14
    CSIBayerBG14P         = 0x810E0108,   ///< Camera Serial Interface Packed BayerBG14
    CSIBayerGR14P         = 0x810E0105,   ///< Camera Serial Interface Packed BayerGR14
    CSIBayerRG14P         = 0x810E0106,   ///< Camera Serial Interface Packed BayerRG14
    JPEG                   = 0x82180015,   ///< JPEG encoding format
    TofIR_FourGroup_Mono16 = 0x81400016,   ///< TofIR_FourGroup_Mono16 encoding format
} TY_PIX_FMT_LIST;
static inline const char* TYGetPixelFormatName (uint32_t format)
{
    switch (format)
    {
        case CSIMono10P            : return "CSIMono10P";              ///< Camera Serial Interface Packed Mono10
        case CSIBayerGB10P         : return "CSIBayerGB10P";           ///< Camera Serial Interface Packed BayerGB10
        case CSIBayerBG10P         : return "CSIBayerBG10P";           ///< Camera Serial Interface Packed BayerBG10
        case CSIBayerGR10P         : return "CSIBayerGR10P";           ///< Camera Serial Interface Packed BayerGR10
        case CSIBayerRG10P         : return "CSIBayerRG10P";           ///< Camera Serial Interface Packed BayerRG10
        case CSIMono12P            : return "CSIMono12P";              ///< Camera Serial Interface Packed Mono12
        case CSIBayerGB12P         : return "CSIBayerGB12P";           ///< Camera Serial Interface Packed BayerGB12
        case CSIBayerBG12P         : return "CSIBayerBG12P";           ///< Camera Serial Interface Packed BayerBG12
        case CSIBayerGR12P         : return "CSIBayerGR12P";           ///< Camera Serial Interface Packed BayerGR12
        case CSIBayerRG12P         : return "CSIBayerRG12P";           ///< Camera Serial Interface Packed BayerRG12
        case CSIMono14P            : return "CSIMono14P";              ///< Camera Serial Interface Packed Mono14
        case CSIBayerGB14P         : return "CSIBayerGB14P";           ///< Camera Serial Interface Packed BayerGB14
        case CSIBayerBG14P         : return "CSIBayerBG14P";           ///< Camera Serial Interface Packed BayerBG14
        case CSIBayerGR14P         : return "CSIBayerGR14P";           ///< Camera Serial Interface Packed BayerGR14
        case CSIBayerRG14P         : return "CSIBayerRG14P";           ///< Camera Serial Interface Packed BayerRG14
        case JPEG                  : return "JPEG";                    ///< JPEG encoding format
        case TofIR_FourGroup_Mono16: return "TofIR_FourGroup_Mono16";  ///< TofIR_FourGroup_Mono16 encoding format

        default: return GetPixelFormatName((PfncFormat) format);
    }
}

/* BinningHorizontal */
#define TY_BIN_H                             "BinningHorizontal"                                     ///< Enumeration. Number of horizontal pixels to combine together
/* enumerations */
typedef enum TY_BIN_H_LIST :uint32_t
{
    BIN_H1 = 1,                                                                               ///< Horizontal binning factor 1
    BIN_H2,                                                                                   ///< Horizontal binning factor 2
    BIN_H3,                                                                                   ///< Horizontal binning factor 3
    BIN_H4,                                                                                   ///< Horizontal binning factor 4
} TY_BIN_H_LIST;
static inline const char* TYGetBinningHorizontalName (uint32_t num)
{
    switch (num) 
    {
        case BIN_H1:                            return "BinningHorizontal1";                                    ///< Horizontal binning factor 1
        case BIN_H2:                            return "BinningHorizontal2";                                    ///< Horizontal binning factor 2
        case BIN_H3:                            return "BinningHorizontal3";                                    ///< Horizontal binning factor 3
        case BIN_H4:                            return "BinningHorizontal4";                                    ///< Horizontal binning factor 4

        default: return "Unknown";
    }
}

/* BinningVertical */
#define TY_BIN_V                             "BinningVertical"                                       ///< Enumeration. Number of vertical pixels to combine together
/* enumerations */
typedef enum TY_BIN_V_LIST :uint32_t
{
    BIN_V1 = 1,                                                                               ///< Vertical binning factor 1
    BIN_V2,                                                                                   ///< Vertical binning factor 2
    BIN_V3,                                                                                   ///< Vertical binning factor 3
    BIN_V4,                                                                                   ///< Vertical binning factor 4
} TY_BIN_V_LIST;
static inline const char* TYGetBinningVerticalName (uint32_t num)
{
    switch (num) 
    {
        case BIN_V1:                            return "BinningVertical1";                                      ///< Vertical binning factor 1
        case BIN_V2:                            return "BinningVertical2";                                      ///< Vertical binning factor 2
        case BIN_V3:                            return "BinningVertical3";                                      ///< Vertical binning factor 3
        case BIN_V4:                            return "BinningVertical4";                                      ///< Vertical binning factor 4

        default: return "Unknown";
    }
}

#define TY_COMP_EN                           "ComponentEnable"                                       ///< Boolean. Enables the component

/* Depth processing related features */
#define TY_DEPTH_SCALE                       "DepthScaleUnit"                                        ///< Float. Depth scale unit
#define TY_DEPTH_SGBM_IMG_NUM                "DepthSgbmImageNumber"                                  ///< Integer. SGBM image number
#define TY_DEPTH_SGBM_DISP_NUM               "DepthSgbmDisparityNumber"                              ///< Integer. SGBM disparity number
#define TY_DEPTH_SGBM_DISP_OFF               "DepthSgbmDisparityOffset"                              ///< Integer. SGBM disparity offset
#define TY_DEPTH_SGBM_WIN_H                  "DepthSgbmMatchWinHeight"                               ///< Integer. SGBM match window height
#define TY_DEPTH_SGBM_P1                     "DepthSgbmSemiParamP1"                                  ///< Integer. SGBM semi global param p1
#define TY_DEPTH_SGBM_P2                     "DepthSgbmSemiParamP2"                                  ///< Integer. SGBM semi global param p2
#define TY_DEPTH_SGBM_UNIQUE                 "DepthSgbmUniqueFactor"                                 ///< Integer. SGBM unique factor
#define TY_DEPTH_SGBM_UNIQUE_DIFF            "DepthSgbmUniqueAbsDiff"                                ///< Integer. SGBM unique absolute difference
#define TY_DEPTH_SGBM_UNIQUE_COST            "DepthSgbmUniqueMaxCost"                                ///< Integer. SGBM unique max cost
#define TY_DEPTH_SGBM_H_WIN                  "DepthSgbmHFilterHalfWin"                               ///< Boolean. SGBM horizontal filter half window
#define TY_DEPTH_SGBM_WIN_W                  "DepthSgbmMatchWinWidth"                                ///< Integer. SGBM match window width
#define TY_DEPTH_SGBM_MED                    "DepthSgbmMedFilter"                                    ///< Boolean. SGBM median filter
#define TY_DEPTH_SGBM_LRC                    "DepthSgbmLRC"                                          ///< Boolean. SGBM LRC
#define TY_DEPTH_SGBM_LRC_DIFF               "DepthSgbmLRCDiff"                                      ///< Integer. SGBM LRC difference
#define TY_DEPTH_SGBM_MED_THRES              "DepthSgbmMedFilterThresh"                              ///< Integer. SGBM median filter threshold
#define TY_DEPTH_SGBM_P1_SCALE               "DepthSgbmSemiParamP1Scale"                             ///< Integer. SGBM semi param P1 scale
#define TY_DEPTH_MIN                         "DepthRangeMin"                                         ///< Integer. Minimum depth range
#define TY_DEPTH_MAX                         "DepthRangeMax"                                         ///< Integer. Maximum depth range
#define TY_DEPTH_SGBM_TEX_OFF                "DepthSgbmTextureFilterValueOffset"                     ///< Integer. Texture filter value offset
#define TY_DEPTH_SGBM_TEX_THRES              "DepthSgbmTextureFilterThreshold"                       ///< Integer. Texture filter threshold
#define TY_DEPTH_SGBM_TEX_EN                 "DepthSgbmTextureFilterEnable"                          ///< Boolean. Texture filter enable
#define TY_DEPTH_SGBM_TEX_WIN                "DepthSgbmTextureFilterWindowSize"                      ///< Integer. Texture filter window size
#define TY_DEPTH_SGBM_TEX_MAX                "DepthSgbmTextureFilterMaxDistance"                     ///< Boolean. Texture filter maximum distance
#define TY_DEPTH_SGBM_SAT_EN                 "DepthSgbmSaturateFilterEnable"                         ///< Boolean. Saturate filter enable
#define TY_DEPTH_SGBM_SAT_VAL                "DepthSgbmSaturateFilterVal"                            ///< Integer. Texture filter threshold
#define TY_DEPTH_SGBM_SAT_BLUR               "DepthSgbmSaturateFilterBlurSize"                       ///< Integer. Saturate filter blur size
#define TY_DEPTH_SGBM_SAT_DILATE             "DepthSgbmSaturateFilterDilateSize"                     ///< Integer. Saturate filter dilate size
#define TY_DEPTH_TOF_THRES                   "DepthStreamTofFilterThreshold"                         ///< Integer. ToF filter threshold
#define TY_DEPTH_TOF_CH                      "DepthStreamTofChannel"                                 ///< Integer. ToF channel
#define TY_DEPTH_TOF_MOD_THRES               "DepthStreamTofModulationThreshold"                     ///< Integer. ToF modulation threshold

/* DepthStreamTofDepthQuality */
#define TY_DEPTH_TOF_QUALITY                 "DepthStreamTofDepthQuality"                            ///< Enumeration. ToF depth quality
/* enumerations */
typedef enum TY_DEPTH_TOF_QUALITY_LIST :uint32_t
{
    DEPTH_TOF_QUALITY_LOW = 1,                                                                ///< Low depth quality
    DEPTH_TOF_QUALITY_MEDIUM = 2,                                                             ///< Medium depth quality
    DEPTH_TOF_QUALITY_HIGH = 4,                                                               ///< High depth quality
} TY_DEPTH_TOF_QUALITY_LIST;
static inline const char* TYGetDepthStreamTofDepthQualityName (uint32_t quality)
{
    switch (quality) 
    {
        case DEPTH_TOF_QUALITY_LOW:             return "DepthQualityLow";                                       ///< Low depth quality
        case DEPTH_TOF_QUALITY_MEDIUM:          return "DepthQualityMedium";                                    ///< Medium depth quality
        case DEPTH_TOF_QUALITY_HIGH:            return "DepthQualityHigh";                                      ///< High depth quality

        default: return "Unknown";
    }
}

#define TY_DEPTH_TOF_HDR                     "DepthStreamTofHdrRatio"                                ///< Integer. ToF HDR ratio
#define TY_DEPTH_TOF_JITTER                  "DepthStreamTofJitterThreshold"                         ///< Integer. ToF jitter threshold

/************************************************Feature for ToF Calib*************************************************/
#define TY_TOF_MOD                           "TofModParam"                                           ///< Integer. ToF modulation parameter
#define TY_TOF_FMOD0                         "TofFmodParam0"                                         ///< Integer. ToF FMOD parameter 0
#define TY_TOF_FMOD1                         "TofFmodParam1"                                         ///< Integer. ToF FMOD parameter 1
#define TY_TOF_DELAY0                        "TofLightDelayParam0"                                   ///< Integer. ToF light delay parameter 0
#define TY_TOF_DELAY1                        "TofLightDelayParam1"                                   ///< Integer. ToF light delay parameter 1
#define TY_TOF_DELAY2                        "TofLightDelayParam2"                                   ///< Integer. ToF light delay parameter 2
/*********************************************End of Feature for ToF Calib*********************************************/

/*************************************************Calibration Data Category********************************************/
#define TY_INTRIN_W                          "IntrinsicWidth"                                        ///< Integer. Intrinsic width
#define TY_INTRIN_H                          "IntrinsicHeight"                                       ///< Integer. Intrinsic height
#define TY_INTRIN                            "Intrinsic"                                             ///< Bytearray. Intrinsic parameters
#define TY_REC_INTRIN                        "Intrinsic2"                                            ///< Bytearray. Rectifyed Intrinsic parameters
#define TY_REC_ROT                           "Rotation"                                              ///< Bytearray. Rectified Rotation matrix
#define TY_DISTORT                           "Distortion"                                            ///< Bytearray. Distortion parameters
#define TY_EXTRIN                            "Extrinsic"                                             ///< Bytearray. Extrinsic parameters to Depth image
/******************************************End Of Calibration Data Category********************************************/

/*************************************************Transport Layer Category*********************************************/
#define TY_PAYLOAD                           "PayloadSize"                                           ///< Integer. Unit size of data payload in bytes
#define TY_GEV_IF_SEL                        "GevInterfaceSelector"                                  ///< Integer. Selects the network interface to control
#define TY_GEV_MAC                           "GevMACAddress"                                         ///< Integer. MAC address of the network interface
#define TY_GEV_IP_CFG_LLA                    "GevCurrentIPConfigurationLLA"                          ///< Boolean. Controls whether the Link Local Address IP configuration scheme is activated
#define TY_GEV_IP_CFG_DHCP                   "GevCurrentIPConfigurationDHCP"                         ///< Boolean. Controls whether the DHCP IP configuration scheme is activated
#define TY_GEV_IP_CFG_PERSIST                "GevCurrentIPConfigurationPersistentIP"                 ///< Boolean. Controls whether the Persistent IP configuration scheme is activated
#define TY_GEV_IP                            "GevCurrentIPAddress"                                   ///< Integer. Indicates the current IP address
#define TY_GEV_SUBNET                        "GevCurrentSubnetMask"                                  ///< Integer. Indicates the current subnet mask
#define TY_GEV_GATEWAY                       "GevCurrentDefaultGateway"                              ///< Integer. Indicates the current default gateway
#define TY_GEV_URL1                          "GevFirstURL"                                           ///< String. The first choice of URL for the XML device description file
#define TY_GEV_URL2                          "GevSecondURL"                                          ///< String. The second choice of URL for the XML device description file
#define TY_GEV_PERSIST_IP                    "GevPersistentIPAddress"                                ///< Integer. Indicates the persistent IP address
#define TY_GEV_PERSIST_SUBNET                "GevPersistentSubnetMask"                               ///< Integer. Indicates the persistent subnet mask
#define TY_GEV_PERSIST_GATEWAY               "GevPersistentDefaultGateway"                           ///< Integer. Indicates the persistent default gateway
#define TY_GEV_GVCP_ACK                      "GevGVCPPendingAck"                                     ///< Boolean. If enabled the device will issue a PENDING ACK in response to all GVCP commands
 
/* GevCCP */ 
#define TY_GEV_CCP                           "GevCCP"                                                ///< Enumeration. Controls the device access privilege of an application
/* enumerations */
typedef enum TY_GEV_CCP_LIST :uint32_t
{
    GEV_CCP_OPEN,                                                                             ///< Open Access
    GEV_CCP_EXCLUSIVE,                                                                        ///< Exclusive Access
    GEV_CCP_CONTROL,                                                                          ///< Control Access
} TY_GEV_CCP_LIST;
static inline const char* TYGetGevCCPName (uint32_t Access)
{
    switch (Access) 
    {
        case GEV_CCP_OPEN:                      return "OpenAccess";                                            ///< Open Access
        case GEV_CCP_EXCLUSIVE:                 return "ExclusiveAccess";                                       ///< Exclusive Access
        case GEV_CCP_CONTROL:                   return "ControlAccess";                                         ///< Control Access

        default: return "Unknown";
    }
}
 
#define TY_GEV_STREAM_CH_SEL                 "GevStreamChannelSelector"                              ///< Integer. Selects the stream channel to configure
#define TY_GEV_SCP_IF_IDX                    "GevSCPInterfaceIndex"                                  ///< Integer. Index of network interface to use
#define TY_GEV_SCP_PORT                      "GevSCPHostPort"                                        ///< Integer. Port to which the device must send data stream images
 
#define TY_GEV_SCP_DIR                       "GevSCPDirection"                                       ///< Integer. Indicates the direction of the data transfer
#define TY_GEV_SCP_PKT_SIZE                  "GevSCPSPacketSize"                                     ///< Integer. Indicates the packet size in bytes
#define TY_GEV_SCPD                          "GevSCPD"                                               ///< Integer. Indicates the delay in ticks inserted between each packet
#define TY_GEV_SCDA                          "GevSCDA"                                               ///< Integer. Indicates the destination IP address
#define TY_GEV_SCSP                          "GevSCSP"                                               ///< Integer. Indicates the destination port
/*****************************************End Of Transport Layer Category**********************************************/

/*******************************************************UserSet Category***********************************************/
#define TY_USER_SET_CUR                      "UserSetCurrent"                                        ///< Integer. Indicates the user set that is currently in use
 
/* UserSetSelector */ 
#define TY_USER_SET_SEL                      "UserSetSelector"                                       ///< Enumeration. Selects the feature User Set to load, save or configure
/* enumerations */
typedef enum TY_USER_SET_SEL_LIST :uint32_t
{
    USER_SET_SEL_DEFAULT0,                                                                    ///< Default user set 0
    USER_SET_SEL_DEFAULT1,                                                                    ///< Default user set 1
    USER_SET_SEL_DEFAULT2,                                                                    ///< Default user set 2
    USER_SET_SEL_DEFAULT3,                                                                    ///< Default user set 3
    USER_SET_SEL_DEFAULT4,                                                                    ///< Default user set 4
    USER_SET_SEL_DEFAULT5,                                                                    ///< Default user set 5
    USER_SET_SEL_DEFAULT6,                                                                    ///< Default user set 6
    USER_SET_SEL_DEFAULT7,                                                                    ///< Default user set 7
    USER_SET_SEL_USER_SET0,                                                                   ///< User set 0
    USER_SET_SEL_USER_SET1,                                                                   ///< User set 1
    USER_SET_SEL_USER_SET2,                                                                   ///< User set 2
    USER_SET_SEL_USER_SET3,                                                                   ///< User set 3
    USER_SET_SEL_USER_SET4,                                                                   ///< User set 4
    USER_SET_SEL_USER_SET5,                                                                   ///< User set 5
    USER_SET_SEL_USER_SET6,                                                                   ///< User set 6
    USER_SET_SEL_USER_SET7,                                                                   ///< User set 7
} TY_USER_SET_SEL_LIST;
static inline const char* TYGetUserSetSelectorName (uint32_t sel)
{
    switch (sel) 
    {
        case USER_SET_SEL_DEFAULT0:             return "Default0";                                              ///< Default user set 0
        case USER_SET_SEL_DEFAULT1:             return "Default1";                                              ///< Default user set 1
        case USER_SET_SEL_DEFAULT2:             return "Default2";                                              ///< Default user set 2
        case USER_SET_SEL_DEFAULT3:             return "Default3";                                              ///< Default user set 3
        case USER_SET_SEL_DEFAULT4:             return "Default4";                                              ///< Default user set 4
        case USER_SET_SEL_DEFAULT5:             return "Default5";                                              ///< Default user set 5
        case USER_SET_SEL_DEFAULT6:             return "Default6";                                              ///< Default user set 6
        case USER_SET_SEL_DEFAULT7:             return "Default7";                                              ///< Default user set 7
        case USER_SET_SEL_USER_SET0:            return "UserSet0";                                              ///< User set 0
        case USER_SET_SEL_USER_SET1:            return "UserSet1";                                              ///< User set 1
        case USER_SET_SEL_USER_SET2:            return "UserSet2";                                              ///< User set 2
        case USER_SET_SEL_USER_SET3:            return "UserSet3";                                              ///< User set 3
        case USER_SET_SEL_USER_SET4:            return "UserSet4";                                              ///< User set 4
        case USER_SET_SEL_USER_SET5:            return "UserSet5";                                              ///< User set 5
        case USER_SET_SEL_USER_SET6:            return "UserSet6";                                              ///< User set 6
        case USER_SET_SEL_USER_SET7:            return "UserSet7";                                              ///< User set 7

        default: return "Unknown";
    }
}

#define TY_USER_SET_DESC                     "UserSetDescription"                                    ///< String. User set description
#define TY_USER_SET_LOAD                     "UserSetLoad"                                           ///< Command. Loads the User Set specified by UserSetSelector to the device
#define TY_USER_SET_SAVE                     "UserSetSave"                                           ///< Command. Save the User Set specified by UserSetSelector to the non-volatile memory of the device
 
/* UserSetDefault */ 
#define TY_USER_SET_DEF                      "UserSetDefault"                                        ///< Enumeration. Indicates which User Set is used as default startup set
/* enumerations */
/*************************************************End Of UserSet Category**********************************************/
 
/* Source control related features */ 
#define TY_SRC_COUNT                         "SourceCount"                                           ///< Integer. Controls or returns the number of sources supported by the device
 
/* SourceSelector */ 
#define TY_SRC_SEL                           "SourceSelector"                                        ///< Enumeration. Selects the source to control
/* enumerations */
typedef enum TY_SRC_SEL_LIST :uint32_t
{
    SRC_SEL_DEPTH,                                                                            ///< Depth source
    SRC_SEL_TEXTURE,                                                                          ///< Texture source
    SRC_SEL_LEFT,                                                                             ///< Left binocular source
    SRC_SEL_RIGHT,                                                                            ///< Right binocular source
} TY_SRC_SEL_LIST;
static inline const char* TYGetSourceSelectorName (uint32_t sel)
{
    switch (sel) 
    {
        case SRC_SEL_DEPTH:                     return "Depth";                                                 ///< Depth source
        case SRC_SEL_TEXTURE:                   return "Texture";                                               ///< Texture source
        case SRC_SEL_LEFT:                      return "Left";                                                  ///< Left binocular source
        case SRC_SEL_RIGHT:                     return "Right";                                                 ///< Right binocular source

        default: return "Unknown";
    }
}
 
#define TY_SRC_ID                            "SourceIDValue"                                         ///< Integer. Source ID value
 
/* Other features */ 
#define TY_CAP_TIME_STAT                     "CaptureTimeStatistic"                                  ///< Integer. Capture time statistic
#define TY_IR_UNDIST                         "IRUndistortion"                                        ///< Boolean. IR undistortion
#define TY_POST_PROC0                        "PostProcessParams0"                                    ///< Bytearray. Post process parameters 0
#define TY_POST_PROC1                        "PostProcessParams1"                                    ///< Bytearray. Post process parameters 1
 

/****************************************************Light Source Category*********************************************/
#define TY_LIGHT_SEL                         "LightControllerSelector"                          ///< Enumeration. Selects the light to control
#define TY_LIGHT_EN                          "LightEnable"                                      ///< Boolean. Enable/Disable the Light
#define TY_LIGHT_BRIGHTNESS                  "LightBrightness"                                  ///< Integer. Config the Light Brightness
/* enumerations */
typedef enum TY_LIGHT_SEL_LIST :uint32_t
{
    LIGHT_SEL_LASER_0,                                                                          ///< Laser controller will effects depth
    LIGHT_SEL_FLOOD_0,                                                                          ///< Flood controller always effects  Left/Right cam when needed
    LIGHT_SEL_TEXTURE_FLOOD,                                                                    ///< Flood controller effects  Texture cam
} TY_LIGTH_SEL_LIST;

static inline const char* TYGetLightSelectorName (uint32_t sel)
{
    switch (sel)
    {
        case LIGHT_SEL_LASER_0:               return "LightController0";                    ///< Laser controller will effects depth
        case LIGHT_SEL_FLOOD_0:               return "LightController1";                    ///< Flood controller always effects  Left/Right cam when needed
        case LIGHT_SEL_TEXTURE_FLOOD:         return "LightController2";                    ///< Flood controller effects  Texture cam

        default: return "Unknown";
    }
}
/**********************************************End Of Light Source Category********************************************/

#endif // TY_FEATURE_LIST_H
