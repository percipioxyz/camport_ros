#ifndef TY_PARAMETER_H
#define TY_PARAMETER_H

#include "TYApi.h"

enum ParamType {
  Command,
  Integer,
  Float,
  Boolean,
  Enumeration,
  String,
  ByteArray,
};

struct TYEnumEntry
{
    int32_t value;
    char name[64];
    char tooltip[512];
    char description[512];
    char displayName[512];
};

TY_CAPI  TYParamGetToolTip              (TY_DEV_HANDLE hDevice, const char* feat, char* pBuffer, uint32_t bufferSize);
TY_CAPI  TYParamGetDescriptor           (TY_DEV_HANDLE hDevice, const char* feat, char* pBuffer, uint32_t bufferSize);
TY_CAPI  TYParamGetDisplayName          (TY_DEV_HANDLE hDevice, const char* feat, char* pBuffer, uint32_t bufferSize);
TY_CAPI  TYParamGetType                 (TY_DEV_HANDLE hDevice, const char* feat, ParamType* type);
TY_CAPI  TYParamGetAccess               (TY_DEV_HANDLE hDevice, const char* feat, TY_ACCESS_MODE* access);
TY_CAPI  TYParamGetVisibility           (TY_DEV_HANDLE hDevice, const char* feat, TY_VISIBILITY_TYPE* visibility);

//Command
TY_CAPI  TYCommandExec                  (TY_DEV_HANDLE hDevice, const char* feat);

//Integer
TY_CAPI  TYIntegerSetValue              (TY_DEV_HANDLE hDevice, const char* feat, int64_t value);
TY_CAPI  TYIntegerGetValue              (TY_DEV_HANDLE hDevice, const char* feat, int64_t* value);
TY_CAPI  TYIntegerGetMin                (TY_DEV_HANDLE hDevice, const char* feat, int64_t* min);
TY_CAPI  TYIntegerGetMax                (TY_DEV_HANDLE hDevice, const char* feat, int64_t* max);
TY_CAPI  TYIntegerGetStep               (TY_DEV_HANDLE hDevice, const char* feat, int64_t* step);
TY_CAPI  TYIntegerGetUnit               (TY_DEV_HANDLE hDevice, const char* feat, char* pBuffer, uint32_t bufferSize);

//Float
TY_CAPI  TYFloatSetValue                (TY_DEV_HANDLE hDevice, const char* feat, double value);
TY_CAPI  TYFloatGetValue                (TY_DEV_HANDLE hDevice, const char* feat, double* value);
TY_CAPI  TYFloatGetMin                  (TY_DEV_HANDLE hDevice, const char* feat, double* min);
TY_CAPI  TYFloatGetMax                  (TY_DEV_HANDLE hDevice, const char* feat, double* max);
TY_CAPI  TYFloatGetStep                 (TY_DEV_HANDLE hDevice, const char* feat, double* step);
TY_CAPI  TYFloatGetUnit                 (TY_DEV_HANDLE hDevice, const char* feat, char* pBuffer, uint32_t bufferSize);

//Boolean
TY_CAPI  TYBooleanSetValue              (TY_DEV_HANDLE hDevice, const char* feat, bool value);
TY_CAPI  TYBooleanGetValue              (TY_DEV_HANDLE hDevice, const char* feat, bool* value);

//Enumeration
TY_CAPI  TYEnumSetValue                 (TY_DEV_HANDLE hDevice, const char* feat, int32_t value);
TY_CAPI  TYEnumSetString                (TY_DEV_HANDLE hDevice, const char* feat, const char* name);
TY_CAPI  TYEnumGetValue                 (TY_DEV_HANDLE hDevice, const char* feat, int32_t* value);
TY_CAPI  TYEnumGetString                (TY_DEV_HANDLE hDevice, const char* feat, char* name, const uint32_t length);
TY_CAPI  TYEnumGetEntryCount            (TY_DEV_HANDLE hDevice, const char* feat, uint32_t* cnt);
TY_CAPI  TYEnumGetEntryInfo             (TY_DEV_HANDLE hDevice, const char* feat, TYEnumEntry* pEnumEntry, uint32_t entryCount, uint32_t* pFilledEntryCount);

//String
TY_CAPI  TYStringSetValue               (TY_DEV_HANDLE hDevice, const char* feat, const char* pBuffer);
TY_CAPI  TYStringGetLength              (TY_DEV_HANDLE hDevice, const char* feat, uint32_t* pLength);
TY_CAPI  TYStringGetValue               (TY_DEV_HANDLE hDevice, const char* feat, char* pBuffer, uint32_t bufferSize);

//ByteArray
TY_CAPI  TYByteArrayGetSize             (TY_DEV_HANDLE hDevice, const char* feat, uint32_t* pSize);
TY_CAPI  TYByteArraySetValue            (TY_DEV_HANDLE hDevice, const char* feat, const uint8_t* pBuffer, uint32_t bufferSize);
TY_CAPI  TYByteArrayGetValue            (TY_DEV_HANDLE hDevice, const char* feat, uint8_t* buffer, uint32_t bufferSize);

#endif
