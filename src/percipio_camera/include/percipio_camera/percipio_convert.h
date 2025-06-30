/*
 * @Description: 
 * @Author: zxy
 * @Date: 2023-08-04 14:00:15
 * @LastEditors: zxy
 * @LastEditTime: 2023-08-04 14:32:44
 */
#ifndef PERCIPIO_CONVERT_H_
#define PERCIPIO_CONVERT_H_

#include "percipio_camera/percipio_device_info.h"
#include "percipio_camera/percipio_video_mode.h"

#include "TYApi.h"
#include "percipio_interface.h"

#include <vector>

namespace percipio_wrapper
{

const PercipioDeviceInfo percipio_convert(const percipio::DeviceInfo* pInfo);

const PercipioVideoMode percipio_convert(const percipio::VideoMode& input);
const percipio::VideoMode percipio_convert(const PercipioVideoMode& input);

const std::vector<PercipioVideoMode> percipio_convert(const percipio::Array<percipio::VideoMode>& input);
}

#endif
