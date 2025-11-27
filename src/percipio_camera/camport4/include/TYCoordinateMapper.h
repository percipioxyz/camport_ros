/**@file TYCoordinateMapper.h
 * @brief Coordinate Conversion API
 * @note  Considering performance, we leave the responsibility of parameters check to users.
 * @copyright  Copyright(C)2016-2018 Percipio All Rights Reserved
 **/
#ifndef TY_COORDINATE_MAPPER_H_
#define TY_COORDINATE_MAPPER_H_

#include <stdlib.h>
#include "TYApi.h"

typedef struct TY_PIXEL_DESC
{
  int16_t x;      // x coordinate in pixels
  int16_t y;      // y coordinate in pixels
  uint16_t depth; // depth value
  uint16_t rsvd;
}TY_PIXEL_DESC;

typedef struct TY_PIXEL_COLOR_DESC
{
	int16_t x;			// x coordinate in pixels
	int16_t y;			// y coordinate in pixels
	uint8_t bgr_ch1;	// color info <channel 1>
	uint8_t bgr_ch2;	// color info <channel 2>
	uint8_t bgr_ch3;	// color info <channel 3>
	uint8_t rsvd;
}TY_PIXEL_COLOR_DESC;

// ------------------------------
//  base convertion
// ------------------------------

/// @brief Calculate 4x4 extrinsic matrix's inverse matrix.
/// @param  [in]  orgExtrinsic          Input extrinsic matrix.
/// @param  [out] invExtrinsic          Inverse matrix.
/// @retval TY_STATUS_OK        Succeed.
/// @retval TY_STATUS_ERROR     Calculation failed.
TY_CAPI   TYInvertExtrinsic         (const TY_CAMERA_EXTRINSIC* orgExtrinsic,
                                     TY_CAMERA_EXTRINSIC* invExtrinsic);

/// @brief Map pixels on depth image to 3D points.
/// @param  [in]  src_calib             Depth image's calibration data.
/// @param  [in]  depthW                Width of depth image.
/// @param  [in]  depthH                Height of depth image.
/// @param  [in]  depthPixels           Pixels on depth image.
/// @param  [in]  count                 Number of depth pixels.
/// @param  [out] point3d               Output point3D.
/// @retval TY_STATUS_OK        Succeed.
TY_CAPI   TYMapDepthToPoint3d       (const TY_CAMERA_CALIB_INFO* src_calib,
                                     uint32_t depthW, uint32_t depthH,
                                     const TY_PIXEL_DESC* depthPixels, uint32_t count,
                                     TY_VECT_3F* point3d,
                                     float f_scale_unit = 1.0f);

/// @brief Map 3D points to pixels on depth image. Reverse operation of TYMapDepthToPoint3d.
/// @param  [in]  dst_calib             Target depth image's calibration data.
/// @param  [in]  point3d               Input 3D points.
/// @param  [in]  count                 Number of points.
/// @param  [in]  depthW                Width of target depth image.
/// @param  [in]  depthH                Height of target depth image.
/// @param  [out] depth                 Output depth pixels.
/// @retval TY_STATUS_OK        Succeed.
TY_CAPI   TYMapPoint3dToDepth       (const TY_CAMERA_CALIB_INFO* dst_calib,
                                     const TY_VECT_3F* point3d, uint32_t count,
                                     uint32_t depthW, uint32_t depthH,
                                     TY_PIXEL_DESC* depth,
                                     float f_scale_unit = 1.0f);

/// @brief Map depth image to 3D points. 0 depth pixels maps to (NAN, NAN, NAN).
/// @param  [in]  src_calib             Depth image's calibration data.
/// @param  [in]  depthW                Width of depth image.
/// @param  [in]  depthH                Height of depth image.
/// @param  [in]  depth                 Depth image.
/// @param  [out] point3d               Output point3D image.
/// @retval TY_STATUS_OK        Succeed.
TY_CAPI   TYMapDepthImageToPoint3d  (const TY_CAMERA_CALIB_INFO* src_calib,
                                     int32_t imageW, int32_t imageH,
                                     const uint16_t* depth,
                                     TY_VECT_3F* point3d, 
                                     float f_scale_unit = 1.0f);

/// @brief Map 3D points to depth image. (NAN, NAN, NAN) will be skipped.
/// @param  [in]  dst_calib             Target depth image's calibration data.
/// @param  [in]  point3d               Input 3D points.
/// @param  [in]  count                 Number of points.
/// @param  [in]  depthW                Width of target depth image.
/// @param  [in]  depthH                Height of target depth image.
/// @param  [in,out] depth              Depth image buffer.
/// @retval TY_STATUS_OK        Succeed.
TY_CAPI   TYMapPoint3dToDepthImage  (const TY_CAMERA_CALIB_INFO* dst_calib,
                                     const TY_VECT_3F* point3d, uint32_t count,
                                     uint32_t depthW, uint32_t depthH, uint16_t* depth,
                                     float f_target_scale = 1.0f);

/// @brief Map 3D points to another coordinate.
/// @param  [in]  extrinsic             Extrinsic matrix.
/// @param  [in]  point3dFrom           Source 3D points.
/// @param  [in]  count                 Number of source 3D points.
/// @param  [out] point3dTo             Target 3D points.
/// @retval TY_STATUS_OK        Succeed.
TY_CAPI   TYMapPoint3dToPoint3d     (const TY_CAMERA_EXTRINSIC* extrinsic,
                                     const TY_VECT_3F* point3dFrom, int32_t count,
                                     TY_VECT_3F* point3dTo);

/// @brief Map depth pixels to color coordinate pixels.
/// @param  [in]  depth_calib           Depth image's calibration data.
/// @param  [in]  depthW                Width of current depth image.
/// @param  [in]  depthH                Height of current depth image.
/// @param  [in]  depth                 Depth image pixels.
/// @param  [in]  count                 Number of depth image pixels.
/// @param  [in]  color_calib           Color image's calibration data.
/// @param  [in]  mappedW               Width of target depth image.
/// @param  [in]  mappedH               Height of target depth image.
/// @param  [out] mappedDepth           Output pixels.
/// @retval TY_STATUS_OK        Succeed.
TY_CAPI TYMapDepthToColorCoordinate(
                  const TY_CAMERA_CALIB_INFO* depth_calib,
                  uint32_t depthW, uint32_t depthH,
                  const TY_PIXEL_DESC* depth, uint32_t count,
                  const TY_CAMERA_CALIB_INFO* color_calib,
                  uint32_t mappedW, uint32_t mappedH,
                  TY_PIXEL_DESC* mappedDepth,
                  float f_scale_unit = 1.0f);

/// @brief Map original depth image to color coordinate depth image.
/// @param  [in]  depth_calib           Depth image's calibration data.
/// @param  [in]  depthW                Width of current depth image.
/// @param  [in]  depthH                Height of current depth image.
/// @param  [in]  depth                 Depth image.
/// @param  [in]  color_calib           Color image's calibration data.
/// @param  [in]  mappedW               Width of target depth image.
/// @param  [in]  mappedH               Height of target depth image.
/// @param  [out] mappedDepth           Output pixels.
/// @retval TY_STATUS_OK        Succeed.
TY_CAPI TYMapDepthImageToColorCoordinate(
                  const TY_CAMERA_CALIB_INFO* depth_calib,
                  uint32_t depthW, uint32_t depthH, const uint16_t* depth,
                  const TY_CAMERA_CALIB_INFO* color_calib,
                  uint32_t mappedW, uint32_t mappedH, uint16_t* mappedDepth, 
                  float f_scale_unit = 1.0f);


/// @brief Map original RGB pixels to depth coordinate.
/// @param  [in]  depth_calib           Depth image's calibration data.
/// @param  [in]  depthW                Width of current depth image.
/// @param  [in]  depthH                Height of current depth image.
/// @param  [in]  depth                 Current depth image.
/// @param  [in]  color_calib           Color image's calibration data.
/// @param  [in]  rgbW                  Width of RGB image.
/// @param  [in]  rgbH                  Height of RGB image.
/// @param  [in]  src                   Input RGB pixels info.
/// @param  [in]  cnt                   Input src RGB pixels cnt
/// @param  [in]  min_distance          The min distance(mm), which is generally set to the minimum measured distance of the current camera
/// @param  [in]  max_distance          The longest distance(mm), which is generally set to the longest measuring distance of the current camera
/// @param  [out] dst                   Output RGB pixels info.
/// @retval TY_STATUS_OK                Succeed.
TY_CAPI TYMapRGBPixelsToDepthCoordinate(
	const TY_CAMERA_CALIB_INFO* depth_calib,
	uint32_t depthW, uint32_t depthH, const uint16_t* depth,
	const TY_CAMERA_CALIB_INFO* color_calib,
	uint32_t rgbW, uint32_t rgbH,
	TY_PIXEL_COLOR_DESC* src, uint32_t cnt,
	uint32_t   min_distance,
	uint32_t   max_distance,
	TY_PIXEL_COLOR_DESC* dst,
	float f_scale_unit = 1.0f);

/// @brief Map original RGB image to depth coordinate RGB image.
/// @param  [in]  depth_calib           Depth image's calibration data.
/// @param  [in]  depthW                Width of current depth image.
/// @param  [in]  depthH                Height of current depth image.
/// @param  [in]  depth                 Current depth image.
/// @param  [in]  color_calib           Color image's calibration data.
/// @param  [in]  rgbW                  Width of RGB image.
/// @param  [in]  rgbH                  Height of RGB image.
/// @param  [in]  inRgb                 Current RGB image.
/// @param  [out] mappedRgb             Output RGB image.
/// @retval TY_STATUS_OK        Succeed.
TY_CAPI TYMapRGBImageToDepthCoordinate(
                  const TY_CAMERA_CALIB_INFO* depth_calib,
                  uint32_t depthW, uint32_t depthH, const uint16_t* depth,
                  const TY_CAMERA_CALIB_INFO* color_calib,
                  uint32_t rgbW, uint32_t rgbH, const uint8_t* inRgb,
                  uint8_t* mappedRgb,
                  float f_scale_unit = 1.0f);

/// @brief Map original RGB48 image to depth coordinate RGB image.
/// @param  [in]  depth_calib           Depth image's calibration data.
/// @param  [in]  depthW                Width of current depth image.
/// @param  [in]  depthH                Height of current depth image.
/// @param  [in]  depth                 Current depth image.
/// @param  [in]  color_calib           Color image's calibration data.
/// @param  [in]  rgbW                  Width of RGB48 image.
/// @param  [in]  rgbH                  Height of RGB48 image.
/// @param  [in]  inRgb                 Current RGB48 image.
/// @param  [out] mappedRgb             Output RGB48 image.
/// @retval TY_STATUS_OK        Succeed.
TY_CAPI TYMapRGB48ImageToDepthCoordinate(
                  const TY_CAMERA_CALIB_INFO* depth_calib,
                  uint32_t depthW, uint32_t depthH, const uint16_t* depth,
                  const TY_CAMERA_CALIB_INFO* color_calib,
                  uint32_t rgbW, uint32_t rgbH, const uint16_t* inRgb,
                  uint16_t* mappedRgb, 
                  float f_scale_unit = 1.0f);

/// @brief Map original MONO16 image to depth coordinate MONO16 image.
/// @param  [in]  depth_calib           Depth image's calibration data.
/// @param  [in]  depthW                Width of current depth image.
/// @param  [in]  depthH                Height of current depth image.
/// @param  [in]  depth                 Current depth image.
/// @param  [in]  color_calib           Color image's calibration data.
/// @param  [in]  rgbW                  Width of MONO16 image.
/// @param  [in]  rgbH                  Height of MONO16 image.
/// @param  [in]  gray                  Current MONO16 image.
/// @param  [out] mappedGray            Output MONO16 image.
/// @retval TY_STATUS_OK        Succeed.
TY_CAPI TYMapMono16ImageToDepthCoordinate(
                  const TY_CAMERA_CALIB_INFO* depth_calib,
                  uint32_t depthW, uint32_t depthH, const uint16_t* depth,
                  const TY_CAMERA_CALIB_INFO* color_calib,
                  uint32_t rgbW, uint32_t rgbH, const uint16_t* gray,
                  uint16_t* mappedGray, 
                  float f_scale_unit = 1.0f);
                  

/// @brief Map original MONO8 image to depth coordinate MONO8 image.
/// @param  [in]  depth_calib           Depth image's calibration data.
/// @param  [in]  depthW                Width of current depth image.
/// @param  [in]  depthH                Height of current depth image.
/// @param  [in]  depth                 Current depth image.
/// @param  [in]  color_calib           Color image's calibration data.
/// @param  [in]  monoW                 Width of MONO8 image.
/// @param  [in]  monoH                 Height of MONO8 image.
/// @param  [in]  inMono                Current MONO8 image.
/// @param  [out] mappedMono            Output MONO8 image.
/// @retval TY_STATUS_OK        Succeed.
TY_CAPI TYMapMono8ImageToDepthCoordinate(
                  const TY_CAMERA_CALIB_INFO* depth_calib,
                  uint32_t depthW, uint32_t depthH, const uint16_t* depth,
                  const TY_CAMERA_CALIB_INFO* color_calib,
                  uint32_t monoW, uint32_t monoH, const uint8_t* inMono,
                  uint8_t* mappedMono,
                  float f_scale_unit = 1.0f);

#endif
