/*
 * @Description: 
 * @Author: zxy
 * @Date: 2023-07-18 15:55:24
 * @LastEditors: zxy
 * @LastEditTime: 2024-05-30 15:01:39
 */

#include <ros/ros.h>
#include <iostream>
#include <sstream>
#include <fstream>
#include <map>
#include <cmath>
#include <numeric>
#include <algorithm>
//#include <math.h>
#include <iostream>
#include <vector>
#include "percipio_interface.h"
#include "TYApi.h"
#include "TYImageProc.h"
#include "TYCoordinateMapper.h"

#define MAX_DEPTH 0x10000 

static void BGRToRGB(const void* bgrFrame, int width, int height, void* rgbFrame)
{
  uint8_t* pBGR = (uint8_t*)bgrFrame;
  uint8_t* pRGB = (uint8_t*)rgbFrame;

  uint8_t r, g, b;
  for (int i = 0; i < height * width; i++)
  {
    b = *pBGR++;
    g = *pBGR++;
    r = *pBGR++;
		
    *pRGB++ = r;
    *pRGB++ = g;
    *pRGB++ = b;
  }
}

class ImgProc 
{
public:
    static inline void parseXYZ48(int16_t* src, int16_t* dst, int width, int height, float f_scale_unit)
    {
      for (int pix = 0; pix < width*height; pix++) {
          dst[pix] =(int16_t)(*(src + 3*pix + 2) * f_scale_unit + 0.5f);
      }
    }

    static int doDepthUndistortion(const TY_CAMERA_CALIB_INFO* depth_calib,
                                percipio::VideoFrameData* src, 
                                percipio::VideoFrameData* dst)
    {
      if(src->getPixelFormat() != TYPixelFormatCoord3D_C16) {
        ROS_ERROR("%s <%d> Invalid pixel format!\n", __FILE__, __LINE__);
        return-1;
      }

      dst->setTimestamp(src->getTimestamp());
      dst->setFrameIndex(src->getFrameIndex());
      dst->setComponentID(src->getComponentID());
      dst->setWidth(src->getWidth());
      dst->setHeight(src->getHeight());
      dst->setPixelFormat(src->getPixelFormat());
      dst->Resize(src->getDataSize());

      TY_IMAGE_DATA src_image, dst_image;
      src_image.width  = src->getWidth();
      src_image.height = src->getHeight();
      src_image.pixelFormat = src->getPixelFormat();
      src_image.buffer = src->getData();

      dst_image.width  = dst->getWidth();
      dst_image.height = dst->getHeight();
      dst_image.pixelFormat = dst->getPixelFormat();
      dst_image.buffer = dst->getData();

      TY_STATUS err = TYUndistortImage(depth_calib, &src_image, nullptr, &dst_image);
      if(err) {
        ROS_WARN("TYUndistortImage fail!");
      }
      return err;
    }

    static int doRGBUndistortion(const TY_CAMERA_CALIB_INFO* color_calib, 
                                percipio::VideoFrameData* src, 
                                percipio::VideoFrameData* dst)
    {
      if(src->getPixelFormat() != TYPixelFormatRGB8 &&
         src->getPixelFormat() != TYPixelFormatMono8) {
        ROS_ERROR("%s <%d> Invalid pixel format!\n", __FILE__, __LINE__);
        return-1;
      }

      dst->setTimestamp(src->getTimestamp());
      dst->setFrameIndex(src->getFrameIndex());
      dst->setComponentID(src->getComponentID());
      dst->setWidth(src->getWidth());
      dst->setHeight(src->getHeight());
      dst->setPixelFormat(src->getPixelFormat());
      dst->Resize(src->getDataSize());

      TY_IMAGE_DATA src_image, dst_image;
      src_image.width  = src->getWidth();
      src_image.height = src->getHeight();
      src_image.pixelFormat = src->getPixelFormat();
      src_image.buffer = src->getData();

      dst_image.width  = dst->getWidth();
      dst_image.height = dst->getHeight();
      dst_image.pixelFormat = dst->getPixelFormat();
      dst_image.buffer = dst->getData();

      TY_STATUS err = TYUndistortImage(color_calib, &src_image, nullptr, &dst_image);
      if(err) {
        ROS_WARN("TYUndistortImage fail!");
      }
      return err;
    }

    static int MapDepthImageToColorCoordinate(const TY_CAMERA_CALIB_INFO* depth_calib, 
                                              const TY_CAMERA_CALIB_INFO* color_calib, 
                                              int target_width,
                                              int target_height,
                                              percipio::VideoFrameData* src, 
                                              percipio::VideoFrameData* dst,
                                              float f_scale_unit)
    {
      if(src->getPixelFormat() != TYPixelFormatCoord3D_C16) {
        ROS_ERROR("%s <%d> Invalid pixel format!\n", __FILE__, __LINE__);
        return-1;
      }

      dst->setTimestamp(src->getTimestamp());
      dst->setFrameIndex(src->getFrameIndex());
      dst->setPixelFormat(src->getPixelFormat());
      if((target_width != 0) && (target_height != 0)) {
        dst->setWidth(target_width);
        dst->setHeight(target_height);
        dst->Resize(2 * target_width * target_height);
      } else {
        dst->setWidth(src->getWidth());
        dst->setHeight(src->getHeight());
        dst->Resize(2 * src->getWidth() * src->getHeight());
      }

      return TYMapDepthImageToColorCoordinate(
                  depth_calib,
                  src->getWidth(), src->getHeight(), (const uint16_t*)src->getData(),
                  color_calib,
                  dst->getWidth(), dst->getHeight(), (uint16_t* )dst->getData(),
                  f_scale_unit);
    }

    static int MapXYZ48ToColorCoordinate(const TY_CAMERA_CALIB_INFO* depth_calib, 
                                              const TY_CAMERA_CALIB_INFO* color_calib, 
                                              int target_width,
                                              int target_height,
                                              percipio::VideoFrameData* src,
                                              percipio::VideoFrameData* dst,
                                              float f_scale_unit)
    {
      if(src->getPixelFormat() != TYPixelFormatCoord3D_ABC16) {
        ROS_ERROR("%s <%d> Invalid pixel format!\n", __FILE__, __LINE__);
        return-1;
      }

      uint32_t points_cnt = src->getWidth() * src->getHeight();
      int16_t* xyz48 = static_cast<int16_t*>(src->getData());

      std::vector<TY_VECT_3F> p3d(points_cnt);
      for(int i = 0; i < points_cnt; i++) {
        if(xyz48[3 * i + 2]) {
          p3d[i].x = static_cast<float>(xyz48[3 * i + 0]);
          p3d[i].y = static_cast<float>(xyz48[3 * i + 1]);
          p3d[i].z = static_cast<float>(xyz48[3 * i + 2]);
        } else {
          p3d[i].x = std::numeric_limits<float>::quiet_NaN();
          p3d[i].y = std::numeric_limits<float>::quiet_NaN();
          p3d[i].z = std::numeric_limits<float>::quiet_NaN();
        }        
      }

      TY_CAMERA_EXTRINSIC extri_inv;
      TYInvertExtrinsic(&color_calib->extrinsic, &extri_inv);
      TYMapPoint3dToPoint3d(&extri_inv, &p3d[0], points_cnt, &p3d[0]);
      
      dst->setTimestamp(src->getTimestamp());
      dst->setFrameIndex(src->getFrameIndex());
      dst->setPixelFormat(TYPixelFormatCoord3D_C16);
      if((target_width != 0) && (target_height != 0)) {
        dst->setWidth(target_width);
        dst->setHeight(target_height);
        dst->Resize(2 * target_width * target_height);
      } else {
        dst->setWidth(src->getWidth());
        dst->setHeight(src->getHeight());
        dst->Resize(2 * src->getWidth() * src->getHeight());
      }
      
      return TYMapPoint3dToDepthImage(color_calib, &p3d[0], points_cnt, dst->getWidth(), dst->getHeight(), (uint16_t*)dst->getData());
    }

    static int cvtColor(percipio::VideoFrameData& src, percipio::VideoFrameData& dst)
    {
      int i, j;
      TYImageInfo src_img;
      src_img.width  = src.getWidth();
      src_img.height = src.getHeight();
      src_img.format = src.getPixelFormat();
      src_img.dataSize = src.getDataSize();
      src_img.data = src.getData();

      uint32_t totalSize = 0;
      TYDecodeError ret = TYGetDecodeBufferSize(&src_img, &totalSize);
      if(ret != TY_DECODE_SUCCESS) {
        return ret;
      }
      
      dst.setTimestamp(src.getTimestamp());
      dst.setWidth(src.getWidth());
      dst.setHeight(src.getHeight());
      dst.setFrameIndex(src.getFrameIndex());
      dst.setComponentID(src.getComponentID());
      dst.setPixelFormat(TYPixelFormatRGB8);
      dst.Resize(totalSize);

      TYDecodeResult retInfo;
      return TYDecodeImage(&src_img,  TY_OUTPUT_FORMAT_AUTO, (void*)dst.getData(), totalSize, &retInfo);
    }
};