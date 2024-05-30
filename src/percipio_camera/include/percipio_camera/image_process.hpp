/*
 * @Description: 
 * @Author: zxy
 * @Date: 2023-07-18 15:55:24
 * @LastEditors: zxy
 * @LastEditTime: 2024-05-30 15:01:39
 */

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

#include <opencv2/opencv.hpp>

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

class distortion_data {
  public:
    distortion_data(const TY_CAMERA_CALIB_INFO& calib, const cv::Mat& x, const cv::Mat& y);
    distortion_data(const distortion_data& data);
    ~distortion_data();

    const int get_map_width() const;
    const int get_map_height() const;
    const cv::Mat& get_map_x() const;
    const cv::Mat& get_map_y() const;
    const TY_CAMERA_CALIB_INFO& get_calib_data() const;

  private:
    TY_CAMERA_CALIB_INFO calib_data;
    int map_size_width;
    int map_size_height;
    cv::Mat map_x;
    cv::Mat map_y;
};

distortion_data::distortion_data(const TY_CAMERA_CALIB_INFO& calib, const cv::Mat& x, const cv::Mat& y)
{
  if((map_x.cols != map_y.cols) || 
     (map_x.rows != map_y.rows))
  {
    ROS_ERROR("Invalid distortion map");
    return;
  }

  calib_data = calib;
  map_x = x.clone();
  map_y = y.clone();
  map_size_width = map_x.cols;
  map_size_height = map_x.rows;
}

distortion_data::distortion_data(const distortion_data& data)
{
  calib_data = data.get_calib_data();
  map_x = data.get_map_x().clone();
  map_y = data.get_map_y().clone();

  map_size_width = map_x.cols;
  map_size_height = map_x.rows;
}

distortion_data::~distortion_data()
{
  //
}

const TY_CAMERA_CALIB_INFO& distortion_data::get_calib_data() const
{
  return calib_data;
}

const cv::Mat& distortion_data::get_map_x() const
{
  return map_x;
}

const cv::Mat& distortion_data::get_map_y() const
{
  return map_y;
}

const int distortion_data::get_map_width() const
{
  return map_size_width;
}

const int distortion_data::get_map_height() const
{
  return map_size_height;
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
    static inline int decodeCsiRaw10(uint8_t* src, unsigned short* dst, int width, int height)
    {
        if(width & 0x3) {
            return -1;
        }
    
        int raw10_line_size = 5 * width / 4;
        for(size_t i = 0, j = 0; i < raw10_line_size * height; i+=5, j+=4)
        {
            //[A2 - A9] | [B2 - B9] | [C2 - C9] | [D2 - D9] | [A0A1-B0B1-C0C1-D0D1]
            dst[j + 0] = ((uint16_t)src[i + 0] << 2) | ((src[i + 4] & 0x3)  >> 0);
            dst[j + 1] = ((uint16_t)src[i + 1] << 2) | ((src[i + 4] & 0xc)  >> 2);
            dst[j + 2] = ((uint16_t)src[i + 2] << 2) | ((src[i + 4] & 0x30) >> 4);
            dst[j + 3] = ((uint16_t)src[i + 3] << 2) | ((src[i + 4] & 0xc0) >> 6);
        }
        return 0;
    }

    static inline int decodeCsiRaw12(uint8_t* src, uint16_t* dst, int width, int height)
    {
        if(width & 0x1) {
            return -1;
        }
        int raw12_line_size = 3 * width / 2;
        for(size_t i = 0, j = 0; i < raw12_line_size * height; i+=3, j+=2)
        {
            //[A4 - A11] | [B4 - B11] | [A0A1A2A3-B0B1B2B3]
            dst[j + 0] = ((uint16_t)src[i + 0] << 4) | ((src[i + 2] & 0x0f)  >> 0);
            dst[j + 1] = ((uint16_t)src[i + 1] << 4) | ((src[i + 2] & 0xf0)  >> 4);
        }
        return 0;
    }

    static inline int parseCsiRaw10(uint8_t* src, uint16_t* dst, int width, int height)
    {
        decodeCsiRaw10(src, dst, width, height);
        return 0;
    }

    static inline int parseCsiRaw12(uint8_t* src, uint16_t* dst, int width, int height)
    {
        decodeCsiRaw12(src, dst, width, height);
        return 0;
    }

    static std::vector<distortion_data> depth_dist_map_list;
    static bool addDepthDistortionMap(TY_CAMERA_CALIB_INFO& depth_calib, int width, int height)
    {
      bool has = false;
      for(size_t i = 0; i < depth_dist_map_list.size(); i++) {
        if((0 == memcmp(&depth_calib, static_cast<const void*>(&depth_dist_map_list[i].get_calib_data()), sizeof(TY_CAMERA_CALIB_INFO))) &&
          (width == depth_dist_map_list[i].get_map_width()) &&
          (height == depth_dist_map_list[i].get_map_height()))
        {
          has = true;
          break;
        }
      }

      if(!has) {
        cv::Mat mapX, mapY;
        std::vector<float> f_intrinsic;
        f_intrinsic.resize(9);
        f_intrinsic[0] = depth_calib.intrinsic.data[0] * width / depth_calib.intrinsicWidth;
        f_intrinsic[1] = 0;
        f_intrinsic[2] = depth_calib.intrinsic.data[2] * width / depth_calib.intrinsicWidth;

        f_intrinsic[3] = 0;
        f_intrinsic[4] = depth_calib.intrinsic.data[4] * height / depth_calib.intrinsicHeight;
        f_intrinsic[5] = depth_calib.intrinsic.data[5] * height / depth_calib.intrinsicHeight;
        
        f_intrinsic[6] = 0;
        f_intrinsic[7] = 0;
        f_intrinsic[8] = 1;

        cv::Mat intrinsic = cv::Mat(cv::Size(3,3), CV_32F, &f_intrinsic[0]);
        cv::Mat distCoeffs = cv::Mat(cv::Size(1, 12), CV_32F, depth_calib.distortion.data);
        cv::initUndistortRectifyMap(intrinsic, distCoeffs, cv::Mat(), intrinsic, cv::Size(width, height), CV_32FC1, mapX, mapY);

        distortion_data temp = distortion_data(depth_calib, mapX, mapY);
        depth_dist_map_list.push_back(temp);
        return true;
      }

      return false;
    }

    static int doDepthUndistortion(const TY_CAMERA_CALIB_INFO* depth_calib,
                                percipio::VideoFrameData* src, 
                                percipio::VideoFrameData* dst)
    {
      if(src->getPixelFormat() != TY_PIXEL_FORMAT_DEPTH16) {
        printf("%s <%d> Invalid pixel format!\n", __FILE__, __LINE__);
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

      for(size_t i = 0; i < depth_dist_map_list.size(); i++) {
        if((0 == memcmp(depth_calib, static_cast<const void*>(&depth_dist_map_list[i].get_calib_data()), sizeof(TY_CAMERA_CALIB_INFO))) &&
          (depth_dist_map_list[i].get_map_width() == src_image.width) &&
          (depth_dist_map_list[i].get_map_height() == src_image.height)) {
          cv::Mat depth           = cv::Mat(cv::Size(src_image.width, src_image.height), CV_16U, src_image.buffer);
          cv::Mat undistory_depth = cv::Mat(cv::Size(dst_image.width, dst_image.height), CV_16U, dst_image.buffer);
          cv::remap(depth, undistory_depth, depth_dist_map_list[i].get_map_x(), depth_dist_map_list[i].get_map_y(), cv::INTER_NEAREST);
          return TY_STATUS_OK;
        }
      }
      ROS_WARN("doDepthUndistortion fail!");
      return TY_STATUS_ERROR;
    }


    static std::vector<distortion_data> color_dist_map_list;
    static bool addColorDistortionMap(TY_CAMERA_CALIB_INFO& color_calib, int width, int height)
    {
      bool has = false;
      for(size_t i = 0; i < color_dist_map_list.size(); i++) {
        if(0 == memcmp(&color_calib, &color_dist_map_list[i], sizeof(TY_CAMERA_CALIB_INFO))) {
          has = true;
          break;
        }
      }

      if(!has) {
        cv::Mat mapX, mapY;
        std::vector<float> f_intrinsic;
        f_intrinsic.resize(9);
        f_intrinsic[0] = color_calib.intrinsic.data[0] * width / color_calib.intrinsicWidth;
        f_intrinsic[1] = 0;
        f_intrinsic[2] = color_calib.intrinsic.data[2] * width / color_calib.intrinsicWidth;

        f_intrinsic[3] = 0;
        f_intrinsic[4] = color_calib.intrinsic.data[4] * height / color_calib.intrinsicHeight;
        f_intrinsic[5] = color_calib.intrinsic.data[5] * height / color_calib.intrinsicHeight;
        
        f_intrinsic[6] = 0;
        f_intrinsic[7] = 0;
        f_intrinsic[8] = 1;

        cv::Mat intrinsic = cv::Mat(cv::Size(3,3), CV_32F, &f_intrinsic[0]);
        cv::Mat distCoeffs = cv::Mat(cv::Size(1, 12), CV_32F, color_calib.distortion.data);

        cv::initUndistortRectifyMap(intrinsic, distCoeffs, cv::Mat(), intrinsic, cv::Size(width, height), CV_32FC1, mapX, mapY);

        distortion_data temp = distortion_data(color_calib, mapX, mapY);
        color_dist_map_list.push_back(temp);
        return true;
      }

      return false;
    }

    static int doRGBUndistortion(const TY_CAMERA_CALIB_INFO* color_calib, 
                                percipio::VideoFrameData* src, 
                                percipio::VideoFrameData* dst)
    {
      if(src->getPixelFormat() != TY_PIXEL_FORMAT_RGB &&
         src->getPixelFormat() != TY_PIXEL_FORMAT_MONO) {
        printf("%s <%d> Invalid pixel format!\n", __FILE__, __LINE__);
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

      for(size_t i = 0; i < color_dist_map_list.size(); i++) {
        if(0 == memcmp(color_calib, &color_dist_map_list[i], sizeof(TY_CAMERA_CALIB_INFO))) {
          cv::Mat color, undistory_color;
          if(src->getPixelFormat() == TY_PIXEL_FORMAT_RGB) {
            color           = cv::Mat(cv::Size(src_image.width, src_image.height), CV_8UC3, src_image.buffer);
            undistory_color = cv::Mat(cv::Size(dst_image.width, dst_image.height), CV_8UC3, dst_image.buffer);
          } else {
            color           = cv::Mat(cv::Size(src_image.width, src_image.height), CV_8UC1, src_image.buffer);
            undistory_color = cv::Mat(cv::Size(dst_image.width, dst_image.height), CV_8UC1, dst_image.buffer);
          }
          cv::remap(color, undistory_color, color_dist_map_list[i].get_map_x(), color_dist_map_list[i].get_map_y(), cv::INTER_LINEAR);
          return TY_STATUS_OK;
        }
      }
      ROS_WARN("doRGBUndistortion fail!");
      return TY_STATUS_ERROR;
    }

    static int MapDepthImageToColorCoordinate(const TY_CAMERA_CALIB_INFO* depth_calib, 
                                              const TY_CAMERA_CALIB_INFO* color_calib, 
                                              int target_width,
                                              int target_height,
                                              percipio::VideoFrameData* src, 
                                              percipio::VideoFrameData* dst,
                                              float f_scale_unit)
    {
      if(src->getPixelFormat() != TY_PIXEL_FORMAT_DEPTH16) {
        printf("%s <%d> Invalid pixel format!\n", __FILE__, __LINE__);
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

      TY_IMAGE_DATA src_image, dst_image;
      src_image.width  = src->getWidth();
      src_image.height = src->getHeight();
      src_image.pixelFormat = src->getPixelFormat();
      src_image.buffer = src->getData();

      dst_image.width  = dst->getWidth();
      dst_image.height = dst->getHeight();
      dst_image.pixelFormat = dst->getPixelFormat();
      dst_image.buffer = dst->getData();

      return  TYMapDepthImageToColorCoordinate(
                  depth_calib,
                  src_image.width, src_image.height, (const uint16_t*)src_image.buffer,
                  color_calib,
                  dst_image.width, dst_image.height, (uint16_t* )dst_image.buffer, 
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
      //TODO
    }

    static int cvtColor(percipio::VideoFrameData& src, percipio::VideoFrameData& dst)
    {
      int i, j;
      dst.setTimestamp(src.getTimestamp());
      dst.setWidth(src.getWidth());
      dst.setHeight(src.getHeight());
      dst.setFrameIndex(src.getFrameIndex());
      dst.setComponentID(src.getComponentID());
      dst.setPixelFormat(TY_PIXEL_FORMAT_RGB);
      dst.Resize(3 * src.getWidth() * src.getHeight());

      int width = src.getWidth();
      int height = src.getHeight();
      int size = src.getDataSize();
      void* src_buffer = src.getData();
      void* dst_buffer = dst.getData();

      cv::Mat src_mat, dst_mat;
      switch (src.getPixelFormat()) {
        case TY_PIXEL_FORMAT_YUYV: {
          src_mat = cv::Mat(height, width, CV_8UC2, src_buffer);
          dst_mat = cv::Mat(height, width, CV_8UC3, dst_buffer);
          cv::cvtColor(src_mat, dst_mat, cv::COLOR_YUV2RGB_YUYV);
          break;
        }
        case TY_PIXEL_FORMAT_YVYU: {
          src_mat = cv::Mat(height, width, CV_8UC2, src_buffer);
          dst_mat = cv::Mat(height, width, CV_8UC3, dst_buffer);
          cv::cvtColor(src_mat, dst_mat, cv::COLOR_YUV2RGB_YVYU);
          break;
        }    
        case TY_PIXEL_FORMAT_JPEG: {
          std::vector<uchar> _v((uchar*)src_buffer, (uchar*)src_buffer + size);
          dst_mat = cv::Mat(height, width, CV_8UC3, dst_buffer);
          cv::Mat bgr = cv::imdecode(_v, cv::IMREAD_COLOR);
          cv::cvtColor(bgr, dst_mat, cv::COLOR_BGR2RGB);
          break;
        }
        case TY_PIXEL_FORMAT_BAYER8GBRG: {
          src_mat = cv::Mat(height, width, CV_8U, src_buffer);
          dst_mat = cv::Mat(height, width, CV_8UC3, dst_buffer);
          cv::cvtColor(src_mat, dst_mat, cv::COLOR_BayerGR2RGB);
          break;
        }
        case TY_PIXEL_FORMAT_BAYER8BGGR: {
          src_mat = cv::Mat(height, width, CV_8U, src_buffer);
          dst_mat = cv::Mat(height, width, CV_8UC3, dst_buffer);
          cv::cvtColor(src_mat, dst_mat, cv::COLOR_BayerRG2RGB);
          break;
        }
        case TY_PIXEL_FORMAT_BAYER8GRBG: {
          src_mat = cv::Mat(height, width, CV_8U, src_buffer);
          dst_mat = cv::Mat(height, width, CV_8UC3, dst_buffer);
          cv::cvtColor(src_mat, dst_mat, cv::COLOR_BayerGB2RGB);
          break;
        }
        case TY_PIXEL_FORMAT_BAYER8RGGB: {
          src_mat = cv::Mat(height, width, CV_8U, src_buffer);
          dst_mat = cv::Mat(height, width, CV_8UC3, dst_buffer);
          cv::cvtColor(src_mat, dst_mat, cv::COLOR_BayerBG2RGB);
          break;
        }
        case TY_PIXEL_FORMAT_BGR: {
          src_mat = cv::Mat(height, width, CV_8UC3, src_buffer);
          dst_mat = cv::Mat(height, width, CV_8UC3, dst_buffer);
          cv::cvtColor(src_mat, dst_mat, cv::COLOR_BGR2RGB);
          break;
        }
        case TY_PIXEL_FORMAT_RGB: {
          memcpy(dst_buffer, src_buffer, 3*width*height);
        }
        case TY_PIXEL_FORMAT_MONO: {
          src_mat = cv::Mat(height, width, CV_8U, src_buffer);
          dst_mat = cv::Mat(height, width, CV_8UC3, dst_buffer);
          cv::cvtColor(src_mat, dst_mat, cv::COLOR_GRAY2RGB);
          break;
        }
        case TY_PIXEL_FORMAT_CSI_MONO10: {
          std::vector<uint16_t> mono10(width * height);
          std::vector<uint8_t> mono8(width * height);
          parseCsiRaw10((uint8_t*)src_buffer, &mono10[0], width, height);
          for(size_t idx = 0; idx < mono10.size(); idx++) {
            mono8[idx] = static_cast<uint8_t>(mono10[idx] >> 2);
          }
          src_mat = cv::Mat(height, width, CV_8U, &mono8[0]);
          dst_mat = cv::Mat(height, width, CV_8UC3, dst_buffer);
          cv::cvtColor(src_mat, dst_mat, cv::COLOR_GRAY2RGB);
          break;
        }
        case TY_PIXEL_FORMAT_CSI_BAYER10GBRG: {
          std::vector<uint16_t> bayer10(width * height);
          std::vector<uint8_t> bayer8(width * height);
          parseCsiRaw10((uint8_t*)src_buffer, &bayer10[0], width, height);
          for(size_t idx = 0; idx < bayer10.size(); idx++) {
            bayer8[idx] = static_cast<uint8_t>(bayer10[idx] >> 2);
          }
          src_mat = cv::Mat(height, width, CV_8U, &bayer8[0]);
          dst_mat = cv::Mat(height, width, CV_8UC3, dst_buffer);
          cv::cvtColor(src_mat, dst_mat, cv::COLOR_BayerGR2RGB);
          break;
        }
        case TY_PIXEL_FORMAT_CSI_BAYER10BGGR: {
          std::vector<uint16_t> bayer10(width * height);
          std::vector<uint8_t> bayer8(width * height);
          parseCsiRaw10((uint8_t*)src_buffer, &bayer10[0], width, height);
          for(size_t idx = 0; idx < bayer10.size(); idx++) {
            bayer8[idx] = static_cast<uint8_t>(bayer10[idx] >> 2);
          }
          src_mat = cv::Mat(height, width, CV_8U, &bayer8[0]);
          dst_mat = cv::Mat(height, width, CV_8UC3, dst_buffer);
          cv::cvtColor(src_mat, dst_mat, cv::COLOR_BayerRG2RGB);
          break;
        }
        case TY_PIXEL_FORMAT_CSI_BAYER10GRBG: {
          std::vector<uint16_t> bayer10(width * height);
          std::vector<uint8_t> bayer8(width * height);
          parseCsiRaw10((uint8_t*)src_buffer, &bayer10[0], width, height);
          for(size_t idx = 0; idx < bayer10.size(); idx++) {
            bayer8[idx] = static_cast<uint8_t>(bayer10[idx] >> 2);
          }
          src_mat = cv::Mat(height, width, CV_8U, &bayer8[0]);
          dst_mat = cv::Mat(height, width, CV_8UC3, dst_buffer);
          cv::cvtColor(src_mat, dst_mat, cv::COLOR_BayerGB2RGB);
          break;
        }
        case TY_PIXEL_FORMAT_CSI_BAYER10RGGB: {
          std::vector<uint16_t> bayer10(width * height);
          std::vector<uint8_t> bayer8(width * height);
          parseCsiRaw10((uint8_t*)src_buffer, &bayer10[0], width, height);
          for(size_t idx = 0; idx < bayer10.size(); idx++) {
            bayer8[idx] = static_cast<uint8_t>(bayer10[idx] >> 2);
          }
          src_mat = cv::Mat(height, width, CV_8U, &bayer8[0]);
          dst_mat = cv::Mat(height, width, CV_8UC3, dst_buffer);
          cv::cvtColor(src_mat, dst_mat, cv::COLOR_BayerBG2RGB);
          break;
        }
        case TY_PIXEL_FORMAT_CSI_MONO12: {
          std::vector<uint16_t> mono12(width * height);
          std::vector<uint8_t> mono8(width * height);
          parseCsiRaw12((uint8_t*)src_buffer, &mono12[0], width, height);
          for(size_t idx = 0; idx < mono12.size(); idx++) {
            mono8[idx] = static_cast<uint8_t>(mono12[idx] >> 4);
          }
          src_mat = cv::Mat(height, width, CV_8U, &mono8[0]);
          dst_mat = cv::Mat(height, width, CV_8UC3, dst_buffer);
          cv::cvtColor(src_mat, dst_mat, cv::COLOR_GRAY2RGB);
          break;
        }
        case TY_PIXEL_FORMAT_CSI_BAYER12GBRG: {
          std::vector<uint16_t> bayer12(width * height);
          std::vector<uint8_t> bayer8(width * height);
          parseCsiRaw12((uint8_t*)src_buffer, &bayer12[0], width, height);
          for(size_t idx = 0; idx < bayer12.size(); idx++) {
            bayer8[idx] = static_cast<uint8_t>(bayer12[idx] >> 4);
          }
          src_mat = cv::Mat(height, width, CV_8U, &bayer8[0]);
          dst_mat = cv::Mat(height, width, CV_8UC3, dst_buffer);
          cv::cvtColor(src_mat, dst_mat, cv::COLOR_BayerGR2RGB);
          break;
        }
        case TY_PIXEL_FORMAT_CSI_BAYER12BGGR: {
          std::vector<uint16_t> bayer12(width * height);
          std::vector<uint8_t> bayer8(width * height);
          parseCsiRaw12((uint8_t*)src_buffer, &bayer12[0], width, height);
          for(size_t idx = 0; idx < bayer12.size(); idx++) {
            bayer8[idx] = static_cast<uint8_t>(bayer12[idx] >> 4);
          }
          src_mat = cv::Mat(height, width, CV_8U, &bayer8[0]);
          dst_mat = cv::Mat(height, width, CV_8UC3, dst_buffer);
          cv::cvtColor(src_mat, dst_mat, cv::COLOR_BayerRG2RGB);
          break;
        }
        case TY_PIXEL_FORMAT_CSI_BAYER12GRBG: {
          std::vector<uint16_t> bayer12(width * height);
          std::vector<uint8_t> bayer8(width * height);
          parseCsiRaw12((uint8_t*)src_buffer, &bayer12[0], width, height);
          for(size_t idx = 0; idx < bayer12.size(); idx++) {
            bayer8[idx] = static_cast<uint8_t>(bayer12[idx] >> 4);
          }
          src_mat = cv::Mat(height, width, CV_8U, &bayer8[0]);
          dst_mat = cv::Mat(height, width, CV_8UC3, dst_buffer);
          cv::cvtColor(src_mat, dst_mat, cv::COLOR_BayerGB2RGB);
          break;
        }
        case TY_PIXEL_FORMAT_CSI_BAYER12RGGB: {
          std::vector<uint16_t> bayer12(width * height);
          std::vector<uint8_t> bayer8(width * height);
          parseCsiRaw12((uint8_t*)src_buffer, &bayer12[0], width, height);
          for(size_t idx = 0; idx < bayer12.size(); idx++) {
            bayer8[idx] = static_cast<uint8_t>(bayer12[idx] >> 4);
          }
          src_mat = cv::Mat(height, width, CV_8U, &bayer8[0]);
          dst_mat = cv::Mat(height, width, CV_8UC3, dst_buffer);
          cv::cvtColor(src_mat, dst_mat, cv::COLOR_BayerBG2RGB);
          break;
        }    
        case TY_PIXEL_FORMAT_MONO16:
        case TY_PIXEL_FORMAT_TOF_IR_MONO16: {
          uint16_t* ptr = (uint16_t*)src_buffer;
          std::vector<uint8_t> mono8(width * height);
          for(size_t idx = 0; idx < width * height; idx++) {
            mono8[idx] = ptr[idx] >> 8;
          }
          src_mat = cv::Mat(height, width, CV_8U, &mono8[0]);
          dst_mat = cv::Mat(height, width, CV_8UC3, dst_buffer);
          cv::cvtColor(src_mat, dst_mat, cv::COLOR_GRAY2RGB);
          break;
        }
        default:
          break;
      }

      return 0;
    }

};

std::vector<distortion_data> ImgProc::depth_dist_map_list;
std::vector<distortion_data> ImgProc::color_dist_map_list;
